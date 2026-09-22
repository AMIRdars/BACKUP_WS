/*
 * move_meca_server.cpp
 *
 * behavior_tree_msgs::action::MoveMeca のアクションサーバー。
 * BT の <MoveMeca action_name="amir/move_meca" posi_x="{posi_x}" posi_y="{posi_y}"/> から呼ばれる。
 *
 * Goal   : float64 posi_x, float64 posi_y  (ワールド座標系の目標位置)
 * Result : string error_string             ("success" or "failure")
 *
 * 制御: /mecanum_drive_controller/odometry から現在位置を取得し、
 *       P制御で /rover_twist (Twist, ロボット座標系) を配信して目標に誘導する。
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behavior_tree_msgs/action/move_meca.hpp>
#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using MoveMeca           = behavior_tree_msgs::action::MoveMeca;
using GoalHandleMoveMeca = rclcpp_action::ServerGoalHandle<MoveMeca>;

class MoveMecaServer : public rclcpp::Node
{
public:
  MoveMecaServer() : Node("move_meca_server")
  {
    // SensorDataQoS (best_effort) で subscribe: ros2_controllers が best_effort で
    // publishする場合でも受信できるよう互換性を確保する
    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_ = *msg;
        odom_received_.store(true);
      });

    // 相対名: ノードの namespace で /<ns>/odom, /<ns>/rover_twist に解決される
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("rover_twist", rclcpp::QoS(10));

    action_server_ = rclcpp_action::create_server<MoveMeca>(
      this, "amir/move_meca",
      std::bind(&MoveMecaServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveMecaServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&MoveMecaServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "move_meca_server 起動完了");
    RCLCPP_INFO(get_logger(), "  odom:  /odom");
    RCLCPP_INFO(get_logger(), "  twist: /rover_twist");
    RCLCPP_INFO(get_logger(), "  action: amir/move_meca");
  }

private:
  static constexpr double KP             = 0.8;   // 位置 P ゲイン
  static constexpr double GOAL_TOLERANCE = 0.10;  // 目標到達判定 [m]
  static constexpr double MAX_VEL        = 0.40;  // 最大速度 [m/s]

  // ── ヨー角をクォータニオンから取得 ──────────────────────────────
  static double getYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // ── アクションコールバック ───────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveMeca::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Goal 受信: x=%.3f, y=%.3f", goal->posi_x, goal->posi_y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveMeca> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "キャンセル要求受信");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveMeca> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // ── 実行ループ ───────────────────────────────────────────────────
  void execute(const std::shared_ptr<GoalHandleMoveMeca> goal_handle)
  {
    const auto goal   = goal_handle->get_goal();
    auto       result = std::make_shared<MoveMeca::Result>();

    // オドメトリが届くまで待機 (最大 30 秒)
    auto wait_start = std::chrono::steady_clock::now();
    while (!odom_received_.load() && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto elapsed = std::chrono::steady_clock::now() - wait_start;
      if (elapsed > std::chrono::seconds(30)) {
        RCLCPP_ERROR(get_logger(),
          "オドメトリタイムアウト (30s): /odom が届きません。"
          " mecanum_drive_controller と odom_tf_relay が起動しているか確認してください。");
        result->error_string = "failure";
        goal_handle->abort(result);
        return;
      }
      if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() % 3000 < 100) {
        RCLCPP_INFO(get_logger(), "オドメトリ待機中... (%.0f s)",
          std::chrono::duration<double>(elapsed).count());
      }
    }

    RCLCPP_INFO(get_logger(), "オドメトリ受信確認。移動開始します。");

    // 50 Hz 制御ループ
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop();
        result->error_string = "canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "キャンセル完了");
        return;
      }

      // オドメトリをスレッドセーフにコピー
      nav_msgs::msg::Odometry odom_snap;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_snap = odom_;
      }

      const double cx   = odom_snap.pose.pose.position.x;
      const double cy   = odom_snap.pose.pose.position.y;
      const double dx   = goal->posi_x - cx;
      const double dy   = goal->posi_y - cy;
      const double dist = std::hypot(dx, dy);

      if (dist < GOAL_TOLERANCE) {
        stop();
        result->error_string = "success";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "目標到達 (dist=%.3f m)", dist);
        return;
      }

      // ワールド座標系の速度指令をロボット座標系に変換
      const double yaw  = getYaw(odom_snap.pose.pose.orientation);
      const double vx_w = std::clamp(KP * dx, -MAX_VEL, MAX_VEL);
      const double vy_w = std::clamp(KP * dy, -MAX_VEL, MAX_VEL);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x =  vx_w * std::cos(yaw) + vy_w * std::sin(yaw);
      cmd.linear.y = -vx_w * std::sin(yaw) + vy_w * std::cos(yaw);
      twist_pub_->publish(cmd);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "移動中: 現在(%.2f, %.2f) -> 目標(%.2f, %.2f)  dist=%.2f m",
        cx, cy, goal->posi_x, goal->posi_y, dist);

      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50 Hz
    }

    stop();
    result->error_string = "failure";
    goal_handle->abort(result);
  }

  void stop()
  {
    twist_pub_->publish(geometry_msgs::msg::Twist{});
  }

  rclcpp_action::Server<MoveMeca>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry odom_;
  std::atomic<bool> odom_received_{false};  // スレッド間の可視性を保証
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor: サブスクリプションコールバックとアクションコールバックを
  // 並行処理できるようにし、execute() スレッドと干渉しない
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MoveMecaServer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
