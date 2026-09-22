// VR/MR テレオペ用ブリッジノード (SDK 非依存・汎用版)
//
// VR コントローラの姿勢 (geometry_msgs/PoseStamped) を購読し、
// デッドマン(/vr/enable, std_msgs/Bool)が true の間だけ、
// 「有効化した瞬間の姿勢」を基準とした変位に比例する手先 Twist を生成して
// /servo_node/delta_twist_cmds (TwistStamped) に publish する。
//   ・コントローラを基準位置から動かすほど速く動く「ばね/クラッチ式」マッピング
//   ・離す(enable=false)と即停止
// これにより VR の絶対姿勢ドリフトに影響されず安定して操作できる。
//
// VR SDK 側は手姿勢を /vr/controller_pose (PoseStamped) として publish するだけでよい。
// Quest/OpenXR/SteamVR いずれでも、その出力をこのトピックに繋げば動く。
//
// 事前に必要 (Servo出力先を有効化):
//   ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
//
// 実行:
//   ros2 run amir_operation vr_twist_bridge
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class VrTwistBridge : public rclcpp::Node
{
public:
  explicit VrTwistBridge(const rclcpp::NodeOptions & options)
  : Node("vr_twist_bridge", options)
  {
    // 入出力トピック
    pose_topic_   = declare_parameter<std::string>("pose_topic", "/vr/controller_pose");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/vr/enable");
    twist_topic_  = declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    command_frame_ = declare_parameter<std::string>("command_frame", "base_footprint");

    // 変位→速度のゲインと上限 (speed_units 前提: 出力は m/s, rad/s)
    lin_gain_ = declare_parameter<double>("linear_gain", 2.0);     // [1/s] : 変位[m]→速度[m/s]
    ang_gain_ = declare_parameter<double>("angular_gain", 2.0);    // [1/s] : 角変位[rad]→角速度[rad/s]
    max_lin_  = declare_parameter<double>("max_linear", 0.25);     // [m/s]
    max_ang_  = declare_parameter<double>("max_angular", 0.8);     // [rad/s]
    deadband_lin_ = declare_parameter<double>("deadband_linear", 0.01);   // [m]
    deadband_ang_ = declare_parameter<double>("deadband_angular", 0.02);  // [rad]
    // VR座標 → ロボット base_footprint 座標 の軸対応。
    // 既定は恒等。MR で実機に重ねる場合はキャリブで上書き or 別途TF変換を推奨。

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SystemDefaultsQoS());

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VrTwistBridge::on_pose, this, std::placeholders::_1));

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10,
      std::bind(&VrTwistBridge::on_enable, this, std::placeholders::_1));

    start_client_ = create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    // 100 Hz で Twist を出力 (incoming_command_timeout より十分速く)
    timer_ = create_wall_timer(10ms, std::bind(&VrTwistBridge::publish_twist, this));

    RCLCPP_INFO(get_logger(),
      "vr_twist_bridge 起動: pose=%s enable=%s → twist=%s",
      pose_topic_.c_str(), enable_topic_.c_str(), twist_topic_.c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_pose_ = *msg;
    have_pose_ = true;
  }

  void on_enable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !enabled_) {
      // 立ち上がり: Servo 開始 & 基準姿勢を取得
      ensure_servo_started();
      if (have_pose_) {
        ref_pose_ = last_pose_;
        have_ref_ = true;
        RCLCPP_INFO(get_logger(), "🟢 teleop 有効化 (基準姿勢を取得)");
      } else {
        have_ref_ = false;
        RCLCPP_WARN(get_logger(), "🟡 enable=true だが pose 未受信。pose待ち。");
      }
    } else if (!msg->data && enabled_) {
      RCLCPP_INFO(get_logger(), "🔴 teleop 無効化 (停止)");
      have_ref_ = false;
    }
    enabled_ = msg->data;
  }

  void ensure_servo_started()
  {
    if (servo_started_) return;
    if (!start_client_->wait_for_service(0s)) {
      RCLCPP_WARN(get_logger(), "/servo_node/start_servo 未提供。Servo起動を確認してください。");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_client_->async_send_request(req);
    servo_started_ = true;
    RCLCPP_INFO(get_logger(), "start_servo を要求しました。");
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  void publish_twist()
  {
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = command_frame_;

    if (enabled_ && have_ref_ && have_pose_) {
      // 並進変位 (VR座標系。command_frame と一致している前提)
      double dx = last_pose_.pose.position.x - ref_pose_.pose.position.x;
      double dy = last_pose_.pose.position.y - ref_pose_.pose.position.y;
      double dz = last_pose_.pose.position.z - ref_pose_.pose.position.z;

      // 回転変位: q_err = q_cur * q_ref^-1 を角度×軸ベクトルに変換
      tf2::Quaternion q_cur(
        last_pose_.pose.orientation.x, last_pose_.pose.orientation.y,
        last_pose_.pose.orientation.z, last_pose_.pose.orientation.w);
      tf2::Quaternion q_ref(
        ref_pose_.pose.orientation.x, ref_pose_.pose.orientation.y,
        ref_pose_.pose.orientation.z, ref_pose_.pose.orientation.w);
      tf2::Quaternion q_err = q_cur * q_ref.inverse();
      q_err.normalize();
      double angle = q_err.getAngle();          // [0, 2pi]
      if (angle > M_PI) angle -= 2.0 * M_PI;     // [-pi, pi] に正規化
      tf2::Vector3 axis = q_err.getAxis();
      double rx = axis.x() * angle;
      double ry = axis.y() * angle;
      double rz = axis.z() * angle;

      // デッドバンド
      auto db = [](double v, double d) { return (std::abs(v) < d) ? 0.0 : v; };
      dx = db(dx, deadband_lin_); dy = db(dy, deadband_lin_); dz = db(dz, deadband_lin_);
      rx = db(rx, deadband_ang_); ry = db(ry, deadband_ang_); rz = db(rz, deadband_ang_);

      out.twist.linear.x  = clamp(lin_gain_ * dx, -max_lin_, max_lin_);
      out.twist.linear.y  = clamp(lin_gain_ * dy, -max_lin_, max_lin_);
      out.twist.linear.z  = clamp(lin_gain_ * dz, -max_lin_, max_lin_);
      out.twist.angular.x = clamp(ang_gain_ * rx, -max_ang_, max_ang_);
      out.twist.angular.y = clamp(ang_gain_ * ry, -max_ang_, max_ang_);
      out.twist.angular.z = clamp(ang_gain_ * rz, -max_ang_, max_ang_);
    }
    // enabled でなければ全ゼロ (= 停止指令)
    twist_pub_->publish(out);
  }

  std::string pose_topic_, enable_topic_, twist_topic_, command_frame_;
  double lin_gain_, ang_gain_, max_lin_, max_ang_, deadband_lin_, deadband_ang_;

  bool have_pose_{false}, have_ref_{false}, enabled_{false}, servo_started_{false};
  geometry_msgs::msg::PoseStamped last_pose_, ref_pose_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<VrTwistBridge>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
