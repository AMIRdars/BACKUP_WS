/*
 * place_server.cpp
 *
 * behavior_tree_msgs::action::Place のアクションサーバー。
 * BT の <PlaceAmir action_name="place" pose_obj="{pose_obj}"/> から呼ばれる。
 *
 * Goal   : geometry_msgs/PoseStamped pose_obj  (置き位置の座標)
 * Result : string error_string                 ("success" or "failure")
 *
 * 動作シーケンス:
 *   1. アプローチ (置き位置の上方)
 *   2. 降下 (置き位置)
 *   3. グリッパーを開く (リリース)
 *   4. 退避 (上方へ戻る)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behavior_tree_msgs/action/place.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using Place              = behavior_tree_msgs::action::Place;
using GoalHandlePlace    = rclcpp_action::ServerGoalHandle<Place>;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using GripperCommand     = control_msgs::action::GripperCommand;
using GripperClient      = rclcpp_action::Client<GripperCommand>;

// ─────────────────────────────────────────────────────────────────
// 設定パラメータ
// ─────────────────────────────────────────────────────────────────
constexpr double APPROACH_HEIGHT = 0.15;   // アプローチ上方オフセット [m]
constexpr double GRIPPER_OPEN    = -1.0;
constexpr double FIXED_JOINT_5   = 0.0;

// ─────────────────────────────────────────────────────────────────
// グリッパー制御
// ─────────────────────────────────────────────────────────────────
static void controlGripper(
    rclcpp::Node::SharedPtr node,
    GripperClient::SharedPtr client,
    double position)
{
    (void)node;
    if (!client->wait_for_action_server(5s)) return;
    auto goal = GripperCommand::Goal();
    goal.command.position   = position;
    goal.command.max_effort = 50.0;
    auto gh = client->async_send_goal(goal).get();
    if (gh) client->async_get_result(gh).get();
}

// ─────────────────────────────────────────────────────────────────
// MoveIt2 ヘルパー (Joint_4/Joint_5 拘束付き直線移動)
// ─────────────────────────────────────────────────────────────────
static bool moveConstrained(MoveGroupInterface & arm, double x, double y, double z, double speed)
{
    arm.setStartStateToCurrentState();

    moveit_msgs::msg::Constraints c;
    moveit_msgs::msg::JointConstraint jc4, jc5;
    jc4.joint_name = "Joint_4"; jc4.position = 0.0;
    jc4.tolerance_above = 0.5; jc4.tolerance_below = 0.5; jc4.weight = 1.0;
    jc5.joint_name = "Joint_5"; jc5.position = FIXED_JOINT_5;
    jc5.tolerance_above = 0.05; jc5.tolerance_below = 0.05; jc5.weight = 1.0;
    c.joint_constraints.push_back(jc4);
    c.joint_constraints.push_back(jc5);
    arm.setPathConstraints(c);

    arm.setPositionTarget(x, y, z);
    arm.setGoalPositionTolerance(0.01);
    arm.setMaxVelocityScalingFactor(speed);
    arm.setMaxAccelerationScalingFactor(speed * 0.5);

    auto result = arm.move();
    arm.clearPathConstraints();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("place_server"), "Move FAILED (%.3f %.3f %.3f)", x, y, z);
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────
// PlaceServer クラス
// ─────────────────────────────────────────────────────────────────
class PlaceServer : public rclcpp::Node
{
public:
    explicit PlaceServer(const rclcpp::NodeOptions & options)
    : Node("place_server", options)
    {
        using namespace std::placeholders;

        // 相対名: ノードの namespace で /<ns>/gripper_controller/... に解決される
        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "gripper_controller/gripper_cmd");

        action_server_ = rclcpp_action::create_server<Place>(
            this, "place",
            std::bind(&PlaceServer::handle_goal,     this, _1, _2),
            std::bind(&PlaceServer::handle_cancel,   this, _1),
            std::bind(&PlaceServer::handle_accepted, this, _1));

        RCLCPP_INFO(get_logger(), "place_server 起動完了");
    }

private:
    rclcpp_action::Server<Place>::SharedPtr action_server_;
    GripperClient::SharedPtr                gripper_client_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Place::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "プレースゴール受信: (%.3f, %.3f, %.3f)",
            goal->pose_obj.pose.position.x,
            goal->pose_obj.pose.position.y,
            goal->pose_obj.pose.position.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePlace>)
    {
        RCLCPP_WARN(get_logger(), "キャンセル要求を受信");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandlePlace> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result     = std::make_shared<Place::Result>();

        const double px = goal->pose_obj.pose.position.x;
        const double py = goal->pose_obj.pose.position.y;
        const double pz = goal->pose_obj.pose.position.z;

        RCLCPP_INFO(get_logger(), "Place 実行開始: (%.3f, %.3f, %.3f)", px, py, pz);

        MoveGroupInterface arm(shared_from_this(), "arm");
        arm.setPoseReferenceFrame("base_footprint");
        arm.setPlanningTime(30.0);
        arm.setNumPlanningAttempts(5);

        auto send_failure = [&](const std::string & msg) {
            RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
            result->error_string = "failure: " + msg;
            goal_handle->succeed(result);
        };

        auto check_cancel = [&]() -> bool {
            if (goal_handle->is_canceling()) {
                result->error_string = "canceled";
                goal_handle->canceled(result);
                return true;
            }
            return false;
        };

        // 1. アプローチ (置き位置の上方)
        if (!moveConstrained(arm, px, py, pz + APPROACH_HEIGHT, 0.5))
            return send_failure("アプローチ失敗");
        if (check_cancel()) return;

        // 2. 降下 (置き位置)
        if (!moveConstrained(arm, px, py, pz, 0.2))
            return send_failure("降下失敗");
        if (check_cancel()) return;

        // 3. グリッパーを開く (リリース)
        controlGripper(shared_from_this(), gripper_client_, GRIPPER_OPEN);
        rclcpp::sleep_for(500ms);
        if (check_cancel()) return;

        // 4. 退避 (上方へ戻る)
        if (!moveConstrained(arm, px, py, pz + APPROACH_HEIGHT, 0.4))
            return send_failure("退避失敗");

        RCLCPP_INFO(get_logger(), "Place 完了");
        result->error_string = "success";
        goal_handle->succeed(result);
    }
};

// ─────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({{"use_sim_time", true}});

    auto node = std::make_shared<PlaceServer>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
