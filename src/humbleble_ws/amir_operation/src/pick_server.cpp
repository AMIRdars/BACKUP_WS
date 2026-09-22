/*
 * pick_server.cpp
 *
 * behavior_tree_msgs::action::Pick のアクションサーバー。
 * BT の <PickAmir action_name="pick" pose_obj="{pose_obj}"/> から呼ばれる。
 *
 * Goal   : geometry_msgs/PoseStamped pose_obj  (把持対象の座標)
 * Result : string error_string                 ("success" or "failure")
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behavior_tree_msgs/action/pick.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using Pick               = behavior_tree_msgs::action::Pick;
using GoalHandlePick     = rclcpp_action::ServerGoalHandle<Pick>;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using GripperCommand     = control_msgs::action::GripperCommand;
using GripperClient      = rclcpp_action::Client<GripperCommand>;

// ─────────────────────────────────────────────────────────────────
// 設定パラメータ
// ─────────────────────────────────────────────────────────────────
constexpr double APPROACH_HEIGHT        = 0.15;   // アプローチ上方オフセット [m]
constexpr double GRIPPER_OPEN           = -1.0;
constexpr double GRIPPER_CLOSE          =  0.1;
constexpr double GRIPPER_MAX_POS        =  0.261799;
constexpr double GRIPPER_PRELOAD        =  0.10;
constexpr double GRIPPER_STALL_THRESHOLD = 0.01;
constexpr int    GRIPPER_CLOSE_STEPS    = 25;
constexpr int    GRIPPER_CLOSE_STEP_MS  = 100;
constexpr int    GRIPPER_DETECT_START   = 5;
constexpr double FIXED_JOINT_5          = 0.0;

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

static void closeGripperGradually(
    rclcpp::Node::SharedPtr node,
    GripperClient::SharedPtr client,
    double target_pos = GRIPPER_CLOSE)
{
    using JointState = sensor_msgs::msg::JointState;

    if (!client->wait_for_action_server(5s)) return;

    std::atomic<double> g_pos{GRIPPER_OPEN};
    auto js_sub = node->create_subscription<JointState>(
        "joint_states", rclcpp::QoS(10),
        [&g_pos](JointState::ConstSharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "Gripper") {
                    if (i < msg->position.size()) g_pos.store(msg->position[i]);
                    break;
                }
            }
        });

    rclcpp::sleep_for(200ms);
    double start_pos  = g_pos.load();
    double range      = target_pos - start_pos;
    bool   contacted  = false;
    double contact_pos = target_pos;
    double prev_actual = start_pos;

    for (int step = 1; step <= GRIPPER_CLOSE_STEPS && rclcpp::ok(); ++step) {
        double cmd_pos = start_pos + range * static_cast<double>(step) / GRIPPER_CLOSE_STEPS;
        auto goal = GripperCommand::Goal();
        goal.command.position   = cmd_pos;
        goal.command.max_effort = 50.0;
        client->async_send_goal(goal);
        rclcpp::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_STEP_MS));

        double actual   = g_pos.load();
        double movement = std::abs(actual - prev_actual);

        if (step >= GRIPPER_DETECT_START && movement < GRIPPER_STALL_THRESHOLD) {
            contact_pos = actual;
            contacted   = true;
            RCLCPP_INFO(node->get_logger(), "Gripper 接触検知 step=%d actual=%.3f", step, actual);
            break;
        }
        prev_actual = actual;
    }

    double hold_target = contacted
        ? std::min(contact_pos + GRIPPER_PRELOAD, GRIPPER_MAX_POS)
        : target_pos;

    auto hold = GripperCommand::Goal();
    hold.command.position   = hold_target;
    hold.command.max_effort = 100.0;
    client->async_send_goal(hold).get();
    rclcpp::sleep_for(500ms);
}

// ─────────────────────────────────────────────────────────────────
// MoveIt2 ヘルパー
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
        RCLCPP_ERROR(rclcpp::get_logger("pick_server"), "Move FAILED (%.3f %.3f %.3f)", x, y, z);
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────
// PickServer クラス
// ─────────────────────────────────────────────────────────────────
class PickServer : public rclcpp::Node
{
public:
    explicit PickServer(const rclcpp::NodeOptions & options)
    : Node("pick_server", options)
    {
        using namespace std::placeholders;

        // 相対名: ノードの namespace で /<ns>/gripper_controller/... に解決される
        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "gripper_controller/gripper_cmd");

        action_server_ = rclcpp_action::create_server<Pick>(
            this, "pick",
            std::bind(&PickServer::handle_goal,     this, _1, _2),
            std::bind(&PickServer::handle_cancel,   this, _1),
            std::bind(&PickServer::handle_accepted, this, _1));

        RCLCPP_INFO(get_logger(), "pick_server 起動完了");
    }

private:
    rclcpp_action::Server<Pick>::SharedPtr action_server_;
    GripperClient::SharedPtr               gripper_client_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Pick::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "ピックゴール受信: (%.3f, %.3f, %.3f)",
            goal->pose_obj.pose.position.x,
            goal->pose_obj.pose.position.y,
            goal->pose_obj.pose.position.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePick>)
    {
        RCLCPP_WARN(get_logger(), "キャンセル要求を受信");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
        // MoveIt2 の move() はブロッキングなので別スレッドで実行
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result     = std::make_shared<Pick::Result>();

        const double ox = goal->pose_obj.pose.position.x;
        const double oy = goal->pose_obj.pose.position.y;
        const double oz = goal->pose_obj.pose.position.z;

        RCLCPP_INFO(get_logger(), "Pick 実行開始: (%.3f, %.3f, %.3f)", ox, oy, oz);

        // MoveGroupInterface はスピン中のノードが必要なため execute() 内で生成
        MoveGroupInterface arm(shared_from_this(), "arm");
        arm.setPoseReferenceFrame("base_footprint");
        arm.setPlanningTime(30.0);
        arm.setNumPlanningAttempts(5);

        auto send_failure = [&](const std::string & msg) {
            RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
            result->error_string = "failure: " + msg;
            goal_handle->succeed(result);
        };

        // キャンセル確認マクロ
        auto check_cancel = [&]() -> bool {
            if (goal_handle->is_canceling()) {
                result->error_string = "canceled";
                goal_handle->canceled(result);
                return true;
            }
            return false;
        };

        // 1. グリッパーを開く
        controlGripper(shared_from_this(), gripper_client_, GRIPPER_OPEN);
        if (check_cancel()) return;

        // 2. アプローチ（上方）
        if (!moveConstrained(arm, ox, oy, oz + APPROACH_HEIGHT, 0.5))
            return send_failure("アプローチ失敗");
        if (check_cancel()) return;

        // 3. 降下
        if (!moveConstrained(arm, ox, oy, oz, 0.2))
            return send_failure("降下失敗");
        if (check_cancel()) return;

        // 4. 把持
        closeGripperGradually(shared_from_this(), gripper_client_, GRIPPER_CLOSE);
        if (check_cancel()) return;

        // 5. 持ち上げ
        if (!moveConstrained(arm, ox, oy, oz + APPROACH_HEIGHT, 0.3))
            return send_failure("持ち上げ失敗");

        RCLCPP_INFO(get_logger(), "Pick 完了");
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

    auto node = std::make_shared<PickServer>(options);

    // MoveIt2 は MultiThreadedExecutor が必要
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
