/*
 * ROS 2 Humble / MoveIt2 向け Pick and Place
 * amir_mecanum3 (モバイルマニピュレーター) 対応版
 *
 * 【機能】:
 * spawn_multibox でスポーンされた複数の物体を順番に連続把持・配置する。
 * OBJ_POSITIONS は spawn_multibox のデフォルト引数 (count=4, y=-0.2, y_step=0.15) と一致。
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using MoveGroupInterface     = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using GripperCommand         = control_msgs::action::GripperCommand;
using GripperClient          = rclcpp_action::Client<GripperCommand>;

// ═══════════════════════════════════════════════════════════════
// 設定パラメータ
// ═══════════════════════════════════════════════════════════════

// アプローチ/降下時に固定する Joint_5 角度 [rad]
const double FIXED_JOINT_5_VALUE = 0.0;

const double APPROACH_HEIGHT = 0.15; // アプローチ上方オフセット [m]

// 把持対象物体の位置リスト (spawn_multibox デフォルト: count=4, x=0.5, y=-0.2, y_step=0.15, z=0.05)
// z は物体中心 0.05 + size_z/2 = 0.10 (グリッパーが掴む高さ)
struct Pose3D { double x, y, z; };
const std::vector<Pose3D> OBJ_POSITIONS = {
    {0.5, -0.20, 0.08},
    {0.5, -0.05, 0.08},
    {0.5,  0.10, 0.08},
    {0.5,  0.25, 0.08},
};

// 配置先リスト (物体同士が重ならないよう y を 0.15 ずつずらす)
const std::vector<Pose3D> PLACE_POSITIONS = {
    {-0.20, 0.40, 0.15},
    {-0.05, 0.40, 0.15},
    {0.10, 0.40, 0.15},
    {0.25, 0.40, 0.15},
};

// グリッパー
const double GRIPPER_OPEN  = -1.0;
const double GRIPPER_CLOSE =  0.1;

// 段階的グリッパー閉鎖の設定
const int    GRIPPER_CLOSE_STEPS        = 25;    // 分割ステップ数
const int    GRIPPER_CLOSE_STEP_MS      = 100;   // 1ステップあたりの待機時間 [ms]
// 失速検知: 前ステップから actual がこの量以下しか動かなければ接触と判断
// モーターの自由閉鎖時は ~0.04 rad/step 動く。物体接触時は ~0 rad/step になる。
const double GRIPPER_STALL_THRESHOLD    = 0.01;  // [rad/step]
const int    GRIPPER_DETECT_START_STEP  = 5;     // 何ステップ目から検知を開始するか
// 接触検知後に追加で押し込む量 [rad]。モーターが物体を押す力になる。
const double GRIPPER_PRELOAD            = 0.10;
const double GRIPPER_MAX_POS            = 0.261799; // グリッパー可動上限 [rad]

// ───────────────────────────────────────────────────────────────
// グリッパー制御 (開く / PLACE 時の開放)
// ───────────────────────────────────────────────────────────────
void controlGripper(
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

// ───────────────────────────────────────────────────────────────
// 段階的グリッパー閉鎖 + 接触検知
// ───────────────────────────────────────────────────────────────
void closeGripperGradually(
    rclcpp::Node::SharedPtr node,
    GripperClient::SharedPtr client,
    double target_pos = GRIPPER_CLOSE)
{
    using JointState = sensor_msgs::msg::JointState;

    if (!client->wait_for_action_server(5s)) return;

    std::atomic<double> g_pos{GRIPPER_OPEN};
    auto js_sub = node->create_subscription<JointState>(
        "/joint_states", rclcpp::QoS(10),
        [&g_pos](JointState::ConstSharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "Gripper") {
                    if (i < msg->position.size()) g_pos.store(msg->position[i]);
                    break;
                }
            }
        });

    rclcpp::sleep_for(200ms);
    double start_pos   = g_pos.load();
    double range       = target_pos - start_pos;
    bool   contacted   = false;
    double contact_pos = target_pos;

    RCLCPP_INFO(node->get_logger(),
        "Gripper 段階的閉じ開始 (start=%.3f → target=%.3f)", start_pos, target_pos);

    double prev_actual = start_pos;

    for (int step = 1; step <= GRIPPER_CLOSE_STEPS && rclcpp::ok(); ++step) {
        double cmd_pos = start_pos + range * static_cast<double>(step) / GRIPPER_CLOSE_STEPS;

        auto goal = GripperCommand::Goal();
        goal.command.position   = cmd_pos;
        goal.command.max_effort = 50.0;
        client->async_send_goal(goal);

        rclcpp::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_STEP_MS));

        double actual   = g_pos.load();
        double movement = std::abs(actual - prev_actual); // このステップで動いた量

        RCLCPP_DEBUG(node->get_logger(),
            "step=%d cmd=%.3f actual=%.3f movement=%.4f", step, cmd_pos, actual, movement);

        // 失速検知: 自由閉鎖中は ~0.04 rad/step 動く。物体に当たると動かなくなる。
        if (step >= GRIPPER_DETECT_START_STEP && movement < GRIPPER_STALL_THRESHOLD) {
            contact_pos = actual;
            contacted   = true;
            RCLCPP_INFO(node->get_logger(),
                "Gripper 失速検知(接触)! step=%d actual=%.3f movement=%.4f",
                step, actual, movement);
            break;
        }

        prev_actual = actual;
    }

    // 接触位置から PRELOAD 分だけ追加で押し込んで握力を確保する
    double hold_target = contacted
        ? std::min(contact_pos + GRIPPER_PRELOAD, GRIPPER_MAX_POS)
        : target_pos;

    auto hold = GripperCommand::Goal();
    hold.command.position   = hold_target;
    hold.command.max_effort = 100.0; // 保持時は高めの effort で固定
    client->async_send_goal(hold).get();

    RCLCPP_INFO(node->get_logger(), "Gripper 保持位置=%.3f", hold_target);

    // 握力が安定するまで待機
    rclcpp::sleep_for(500ms);
}

// ───────────────────────────────────────────────────────────────
// 【アプローチ/降下用】位置指定 + Joint_5 固定
// ───────────────────────────────────────────────────────────────
bool moveToPositionConstrained(
    MoveGroupInterface &group,
    double x, double y, double z,
    double speed_scaling)
{
    group.setStartStateToCurrentState();

    moveit_msgs::msg::Constraints constraints;
    // moveit_msgs::msg::JointConstraint jc;
    // jc.joint_name     = "Joint_5";
    // jc.position       = FIXED_JOINT_5_VALUE;
    // jc.tolerance_above = 0.05; // ±約3度の余裕を持たせてプランを通りやすくする
    // jc.tolerance_below = 0.05;
    // jc.weight         = 1.0;
    // constraints.joint_constraints.push_back(jc);
    // group.setPathConstraints(constraints);

    moveit_msgs::msg::JointConstraint jc4;
    jc4.joint_name      = "Joint_4";
    jc4.position        = 0.0;  // 中心値
    jc4.tolerance_above = 0.5;  // 上限方向への許容幅 (0.0 + 0.5 = 0.5)
    jc4.tolerance_below = 0.5;  // 下限方向への許容幅 (0.0 - 0.5 = -0.5)
    jc4.weight          = 1.0;
    constraints.joint_constraints.push_back(jc4);

    // Joint_5 の固定 (必要に応じてこちらも範囲制限に変更可能)
    moveit_msgs::msg::JointConstraint jc5;
    jc5.joint_name      = "Joint_5";
    jc5.position        = FIXED_JOINT_5_VALUE;
    jc5.tolerance_above = 0.05; // ほぼ固定
    jc5.tolerance_below = 0.05; // ほぼ固定
    jc5.weight          = 1.0;
    constraints.joint_constraints.push_back(jc5);

    group.setPathConstraints(constraints);


    group.setPositionTarget(x, y, z);
    group.setGoalPositionTolerance(0.01);
    group.setMaxVelocityScalingFactor(speed_scaling);
    group.setMaxAccelerationScalingFactor(speed_scaling * 0.5);

    RCLCPP_INFO(rclcpp::get_logger("pick_place"),
        "[Constrained] -> (%.3f, %.3f, %.3f) Joint_5=%.2f", x, y, z, FIXED_JOINT_5_VALUE);

    auto result = group.move();
    group.clearPathConstraints();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("pick_place"), "Move FAILED (Code: %d)", result.val);
        return false;
    }
    return true;
}

// ───────────────────────────────────────────────────────────────
// 【把持後の移動用】拘束なし・位置のみ指定
// 把持後は関節状態が変わるため Joint_5 拘束を外してプランを立てる。
// ───────────────────────────────────────────────────────────────
bool moveToPositionFree(
    MoveGroupInterface &group,
    double x, double y, double z,
    double speed_scaling)
{
    group.setStartStateToCurrentState();
    group.clearPathConstraints();

    group.setPositionTarget(x, y, z);
    group.setGoalPositionTolerance(0.01);
    group.setMaxVelocityScalingFactor(speed_scaling);
    group.setMaxAccelerationScalingFactor(speed_scaling * 0.5);

    RCLCPP_INFO(rclcpp::get_logger("pick_place"),
        "[Free] -> (%.3f, %.3f, %.3f)", x, y, z);

    auto result = group.move();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("pick_place"), "Move FAILED (Code: %d)", result.val);
        return false;
    }
    return true;
}

// ───────────────────────────────────────────────────────────────
// 単一物体の Pick & Place シーケンス
// 失敗した場合は false を返す
// ───────────────────────────────────────────────────────────────
bool pickAndPlace(
    rclcpp::Node::SharedPtr node,
    MoveGroupInterface &arm,
    GripperClient::SharedPtr gripper_client,
    const Pose3D &obj,
    const Pose3D &place,
    size_t index)
{
    RCLCPP_INFO(node->get_logger(),
        "--- [%zu] 物体 (%.2f, %.2f, %.2f) → 配置 (%.2f, %.2f, %.2f) ---",
        index, obj.x, obj.y, obj.z, place.x, place.y, place.z);

    // 1. グリッパーを開く
    controlGripper(node, gripper_client, GRIPPER_OPEN);

    // 2. アプローチ
    if (!moveToPositionConstrained(arm, obj.x, obj.y, obj.z + APPROACH_HEIGHT, 0.5))
        return false;

    // 3. 降下
    if (!moveToPositionConstrained(arm, obj.x, obj.y, obj.z, 0.2))
        return false;

    // 4. 把持
    RCLCPP_INFO(node->get_logger(), "把持中（段階的グリッパー閉鎖）...");
    closeGripperGradually(node, gripper_client, GRIPPER_CLOSE);

    // 5. 持ち上げ
    if (!moveToPositionConstrained(arm, obj.x, obj.y, obj.z + APPROACH_HEIGHT, 0.3))
        return false;

    // 6. 配置位置上方へ移動
    if (!moveToPositionConstrained(arm, place.x, place.y, place.z + APPROACH_HEIGHT, 0.5))
        return false;

    // 7. 配置位置へ降下
    if (!moveToPositionConstrained(arm, place.x, place.y, place.z, 0.2))
        return false;

    // 8. グリッパーを開いて開放
    controlGripper(node, gripper_client, GRIPPER_OPEN);
    RCLCPP_INFO(node->get_logger(), "[%zu] 配置完了。", index);

    // 9. 退避
    moveToPositionConstrained(arm, place.x, place.y, place.z + APPROACH_HEIGHT, 0.3);

    return true;
}

// ───────────────────────────────────────────────────────────────
// メインシーケンス
// ───────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({{"use_sim_time", true}});
    auto node = rclcpp::Node::make_shared("fixed_j5_pick_place", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto gripper_client = rclcpp_action::create_client<GripperCommand>(
        node, "/gripper_controller/gripper_cmd");

    MoveGroupInterface arm(node, "arm");
    arm.setPoseReferenceFrame("base_footprint");
    arm.setPlanningTime(30.0);
    arm.setNumPlanningAttempts(5);

    const size_t total = std::min(OBJ_POSITIONS.size(), PLACE_POSITIONS.size());
    RCLCPP_INFO(node->get_logger(), "=== 連続 Pick & Place 開始 (%zu 個) ===", total);

    size_t success_count = 0;
    for (size_t i = 0; i < total && rclcpp::ok(); ++i) {
        if (pickAndPlace(node, arm, gripper_client, OBJ_POSITIONS[i], PLACE_POSITIONS[i], i)) {
            ++success_count;
        } else {
            RCLCPP_WARN(node->get_logger(), "[%zu] Pick & Place 失敗。次の物体へスキップ。", i);
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "=== 全シーケンス終了: %zu / %zu 成功 ===", success_count, total);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
