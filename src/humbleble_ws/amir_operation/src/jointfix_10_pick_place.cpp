/*
 * 10個連続 Pick & Place
 * spawn_10box でスポーンされた物体を順番に把持・配置する。
 *
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * 【場所の設定方法】
 *   このファイル冒頭の「CONFIGURATION」セクションだけを編集する。
 *   OBJ_POSITIONS  : 把持する物体の座標リスト (spawn_10box の引数と合わせる)
 *   PLACE_POSITIONS: 配置先の座標リスト
 *   個数を変えたい場合は両リストの要素数を同じにするだけでよい。
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
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

// ╔═══════════════════════════════════════════════════════════════╗
// ║                      CONFIGURATION                            ║
// ╚═══════════════════════════════════════════════════════════════╝

struct Pose3D { double x, y, z; };

// ── 把持対象の座標 ──────────────────────────────────────────────
// spawn_10box デフォルト: 2列×5行グリッド
//   列0: x=0.50, 列1: x=0.65  (x_step=0.15)
//   y: -0.30 〜 0.30           (y_step=0.15)
//   z はグリッパーが掴む高さ = spawn z(0.05) + size_z/2(0.05) = 0.10
const std::vector<Pose3D> OBJ_POSITIONS = {
    // 列0 (x=0.50)
    {0.30, -0.30, 0.10},  // 0
    {0.30, -0.15, 0.10},  // 1
    {0.30,  0.00, 0.10},  // 2
    {0.30,  0.15, 0.10},  // 3
    {0.30,  0.30, 0.10},  // 4
    // 列1 (x=0.65)
    {0.45, -0.30, 0.10},  // 5
    {0.45, -0.15, 0.10},  // 6
    {0.45,  0.00, 0.10},  // 7
    {0.45,  0.15, 0.10},  // 8
    {0.45,  0.30, 0.10},  // 9
};

// ── 配置先の座標 ────────────────────────────────────────────────
// 元の4個版と同じ方針: Y を固定して X 方向に並べる (可動域内に収まる)
//   行0 (y=0.40): x = -0.30 〜 0.30  (5個)
//   行1 (y=0.55): x = -0.30 〜 0.30  (5個)
const std::vector<Pose3D> PLACE_POSITIONS = {
    {-0.30, 0.40, 0.15},  // 0
    {-0.15, 0.40, 0.15},  // 1
    { 0.00, 0.40, 0.15},  // 2
    { 0.15, 0.40, 0.15},  // 3
    { 0.30, 0.40, 0.15},  // 4
    {-0.30, 0.55, 0.15},  // 5
    {-0.15, 0.55, 0.15},  // 6
    { 0.00, 0.55, 0.15},  // 7
    { 0.15, 0.55, 0.15},  // 8
    { 0.30, 0.55, 0.15},  // 9
};

// ── アーム動作パラメータ ─────────────────────────────────────────
const double APPROACH_HEIGHT     = 0.15;   // アプローチ上方オフセット [m]
const double FIXED_JOINT_5_VALUE = 0.0;    // アプローチ/降下時の Joint_5 固定角度 [rad]

// ── 把持物体サイズ (spawn_10box の設定と合わせる) ────────────────
constexpr double OBJ_SIZE_X = 0.05;
constexpr double OBJ_SIZE_Y = 0.05;
constexpr double OBJ_SIZE_Z = 0.10;

// ── アタッチ時に接触を許可するリンク (グリッパー全体 + カメラ) ───
// attachObject の touch_links に渡すことで、把持中の自己衝突誤検知を防ぐ
const std::vector<std::string> TOUCH_LINKS = {
    "gripper_base_1",
    "finger_left_1",
    "finger_right_1",
    "inner_link_left_1",
    "inner_link_right_1",
    "outer_link_left_1",
    "outer_link_right_1",
    "tcp_link",
    "d435_link",
    "d435_bottom_screw_frame",
};

// ── グリッパーパラメータ ─────────────────────────────────────────
const double GRIPPER_OPEN            = -1.0;
const double GRIPPER_CLOSE           =  0.1;
const int    GRIPPER_CLOSE_STEPS     = 25;
const int    GRIPPER_CLOSE_STEP_MS   = 100;
const double GRIPPER_STALL_THRESHOLD = 0.01;
const int    GRIPPER_DETECT_START    = 5;
const double GRIPPER_PRELOAD         = 0.10;
const double GRIPPER_MAX_POS         = 0.261799;

// ╔═══════════════════════════════════════════════════════════════╗
// ║                    IMPLEMENTATION                             ║
// ╚═══════════════════════════════════════════════════════════════╝

static std::string objId(size_t i) { return "box_" + std::to_string(i); }

// 衝突オブジェクトをシーンに追加
void addCollisionObject(PlanningSceneInterface &scene,
                        const std::string &id, const Pose3D &pos)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = id;
    obj.header.frame_id = "base_footprint";

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {OBJ_SIZE_X, OBJ_SIZE_Y, OBJ_SIZE_Z};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x    = pos.x;
    pose.position.y    = pos.y;
    pose.position.z    = pos.z;  // OBJ_POSITIONS の z はボックス中心

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;

    scene.applyCollisionObject(obj);
}

// 衝突オブジェクトをシーンから除去
void removeCollisionObject(PlanningSceneInterface &scene, const std::string &id)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.id        = id;
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.applyCollisionObject(obj);
}

// 全物体をシーンに登録 (起動時に一括追加)
void initAllObjects(PlanningSceneInterface &scene)
{
    for (size_t i = 0; i < OBJ_POSITIONS.size(); ++i)
        addCollisionObject(scene, objId(i), OBJ_POSITIONS[i]);
    rclcpp::sleep_for(500ms);
}

// 把持後にグリッパーへアタッチ (touch_links でグリッパー/カメラとの誤衝突を抑制)
void attachToGripper(MoveGroupInterface &group, const std::string &id)
{
    group.attachObject(id, "gripper_base_1", TOUCH_LINKS);
    rclcpp::sleep_for(500ms);
}

// グリッパーからデタッチのみ (シーン登録は退避後に行う)
void detachOnly(MoveGroupInterface &group, const std::string &id)
{
    group.detachObject(id);
    rclcpp::sleep_for(200ms);
}


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
    double start_pos = g_pos.load();
    double range     = target_pos - start_pos;
    bool   contacted = false;
    double contact_pos = target_pos;
    double prev_actual = start_pos;

    RCLCPP_INFO(node->get_logger(),
        "Gripper 段階的閉じ (start=%.3f → target=%.3f)", start_pos, target_pos);

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
            RCLCPP_INFO(node->get_logger(),
                "Gripper 接触検知 step=%d actual=%.3f", step, actual);
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

    RCLCPP_INFO(node->get_logger(), "Gripper 保持位置=%.3f", hold_target);
    rclcpp::sleep_for(500ms);
}

bool moveConstrained(
    MoveGroupInterface &group,
    double x, double y, double z,
    double speed)
{
    group.setStartStateToCurrentState();

    moveit_msgs::msg::Constraints constraints;

    moveit_msgs::msg::JointConstraint jc4;
    jc4.joint_name      = "Joint_4";
    jc4.position        = 0.0;
    jc4.tolerance_above = 0.5;
    jc4.tolerance_below = 0.5;
    jc4.weight          = 1.0;
    constraints.joint_constraints.push_back(jc4);

    moveit_msgs::msg::JointConstraint jc5;
    jc5.joint_name      = "Joint_5";
    jc5.position        = FIXED_JOINT_5_VALUE;
    jc5.tolerance_above = 0.05;
    jc5.tolerance_below = 0.05;
    jc5.weight          = 1.0;
    constraints.joint_constraints.push_back(jc5);

    group.setPathConstraints(constraints);
    group.setPositionTarget(x, y, z);
    group.setGoalPositionTolerance(0.01);
    group.setMaxVelocityScalingFactor(speed);
    group.setMaxAccelerationScalingFactor(speed * 0.5);

    RCLCPP_INFO(rclcpp::get_logger("pick_place_10"),
        "-> (%.3f, %.3f, %.3f)", x, y, z);

    auto result = group.move();
    group.clearPathConstraints();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("pick_place_10"), "Move FAILED (Code: %d)", result.val);
        return false;
    }
    return true;
}

bool pickAndPlace(
    rclcpp::Node::SharedPtr node,
    MoveGroupInterface &arm,
    PlanningSceneInterface &scene,
    GripperClient::SharedPtr gc,
    const Pose3D &obj,
    const Pose3D &place,
    size_t index)
{
    const std::string id = objId(index);

    RCLCPP_INFO(node->get_logger(),
        "--- [%zu/10] 物体(%.2f,%.2f,%.2f) → 配置(%.2f,%.2f,%.2f) ---",
        index + 1, obj.x, obj.y, obj.z, place.x, place.y, place.z);

    controlGripper(node, gc, GRIPPER_OPEN);

    // アプローチ前に対象物体をシーンから除去 (降下時に自己障害物判定しないよう)
    removeCollisionObject(scene, id);
    rclcpp::sleep_for(200ms);

    if (!moveConstrained(arm, obj.x, obj.y, obj.z + APPROACH_HEIGHT, 0.5)) return false;
    if (!moveConstrained(arm, obj.x, obj.y, obj.z,                   0.2)) return false;

    RCLCPP_INFO(node->get_logger(), "把持中...");
    closeGripperGradually(node, gc, GRIPPER_CLOSE);

    // attachObject はワールドにオブジェクトが存在している必要があるため再登録
    addCollisionObject(scene, id, obj);
    rclcpp::sleep_for(200ms);

    // グリッパーへアタッチ (touch_links でグリッパー/カメラとの誤衝突を抑制)
    attachToGripper(arm, id);

    if (!moveConstrained(arm, obj.x,   obj.y,   obj.z   + APPROACH_HEIGHT, 0.3)) return false;
    if (!moveConstrained(arm, place.x, place.y, place.z + APPROACH_HEIGHT, 0.5)) return false;
    if (!moveConstrained(arm, place.x, place.y, place.z,                   0.2)) return false;

    controlGripper(node, gc, GRIPPER_OPEN);

    detachOnly(arm, id);
    // detachObject がシーンへ物体を戻す場合に備えて明示的に除去
    removeCollisionObject(scene, id);

    RCLCPP_INFO(node->get_logger(), "[%zu] 配置完了", index + 1);

    moveConstrained(arm, place.x, place.y, place.z + APPROACH_HEIGHT, 0.3);
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({{"use_sim_time", true}});
    auto node = rclcpp::Node::make_shared("jointfix_10_pick_place", opts);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto gc = rclcpp_action::create_client<GripperCommand>(
        node, "/gripper_controller/gripper_cmd");

    MoveGroupInterface arm(node, "arm");
    arm.setPoseReferenceFrame("base_footprint");
    arm.setPlanningTime(30.0);
    arm.setNumPlanningAttempts(5);

    PlanningSceneInterface scene;

    // 全物体をシーンに登録 (経路計画が他の物体を障害物として認識できるよう)
    initAllObjects(scene);

    const size_t total = std::min(OBJ_POSITIONS.size(), PLACE_POSITIONS.size());
    RCLCPP_INFO(node->get_logger(), "=== 連続 Pick & Place 開始 (%zu 個) ===", total);

    size_t success = 0;
    std::string last_placed_id;
    Pose3D      last_placed_pos{};

    for (size_t i = 0; i < total && rclcpp::ok(); ++i) {
        // 前回設置成功した物体をここで障害物登録 (アームが離れた後のタイミング)
        if (!last_placed_id.empty()) {
            addCollisionObject(scene, last_placed_id, last_placed_pos);
            rclcpp::sleep_for(300ms);
        }

        if (pickAndPlace(node, arm, scene, gc, OBJ_POSITIONS[i], PLACE_POSITIONS[i], i)) {
            ++success;
            last_placed_id  = objId(i);
            last_placed_pos = PLACE_POSITIONS[i];
        } else {
            last_placed_id.clear();
            RCLCPP_WARN(node->get_logger(), "[%zu] 失敗。次の物体へスキップ。", i + 1);
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "=== 全完了: %zu / %zu 成功 ===", success, total);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
