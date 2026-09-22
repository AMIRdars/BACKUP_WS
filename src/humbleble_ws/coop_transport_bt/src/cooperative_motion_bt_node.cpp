#include "coop_transport_bt/cooperative_motion_bt_node.hpp"

#include <chrono>
#include <utility>

namespace coop_transport_bt
{

CooperativeMotionBTNode::CooperativeMotionBTNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("CooperativeMotion requires a rclcpp node in blackboard key 'node'");
  }
  client_ = rclcpp_action::create_client<Action>(node_, "/cooperative_motion");
}

BT::PortsList CooperativeMotionBTNode::providedPorts()
{
  return {
    BT::InputPort<int>("motion_type"),
    BT::InputPort<int>("angle_mode", 0, "absolute (0) or relative (1)"),
    BT::InputPort<double>("target_x", 0.0, "world x target"),
    BT::InputPort<double>("target_y", 0.0, "world y target"),
    BT::InputPort<double>("target_yaw", 0.0, "world yaw target"),
    BT::InputPort<double>("pivot_x", 0.0, "world x pivot"),
    BT::InputPort<double>("pivot_y", 0.0, "world y pivot"),
    BT::InputPort<double>("rotation_angle", 0.0, "relative yaw"),
    BT::InputPort<double>("linear_velocity", 0.03, "m/s"),
    BT::InputPort<double>("angular_velocity", 0.05, "rad/s"),
    BT::InputPort<double>("position_tolerance", 0.02, "m"),
    BT::InputPort<double>("angle_tolerance", 0.035, "rad"),
    BT::InputPort<double>("joint_1_target", 0.0, "Joint_1 target in rad"),
    BT::InputPort<double>("joint_2_target", 0.0, "Joint_2 target in rad"),
    BT::InputPort<double>("joint_3_target", 0.0, "Joint_3 target in rad"),
    BT::InputPort<double>("joint_4_target", 0.0, "Joint_4 target in rad"),
    BT::InputPort<double>("joint_5_target", 0.0, "Joint_5 target in rad"),
    BT::InputPort<double>("joint_tolerance", 0.03, "rad"),
    BT::InputPort<double>("arm_motion_duration", 8.0, "s")
  };
}

template<typename T>
bool CooperativeMotionBTNode::readInput(const std::string & key, T & value)
{
  auto input = getInput<T>(key);
  if (!input) {
    RCLCPP_ERROR(node_->get_logger(), "%s: missing or invalid port '%s': %s",
      name().c_str(), key.c_str(), input.error().c_str());
    return false;
  }
  value = input.value();
  return true;
}

BT::NodeStatus CooperativeMotionBTNode::onStart()
{
  if (!client_->wait_for_action_server(std::chrono::milliseconds(0))) {
    RCLCPP_ERROR(node_->get_logger(), "CooperativeMotion Action server is unavailable");
    return BT::NodeStatus::FAILURE;
  }

  Action::Goal goal;
  int motion_type = 0;
  int angle_mode = 0;
  if (!readInput("motion_type", motion_type) || !readInput("angle_mode", angle_mode) ||
    !readInput("target_x", goal.target_x) || !readInput("target_y", goal.target_y) ||
    !readInput("target_yaw", goal.target_yaw) || !readInput("pivot_x", goal.pivot_x) ||
    !readInput("pivot_y", goal.pivot_y) || !readInput("rotation_angle", goal.rotation_angle) ||
    !readInput("linear_velocity", goal.linear_velocity) ||
    !readInput("angular_velocity", goal.angular_velocity) ||
    !readInput("position_tolerance", goal.position_tolerance) ||
    !readInput("angle_tolerance", goal.angle_tolerance) ||
    !readInput("joint_1_target", goal.joint_1_target) ||
    !readInput("joint_2_target", goal.joint_2_target) ||
    !readInput("joint_3_target", goal.joint_3_target) ||
    !readInput("joint_4_target", goal.joint_4_target) ||
    !readInput("joint_5_target", goal.joint_5_target) ||
    !readInput("joint_tolerance", goal.joint_tolerance) ||
    !readInput("arm_motion_duration", goal.arm_motion_duration))
  {
    return BT::NodeStatus::FAILURE;
  }
  goal.motion_type = static_cast<uint8_t>(motion_type);
  goal.angle_mode = static_cast<uint8_t>(angle_mode);
  result_received_ = false;
  result_succeeded_ = false;
  result_message_.clear();
  goal_handle_.reset();
  halted_ = false;

  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        result_message_ = "Action goal was rejected";
        result_received_ = true;
        return;
      }
      goal_handle_ = handle;
      if (halted_) {
        client_->async_cancel_goal(handle);
      }
    };
  options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      result_succeeded_ = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->success;
      result_message_ = result.result ? result.result->message : "Action returned no result";
      result_received_ = true;
    };
  client_->async_send_goal(goal, options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CooperativeMotionBTNode::onRunning()
{
  if (!result_received_) {
    return BT::NodeStatus::RUNNING;
  }
  if (result_succeeded_) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(node_->get_logger(), "CooperativeMotion failed: %s", result_message_.c_str());
  return BT::NodeStatus::FAILURE;
}

void CooperativeMotionBTNode::onHalted()
{
  halted_ = true;
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace coop_transport_bt
