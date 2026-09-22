#include "coop_transport_bt/transport_safety_nodes.hpp"

namespace coop_transport_bt
{

CheckTransportSafe::CheckTransportSafe(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("CheckTransportSafe requires a rclcpp node in blackboard key 'node'");
  }
  stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/cooperative_transport/emergency_stop", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {emergency_stop_ = msg->data;});
  grasp1_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/cooperative_transport/amir1/grasp_state", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {grasp1_ = msg->data; have_grasp1_ = true;});
  grasp2_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/cooperative_transport/amir2/grasp_state", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {grasp2_ = msg->data; have_grasp2_ = true;});
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cooperative_transport/payload_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      have_pose_ = true;
      pose_stamp_ = rclcpp::Time(msg->header.stamp);
    });
}

BT::PortsList CheckTransportSafe::providedPorts()
{
  return {
    BT::InputPort<double>("pose_timeout", 0.5, "seconds"),
    BT::InputPort<bool>("require_grasp", true, "require both grasp states")
  };
}

BT::NodeStatus CheckTransportSafe::tick()
{
  double timeout = 0.5;
  bool require_grasp = true;
  getInput("pose_timeout", timeout);
  getInput("require_grasp", require_grasp);
  const bool pose_fresh = have_pose_ &&
    (node_->get_clock()->now() - pose_stamp_).seconds() <= timeout;
  if (emergency_stop_ || !pose_fresh ||
    (require_grasp && (!have_grasp1_ || !have_grasp2_ || !grasp1_ || !grasp2_)))
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

EmergencyStop::EmergencyStop(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  rclcpp::Node::SharedPtr node;
  if (!config.blackboard || !config.blackboard->get("node", node)) {
    throw BT::RuntimeError("EmergencyStop requires a rclcpp node in blackboard key 'node'");
  }
  enable_pub_ = node->create_publisher<std_msgs::msg::Bool>("/cooperative_transport/enable", 10);
  robot1_stop_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/amir1/rover_twist", 10);
  robot2_stop_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/amir2/rover_twist", 10);
}

BT::PortsList EmergencyStop::providedPorts()
{
  return {};
}

BT::NodeStatus EmergencyStop::tick()
{
  std_msgs::msg::Bool disable;
  disable.data = false;
  geometry_msgs::msg::Twist stop;
  enable_pub_->publish(disable);
  robot1_stop_pub_->publish(stop);
  robot2_stop_pub_->publish(stop);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace coop_transport_bt
