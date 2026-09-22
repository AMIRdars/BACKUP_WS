#ifndef COOP_TRANSPORT_BT__TRANSPORT_SAFETY_NODES_HPP_
#define COOP_TRANSPORT_BT__TRANSPORT_SAFETY_NODES_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace coop_transport_bt
{

class CheckTransportSafe : public BT::ConditionNode
{
public:
  CheckTransportSafe(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool emergency_stop_{false};
  bool grasp1_{false};
  bool grasp2_{false};
  bool have_grasp1_{false};
  bool have_grasp2_{false};
  bool have_pose_{false};
  rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp1_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

class EmergencyStop : public BT::SyncActionNode
{
public:
  EmergencyStop(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot1_stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot2_stop_pub_;
};

}  // namespace coop_transport_bt

#endif  // COOP_TRANSPORT_BT__TRANSPORT_SAFETY_NODES_HPP_
