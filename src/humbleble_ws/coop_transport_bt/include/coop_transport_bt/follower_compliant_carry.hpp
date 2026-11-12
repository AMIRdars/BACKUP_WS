#ifndef COOP_TRANSPORT_BT__FOLLOWER_COMPLIANT_CARRY_HPP_
#define COOP_TRANSPORT_BT__FOLLOWER_COMPLIANT_CARRY_HPP_
#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace coop_transport_bt
{
// This action only enables/disables and checks status. No motion math or commands.
class FollowerCompliantCarry : public BT::StatefulActionNode
{
public:
  FollowerCompliantCarry(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  void disable();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture stop_response_;
  rclcpp::Time started_{0,0,RCL_ROS_TIME}, status_at_{0,0,RCL_ROS_TIME};
  rclcpp::Time stop_at_{0,0,RCL_ROS_TIME};
  rclcpp::Time enabled_at_{0,0,RCL_ROS_TIME};
  std::string status_;
  bool accepted_{false};
  bool stopping_{false};
  bool request_sent_{false};
  bool clock_ready_{false};
};
}
#endif
