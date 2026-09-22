#ifndef COOP_TRANSPORT_BT__COOPERATIVE_MOTION_BT_NODE_HPP_
#define COOP_TRANSPORT_BT__COOPERATIVE_MOTION_BT_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "coop_transport_interfaces/action/cooperative_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace coop_transport_bt
{

class CooperativeMotionBTNode : public BT::StatefulActionNode
{
public:
  using Action = coop_transport_interfaces::action::CooperativeMotion;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  CooperativeMotionBTNode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  template<typename T>
  bool readInput(const std::string & key, T & value);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::atomic_bool result_received_{false};
  std::atomic_bool result_succeeded_{false};
  std::atomic_bool halted_{false};
  std::string result_message_;
};

}  // namespace coop_transport_bt

#endif  // COOP_TRANSPORT_BT__COOPERATIVE_MOTION_BT_NODE_HPP_
