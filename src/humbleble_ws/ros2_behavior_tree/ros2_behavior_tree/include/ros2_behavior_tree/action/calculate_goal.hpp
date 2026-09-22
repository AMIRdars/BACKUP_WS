#ifndef ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_GOAL_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_GOAL_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

namespace ros2_behavior_tree
{

class CalculateGoal : public BT::SyncActionNode
{
public:
  CalculateGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BT::Position2D>("input_position", "position"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_msg", "geometry_msgs::msg::PoseStamped")
    };
  }

  BT::NodeStatus tick() override
  {
    BT::Position2D input_position;
    if (!getInput<BT::Position2D>("input_position", input_position)) {
      throw BT::RuntimeError("Missing parameter [input_position] in CalculateGoal node");
    }
    geometry_msgs::msg::PoseStamped  output_msg;
    
    output_msg.pose.position.x = input_position.x;
    output_msg.pose.position.y = input_position.y;
    output_msg.header.frame_id = "map";
    setOutput<geometry_msgs::msg::PoseStamped>("output_msg", output_msg);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_GOAL_HPP_
