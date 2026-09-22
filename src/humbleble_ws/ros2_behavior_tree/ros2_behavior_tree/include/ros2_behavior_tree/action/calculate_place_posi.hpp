#ifndef ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_PLACE_POSI_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_PLACE_POSI_HPP_

#include <string>
#include <memory>
#include <math.h>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/float64.hpp"

namespace ros2_behavior_tree
{

class CalculatePlacePosi : public BT::SyncActionNode
{
public:
  CalculatePlacePosi(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
    // test_pub_ = node_->create_publisher<std_msgs::msg::Float64>("btl_posi",1);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", "Target x position"),
      BT::InputPort<double>("y", "Target y position"),
      BT::InputPort<double>("z", "Target z position"),
      // BT::InputPort<std::string>("obj_topic_",std::string("/amir1/object_position")),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_obj", "")
    };
  }

  BT::NodeStatus tick() override
  {
    double x, y, z;
    // 入力ポートから値を取得
    if (!getInput<double>("x", x)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [x]");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput<double>("y", y)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [y]");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput<double>("z", z)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [z]");
      return BT::NodeStatus::FAILURE;
    }
    

    geometry_msgs::msg::PoseStamped pose_obj;
    pose_obj.header.frame_id = "map";
    // pose_obj.pose = obj_posi_.pose;
    pose_obj.pose.position.x = x;
    pose_obj.pose.position.y = y;
    pose_obj.pose.position.z = z;
    
    setOutput<geometry_msgs::msg::PoseStamped>("pose_obj",pose_obj);

    return BT::NodeStatus::SUCCESS;
  }

private:
rclcpp::Node::SharedPtr node_;
BT::NodeStatus b_status_;
std::chrono::milliseconds server_timeout_;

// geometry_msgs::msg::PoseStamped obj_posi_;
// std::string obj_topic_;

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub_;


};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_PLACE_POSI_HPP_
