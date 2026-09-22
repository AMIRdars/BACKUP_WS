#ifndef ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_AND_RECEIVE_GOAL_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_AND_RECEIVE_GOAL_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
// #include "behaviortree_cpp_v3/condition_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr destination_sub_;
// geometry_msgs::msg::PoseStamped destination;

namespace ros2_behavior_tree
{

class CalculateReceiveGoal : public BT::SyncActionNode
{
public:
  // QoS 設定: 
  //RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  //RMW_QOS_POLICY_RELIABILITY_RELIABLE
  // rclcpp::QoS qos_profile(10); // キューサイズ10
  // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  // base_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/amir/destination",qos_profile,std::bind(&PickServer::destinationCallback,this,std::placeholders::_1));

  // void destinationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr destinationMsg){
  //   destination = *destinationMsg;
  // }

  CalculateReceiveGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    destination("/amir/destination")
  {
    node_=BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<BT::Position2D>("input_position", "position"),
      BT::InputPort<std::string>("destination", std::string("/amir/destination"),"" ),
      // BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_msg", "geometry_msgs::msg::PoseStamped")
      BT::OutputPort<std_msgs::msg::Float64>("output_x_pose", "std_msgs::msg::Float64")
      BT::OutputPort<std_msgs::msg::Float64>("output_y_pose", "std_msgs::msg::Float64")
    };
  }

  BT::NodeStatus tick() override
  {
    // BT::Position2D input_position;
    geometry_msgs::msg::PoseStamped input_position;

    // input_position.x = destination_position.pose.position.x;
    // input_position.y = destination_position.pose.position.y;

    // if (!getInput<geometry_msgs::msg::PoseStamped>("destination", input_position)) {
    //   throw BT::RuntimeError("Missing parameter [input_position] in CalculateReceiveGoal node");
    // }

    // while (!rclcpp::wait_for_message(destination_topic,node_,"/amir/destination",server_timeout_))
    // {
    //   RCLCPP_DEBUG(node_->get_logger(),"timeout /amir/destination topic");
    //   return BT::NodeStatus::FAILURE;
    // }

    while (!rclcpp::wait_for_message(destination_topic,node_,"/amir/destination"))
    {
      RCLCPP_DEBUG(node_->get_logger(),"timeout /amir/destination topic");
      return BT::NodeStatus::FAILURE;
    }
    
    std_msgs::msg::Float64 pose_x;
    std_msgs::msg::Float64 pose_y;
    pose_x = destination_topic.pose.position.x;
    pose_y = destination_topic.pose.position.y;
    setOutput<std_msgs::msg::Float64>("output_x_pose", pose_x);
    setOutput<std_msgs::msg::Float64>("output_y_pose", pose_y);
    
    // geometry_msgs::msg::PoseStamped  output_msg;
    // output_msg.pose.position.x = destination_topic.pose.position.x;
    // output_msg.pose.position.y = destination_topic.pose.position.y;
    // output_msg.header.frame_id = "map";
    
    // setOutput<geometry_msgs::msg::PoseStamped>("output_msg", output_msg);
    return BT::NodeStatus::SUCCESS;
  }
private:
  rclcpp::Node::SharedPtr node_;
  std::string destination;
  geometry_msgs::msg::PoseStamped destination_topic;

  BT::NodeStatus b_status_;

  std::chrono::milliseconds server_timeout_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_GOAL_HPP_
