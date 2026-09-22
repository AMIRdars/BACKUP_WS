#ifndef ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_BTL_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_BTL_HPP_

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

class SearchBtl : public BT::SyncActionNode
{
public:
  SearchBtl(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    btl_topic_("/btl_poses")
  {
    node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
    // test_pub_ = node_->create_publisher<std_msgs::msg::Float64>("btl_posi",1);
  }

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<BT::Position2D>("input_position", "position"),
      // BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_msg", "geometry_msgs::msg::PoseStamped")
      // BT::InputPort<BT::geometry_msgs::msg::PoseArray>("btl_topic_",std::string("/aruco_poses") "position"),
      BT::InputPort<std::string>("btl_topic_",std::string("/btl_poses")),
      // BT::OutputPort<double>("btl_x", ""),
      // BT::OutputPort<double>("btl_y", "")
      BT::OutputPort<double>("posi_x", ""),
      BT::OutputPort<double>("posi_y", "")

    };
  }

  BT::NodeStatus tick() override
  {
    // BT::Position2D input_position;
    // if (!getInput<BT::Position2D>("input_position", input_position)) {
    //   throw BT::RuntimeError("Missing parameter [input_position] in CalculateGoal node");
    // }

    while (!rclcpp::wait_for_message(btl_posi_,node_,"/btl_poses",std::chrono::milliseconds(6000)))
    {
      RCLCPP_DEBUG(node_ ->get_logger(),"timeout /btl_poses");
      return BT::NodeStatus::FAILURE;
    }

    btl_x = btl_posi_.position.z + 0.15;
    btl_y = -btl_posi_.position.x + 0.05;



    if(btl_x > 0.6)
    {
      posi_x = btl_x - 0.50;
      posi_y = 0.0;
      setOutput<double>("posi_x", posi_x);
      setOutput<double>("posi_y", posi_y);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      posi_x = 0.0;
      posi_y = 0.0;
      setOutput<double>("posi_x", posi_x);
      setOutput<double>("posi_y", posi_y);
      return BT::NodeStatus::FAILURE;
    }



    // setOutput<double>("posi_x", posi_x);
    // setOutput<double>("posi_y", posi_y);


    // return BT::NodeStatus::SUCCESS;
  }

private:
rclcpp::Node::SharedPtr node_;
BT::NodeStatus b_status_;
std::chrono::milliseconds server_timeout_;

geometry_msgs::msg::Pose btl_posi_;
std::string btl_topic_;

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub_;

double btl_x;
double btl_y;

double posi_x;
double posi_y;

};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_GOAL_HPP_
