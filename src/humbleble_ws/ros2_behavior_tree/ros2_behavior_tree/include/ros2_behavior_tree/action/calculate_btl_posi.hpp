#ifndef ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_BTL_POSI_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_BTL_POSI_HPP_

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

class CalculateBtl : public BT::SyncActionNode
{
public:
  CalculateBtl(const std::string & name, const BT::NodeConfiguration & config)
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
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_obj", "")
    };
  }

  BT::NodeStatus tick() override
  {
    // BT::Position2D input_position;
    // if (!getInput<BT::Position2D>("input_position", input_position)) {
    //   throw BT::RuntimeError("Missing parameter [input_position] in CalculateGoal node");
    // }

    while (!rclcpp::wait_for_message(btl_posi_,node_,"/btl_poses",server_timeout_))
    {
      RCLCPP_DEBUG(node_ ->get_logger(),"timeout /btl_poses");
    }

    // btl_x = btl_posi_.position.z + 0.13;
    // btl_y = -btl_posi_.position.x + 0.03;


    // setOutput<double>("btl_x", btl_x);
    // setOutput<double>("btl_y", btl_y);
    geometry_msgs::msg::PoseStamped pose_obj;
    pose_obj.header.frame_id = "map";
    // pose_obj.pose = btl_posi_;sonomama
    pose_obj.pose.position.x = 0.57;
    pose_obj.pose.position.y = -0.08;
    pose_obj.pose.position.z = 0.42;
    
    pose_obj.pose.orientation = btl_posi_.orientation;
    
    setOutput<geometry_msgs::msg::PoseStamped>("pose_obj",pose_obj);

    // for (const auto& pose : btl_posi_.poses) {
    //     double btl_x = pose.position.z;
    //     // double y = pose.position.y;
    //     // double z = pose.position.z;
    //     // 各ポーズに対する処理
    // }    
    // std_msgs::msg::Float64 msg;
    // msg.data = btl_x;

    // test_pub_ ->publish(msg);
    // geometry_msgs::msg::PoseStamped  output_msg;
    
    // output_msg.pose.position.x = input_position.x;
    // output_msg.pose.position.y = input_position.y;
    // output_msg.header.frame_id = "map";
    // setOutput<geometry_msgs::msg::PoseStamped>("output_msg", output_msg);
    return BT::NodeStatus::SUCCESS;
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

};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_BTL_POSI_HPP_
