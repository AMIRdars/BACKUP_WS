// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__CHECK_VEL_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__CHECK_VEL_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include <math.h>
#include "geometry_msgs/msg/twist.hpp"
// #include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/buffer.h"
// #include "geometry/msg/posestamped.hpp"
// #include "observation_msgs/msg/object.hpp"


namespace ros2_behavior_tree
{

class CheckVelNode : public BT::ConditionNode
{
public:
  CheckVelNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    rover_check("rover_odo")

  {
    node_=BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;

  }

  static BT::PortsList providedPorts()
  {
    return {
      
      BT::InputPort<std::string>("rover_check",std::string("rover_odo") ,""),
      // BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination")
    };
  }

  BT::NodeStatus tick() override
  {
   
    while (!rclcpp::wait_for_message(rover_topic_,node_,"rover_odo",server_timeout_))
    {
      RCLCPP_DEBUG(node_->get_logger(),"timeout rover_odo topic");
      return BT::NodeStatus::FAILURE;
    }

    // number = help_topic.data;
    // geometry_msgs::msg::PoseStamped goal;
    // getInput("goal", goal);
    // double dx = goal.pose.position.x - current_pose.pose.position.x;
    // double dy = goal.pose.position.y - current_pose.pose.position.y;
    rclcpp::Rate loop_rate(1);

    double vel_x = rover_topic_.linear.x;
    double vel_y = rover_topic_.linear.y;

    if(vel_x != 0 && vel_y != 0)
    {
      return BT::NodeStatus::FAILURE;
    }
    else{
      return BT::NodeStatus::SUCCESS;
    }

    // if(number == 1.0)
    // {
    //   return BT::NodeStatus::SUCCESS;
    // }
    // else 
    // {
    //   return BT::NodeStatus::FAILURE;

    // }

    // return BT::NodeStatus::SUCCESS;

  }

private:
  rclcpp::Node::SharedPtr node_;

  // geometry_msgs::msg::PoseStamped drone_plan_;
  // std_msgs::msg::Float64 drone_task_;
  // std::string task_topic_;
  // std::string drone_goal_path_topic_;
  // std_msgs::msg::Float64 help_topic;
  // std::string help_pub_drone;
  // double number;
  // double path_x;

  // nav_msgs::msg::Odometry odom_topic_;
  // std::string odom_check;

  geometry_msgs::msg::Twist rover_topic_;
  std::string rover_check;

  BT::NodeStatus b_status_;

  std::chrono::milliseconds server_timeout_;



};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__CHECK_ALTITUDE_HIGH_NODE_HPP_
