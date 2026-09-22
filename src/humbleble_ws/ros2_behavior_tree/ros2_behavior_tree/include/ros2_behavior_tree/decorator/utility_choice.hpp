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

#ifndef ROS2_BEHAVIOR_TREE__DECORATOR__UTILITY_CHOICE_HPP_
#define ROS2_BEHAVIOR_TREE__DECORATOR__UTILITY_CHOICE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ros2_behavior_tree
{

class UtilityChoice : public BT::DecoratorNode
{
public:
  UtilityChoice(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config),
    bebop_pose_topic_("/bebop/pose"),
    meca_position_topic_("/mecanum2/odom1")
  {
    //executed_=false;

    node_=BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("bebop_pose_topic_",std::string("/bebop/pose") ,""),
      BT::InputPort<std::string>("meca_position_topic_",std::string("/mecanum2/odom1") ,"odom1"),

    };
  }

  BT::NodeStatus tick() override
  {
    while (!rclcpp::wait_for_message(bebop_pose_,node_,"/bebop/pose",server_timeout_))
    {
      RCLCPP_DEBUG(node_->get_logger(),"timeout /bebop/pose topic");
    }

    while (!rclcpp::wait_for_message(meca_position_,node_,"/mecanum2/odom1",server_timeout_))
    {
      RCLCPP_DEBUG(node_->get_logger(),"timeout /mecanum2/odom1 topic");
    }

    
    //rclcpp::WallRate loop_rate(1);

    bebop_x = bebop_pose_.pose.position.x;
    bebop_y = bebop_pose_.pose.position.y;
    // bebop_x = 100;
    // bebop_y = 1;
    meca_x = meca_position_.pose.pose.position.x -1;
    meca_y = meca_position_.pose.pose.position.y;
    kyori_x = bebop_x - meca_x;
    kyori_y = bebop_y - meca_y;
    diff = sqrt(kyori_x * kyori_x + kyori_y * kyori_y);


    if(diff < 3)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      auto child_state = child_node_->executeTick();
      return BT::NodeStatus::FAILURE;
    }
    
    //setStatus(BT::NodeStatus::RUNNING);

    //auto child_state = child_node_->executeTick();

    // switch (child_state) {
    //   case BT::NodeStatus::SUCCESS:
    //     executed_ = true;
    //     return BT::NodeStatus::SUCCESS;

    //   case BT::NodeStatus::RUNNING:
    //     return BT::NodeStatus::RUNNING;

    //   case BT::NodeStatus::FAILURE:
    //     executed_ = true;
    //     return BT::NodeStatus::FAILURE;

    //   default:
    //     throw BT::LogicError("Invalid status return from BT node");
    // }

    // None of the poses worked, so fail
    // child()->halt();
    // return BT::NodeStatus::FAILURE;
  }

private:
  bool executed_;

  rclcpp::Node::SharedPtr node_;


  BT::NodeStatus b_status_;

  geometry_msgs::msg::PoseStamped bebop_pose_;
  std::string bebop_pose_topic_;

  nav_msgs::msg::Odometry meca_position_;
  std::string meca_position_topic_;

  std::chrono::milliseconds server_timeout_;


  double bebop_x;
  double bebop_y;
  double meca_x;
  double meca_y;
  double kyori_x;
  double kyori_y;
  double diff;


};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__DECORATOR__FOR_UTILITY_CHOICE_NODE_HPP_
