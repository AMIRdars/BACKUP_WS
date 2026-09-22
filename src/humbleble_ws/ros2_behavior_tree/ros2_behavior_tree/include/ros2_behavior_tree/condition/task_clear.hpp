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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__TASK_CLEAR_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__TASK_CLEAR_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/path.hpp"
#include <math.h>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
// #include "geometry/msg/posestamped.hpp"
// #include "observation_msgs/msg/object.hpp"


namespace ros2_behavior_tree
{

class TaskClearNode : public BT::ConditionNode
{
public:
  TaskClearNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
    // task_clear_topic_("path_cost")
  {
    node_=BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
    clear_pub_ = node_->create_publisher<std_msgs::msg::Float64>("task_clear", 1);
  }

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<std::string>("task_clear_topic_",std::string("path_cost") ,""),
    };
  }

  BT::NodeStatus tick() override
  {
    // while (!rclcpp::wait_for_message(check_entropy_,node_,"path_cost",server_timeout_))
    // {
    //   RCLCPP_DEBUG(node_->get_logger(),"timeout path_cost topic");
    // }

    rclcpp::Rate loop_rate(1);
    std_msgs::msg::Float64 task_clear;
    task_clear.data = 1.0;
    clear_pub_->publish(task_clear);
    return BT::NodeStatus::SUCCESS;

    // if(check_entropy_.data <= 1.0)
    // {
      
    // }
    // else
    // {
    //   return BT::NodeStatus::FAILURE;
    // }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std_msgs::msg::Float64 check_entropy_;
  std::string task_clear_topic_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr clear_pub_;

  BT::NodeStatus b_status_;

  std::chrono::milliseconds server_timeout_;


};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__TASK_CLEAR_NODE_HPP_
