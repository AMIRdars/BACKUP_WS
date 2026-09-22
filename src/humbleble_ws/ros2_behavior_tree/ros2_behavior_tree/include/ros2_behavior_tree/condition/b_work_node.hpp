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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__B_WORK_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__B_WORK_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_behavior_tree
{

// using BWorkNode = behavior_tree_msgs::action::BWorkNode;

class BWorkNode : public BT::ConditionNode
{
public:
  BWorkNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return ({
        // BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        // 
        // BT::InputPort<std::string>("label", ""),
        // BT::InputPort<moveit_msgs::msg::CollisionObject>("object", ""),
        BT::InputPort<std::string>("topic", ""), //  エラー対策
        BT::InputPort<std::string>("type", ""),
        // BT::InputPort<std::string>("location_id", ""),
        // BT::OutputPort<geometry_msgs::msg::PoseStamped>("obj_pose", ""),
       
      });
  }

  BT::NodeStatus tick() override
  {
    std::string topic;
    if (!getInput<std::string>("topic", topic)) {
      throw BT::RuntimeError("Missing parameter [topic] in CanTransform node");
    }


    std::string type;
    if (!getInput<std::string>("type", type)) {
      throw BT::RuntimeError("Missing parameter [topic] in CanTransform node");
    }

    rclpy.publisher('topic','type',queue_size=1)

    
    // rclcpp::Time transform_time = node->now();
    // std::string tf_error;
    // double timeout = 0.0;

    if(getInput<std::string>("topic", topic) && getInput<std::string>("type", type))
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  
  

  // bool read_new_goal(BWorkNode::Goal & goal) override
  // {
  //   nav_msgs::msg::Path path;
  //   if (!getInput<nav_msgs::msg::Path>("path", path)) {
  //     throw BT::RuntimeError("Missing parameter [path] in BWorkNode node");
  //   }

  //   // If it's not the same as the goal we're currently working on, update the goal
  //   // and return true
  //   return (path != goal_.path) ? goal_.path = path, true : false;
  // }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__B_WORK_NODE_HPP_
