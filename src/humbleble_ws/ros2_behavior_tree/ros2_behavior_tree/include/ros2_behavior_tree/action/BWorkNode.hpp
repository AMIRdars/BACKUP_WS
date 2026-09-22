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

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/behavior_tree.h>
#include "behaviortree_cpp_v3/bt_factory.h"

namespace ros2_behavior_tree
{

// using BWorkNode = behavior_tree_msgs::action::BWorkNode;

class BWorkNode : public ros2_behavior_tree::ROS2ActionClientNode<BWorkNode>
{
public:
  explicit BWorkNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<BWorkNode>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        // BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        // 
        // BT::InputPort<std::string>("label", ""),
        // BT::InputPort<moveit_msgs::msg::CollisionObject>("object", ""),
        BT::InputPort<std::string>("topic", ""), //  エラー対策
        // BT::InputPort<std::string>("location_id", ""),
        // BT::OutputPort<geometry_msgs::msg::PoseStamped>("obj_pose", ""),
       
      });
  }

  void read_input_ports(BWorkNode::Goal & goal) override
  {
    // if (!getInput<nav_msgs::msg::Path>("path", goal.path)) {
    //   throw BT::RuntimeError("Missing parameter [path] in BWorkNodeNode node");
    // }

    // if(!getInput<moveit_msgs::msg::CollisionObject>("object", goal))
    // {
    //   RCLCPP_WARN(ros2_node_->get_logger(),"Missing parameter [object] in BWorkNodeNode node");
    // }


    if(!getInput<std::string>("topic"))
    {
      throw BT::RuntimeError("Missing parameter [topic] in BWork node");
    }

   
  }

  void write_output_ports(
    rclcpp_action::ClientGoalHandle<BWorkNode>::WrappedResult & result) override
  {

    // if (result.result->poses.size() > 0);
    // {
    //   setOutput<geometry_msgs::msg::PoseStamped>("obj_pose", result.result->poses[0]);
    // }
 
  }
 
  // アクション結果から最終的なBTの状態を返す
  BT::NodeStatus set_bt_result(
    typename rclcpp_action::ClientGoalHandle<BWorkNode>::WrappedResult & result) override
  {
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        // write_output_ports(result_);
        if (result_.getInput<std::string>("topic"))
        {
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          return BT::NodeStatus::FAILURE;
        }
        

      case rclcpp_action::ResultCode::ABORTED:
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        return BT::NodeStatus::SUCCESS;

      default:
        throw std::logic_error("ROS2ActionClientNode::tick: invalid status value");
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
