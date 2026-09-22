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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__PICK_AMIR_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__PICK_AMIR_HPP_

#include <string>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "behavior_tree_msgs/action/pick.hpp"

namespace ros2_behavior_tree
{

using PickAmir = behavior_tree_msgs::action::Pick;

class PickAmirNode : public ros2_behavior_tree::ROS2ActionClientNode<PickAmir>
{
public:
  explicit PickAmirNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<PickAmir>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_obj", "")
    
      });
  }
  void read_input_ports(PickAmir::Goal & goal) override
  {
    if(!getInput<geometry_msgs::msg::PoseStamped>("pose_obj", goal.pose_obj))
    {
      throw BT::RuntimeError("Missing parameter pose_obj");
    }


  }

 BT::NodeStatus set_bt_result(
    typename rclcpp_action::ClientGoalHandle<PickAmir>::WrappedResult & result) override
  {
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result_.result->error_string == "success")
        {
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
        

      case rclcpp_action::ResultCode::ABORTED:
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        return BT::NodeStatus::SUCCESS;

      default:
        throw std::logic_error("ROS2ActionClientNode::tick: invalid status value");
    }
  }



  // bool read_new_goal(Pick::Goal & goal) override
  // {
  //   nav_msgs::msg::Path path;
  //   if (!getInput<nav_msgs::msg::Path>("path", path)) {
  //     throw BT::RuntimeError("Missing parameter [path] in PickNode node");
  //   }

  //   // If it's not the same as the goal we're currently working on, update the goal
  //   // and return true
  //   return (path != goal_.path) ? goal_.path = path, true : false;
  // }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__PICK_NODE_HPP_
