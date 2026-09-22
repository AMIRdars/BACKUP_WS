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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__PLACE_AMIR_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__PLACE_AMIR_HPP_

#include <string>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "behavior_tree_msgs/action/place.hpp"

namespace ros2_behavior_tree
{

using PlaceAmir = behavior_tree_msgs::action::Place;

class PlaceAmirNode : public ros2_behavior_tree::ROS2ActionClientNode<PlaceAmir>
{
public:
  explicit PlaceAmirNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<PlaceAmir>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
      
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_obj", "")
    
      });
  }
  void read_input_ports(PlaceAmir::Goal & goal) override
  {
    // msg = behavior_tree_msgs::msg::Movemeca();
    // if(!getInput<double>("x", goal.x))
    // {
    //   throw BT::RuntimeError("Missing parameter x");
    // }
    // if(!getInput<double>("y", goal.y))
    // {
    //   throw BT::RuntimeError("Missing parameter y");
    // }
    // if(!getInput<double>("z", goal.z))
    // {
    //   throw BT::RuntimeError("Missing parameter z");
    // }
    if(!getInput<geometry_msgs::msg::PoseStamped>("pose_obj", goal.pose_obj))
    {
      throw BT::RuntimeError("Missing parameter pose_obj");
    }
  }

 BT::NodeStatus set_bt_result(
    typename rclcpp_action::ClientGoalHandle<PlaceAmir>::WrappedResult & result) override
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
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__PLACE_AMIR_NODE_HPP_
