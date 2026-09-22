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

#ifndef ROS2_BEHAVIOR_TREE__DECORATOR__ONCE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__DECORATOR__ONCE_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ros2_behavior_tree
{

class OnceNode : public BT::DecoratorNode
{
public:
  OnceNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    executed_=false;
  }

  static BT::PortsList providedPorts()
  {
    return {
    };
  }

  BT::NodeStatus tick() override
  {
    if(executed_)
    {
      return BT::NodeStatus::SUCCESS;
    }
    
    setStatus(BT::NodeStatus::RUNNING);

    auto child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::SUCCESS:
        executed_ = true;
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::FAILURE:
        executed_ = true;
        return BT::NodeStatus::FAILURE;

      default:
        throw BT::LogicError("Invalid status return from BT node");
    }

    // None of the poses worked, so fail
    child()->halt();
    return BT::NodeStatus::FAILURE;
  }

private:
  bool executed_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__DECORATOR__FOR_EACH_POSE_NODE_HPP_
