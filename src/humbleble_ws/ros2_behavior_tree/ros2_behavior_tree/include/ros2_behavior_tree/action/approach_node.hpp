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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__APPROACH_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__APPROACH_NODE_HPP_

#include <string>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "behavior_tree_msgs/action/gen_approach.hpp"

namespace ros2_behavior_tree
{

using Approach = behavior_tree_msgs::action::GenApproach;

class ApproachNode : public ros2_behavior_tree::ROS2ActionClientNode<Approach>
{
public:
  explicit ApproachNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<Approach>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "The destination to plan to"),
      });
  }

  void read_input_ports(Approach::Goal & goal) override
  {
    if (!getInput<geometry_msgs::msg::PoseStamped>("goal", goal.pose)) {
      throw BT::RuntimeError("Missing parameter [goal] in ComputePathToPoseNode node");
    }
  }

};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__APPROACH_NODE_HPP_
