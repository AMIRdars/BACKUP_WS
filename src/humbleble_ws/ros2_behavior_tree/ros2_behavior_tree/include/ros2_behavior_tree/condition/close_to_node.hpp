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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__CLOSE_TO_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__CLOSE_TO_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_behavior_tree
{

class CloseToNode : public BT::ConditionNode
{
public:
  CloseToNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  CloseToNode() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("obj_pose", ""),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("location_pose", ""),
      BT::InputPort<double>("outer_radius", "threshold"),
      BT::InputPort<double>("inner_radius", "threshold"),
      BT::InputPort<std::string>("obj_id", ""), //xmlに余計なものがあるとエラーが出るので
      BT::InputPort<std::string>("location_id", ""),
      BT::OutputPort<double>("distance", "distance"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped obj_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("obj_pose", obj_pose)) {
      throw BT::RuntimeError("Missing parameter [obj_pose] in CloseTo node");
    }

    geometry_msgs::msg::PoseStamped location_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("location_pose", location_pose)) {
      throw BT::RuntimeError("Missing parameter [location_pose] in CloseTo node");
    }

    double outer_radius;
    if (!getInput<double>("outer_radius", outer_radius)) {
      throw BT::RuntimeError("Missing parameter [outer_radius] in CloseTo node");
    }

    double inner_radius;
    if (!getInput<double>("inner_radius", inner_radius)) {
      inner_radius = 0;
    }

    double distance 
    = sqrt(pow(obj_pose.pose.position.x - location_pose.pose.position.x,2) 
        + pow(obj_pose.pose.position.y - location_pose.pose.position.y, 2));

    setOutput<double>("distance", distance);


    if(distance >= inner_radius && distance <= outer_radius)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }

  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__CLOSE_TO_NODE_HPP_
