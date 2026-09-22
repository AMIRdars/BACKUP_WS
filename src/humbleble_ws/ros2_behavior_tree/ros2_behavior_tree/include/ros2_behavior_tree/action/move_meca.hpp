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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__MOVE_MECA_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__MOVE_MECA_HPP_

#include <string>
// #include "rclcpp/rclcpp.hpp"
#include "ros2_behavior_tree/ros2_action_client_node.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
#include "behavior_tree_msgs/action/move_meca.hpp"

//もともとslowdown

namespace ros2_behavior_tree
{

using MoveMeca = behavior_tree_msgs::action::MoveMeca;
// using MoveMecaAction = behavior_tree_msgs::action::MoveMeca;

class MoveMecaNode : public ros2_behavior_tree::ROS2ActionClientNode<MoveMeca>
{
public:
  explicit MoveMecaNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<MoveMeca>(name, config)
  {    
    // node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // movemeca_pub = node_->create_publisher<std_msgs::msg::Empty>("/amir/move_meca", 1);
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
      BT::InputPort<double>("posi_x", "Target x position"),
      BT::InputPort<double>("posi_y", "Target y position")
      });
  }

  void read_input_ports(MoveMeca::Goal & goal) override
  {
    // msg = behavior_tree_msgs::msg::Movemeca();
    if(!getInput<double>("posi_x", goal.posi_x))
    {
      throw BT::RuntimeError("Missing parameter x");
    }
    if(!getInput<double>("posi_y", goal.posi_y))
    {
      throw BT::RuntimeError("Missing parameter y");
    }
  }

  // bool read_new_goal(MoveMeca::Goal & goal) override
  // {
  //   double x;
  //   double y;
  // }

  // void write_feedback_ports(const std::shared_ptr<const MoveMeca::Feedback> feedback)
  // {}
  // void write_output_ports(rclcpp_action::ClientGoalHandle<MoveMeca>::WrappedResult & result) override
  // {


  // }


  BT::NodeStatus set_bt_result(
    typename rclcpp_action::ClientGoalHandle<MoveMeca>::WrappedResult & result) override
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
  // protected:
  //   std::shared_ptr<rclcpp::Node> node_;
  //   rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr movemeca_pub;
  
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__PID_BEBOP_NODE_HPP_
