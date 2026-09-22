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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__IS_HAND_FREE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__IS_HAND_FREE_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_behavior_tree
{

class IsHandFreeNode : public BT::ConditionNode
{
public:
  IsHandFreeNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    grasp_state_topic_("gripper_state"),
    is_grasped_obj_(false),
    block_grasp_check_(false)
  {
    ros2_node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = ros2_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,false);
    callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());


    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    grasp_state_sub_ = ros2_node_->create_subscription<std_msgs::msg::Bool>(
    grasp_state_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsHandFreeNode::GraspStateCB, this, std::placeholders::_1),
    sub_option);

  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("grasp_state_topic", std::string("gripper_state"), "gripper_state"),
      BT::InputPort<bool>("block_grasp_check", " "),  //  trueなら判定しないで以前の判定を用いる
      BT::OutputPort<moveit_msgs::msg::AttachedCollisionObject>("grasped_object", ""),
    };
  }

  BT::NodeStatus tick() override
  {
    getInput<bool>("block_grasp_check", block_grasp_check_);
    if(!block_grasp_check_) callback_group_executor_.spin_some(); //  block中はsubscribeしない

    //  blockしてない場合に離した場合はgrasped_objectのidをemptyにする。
    if(!is_grasped_obj_ && !block_grasp_check_)
    {
      moveit_msgs::msg::AttachedCollisionObject grasped_object;
      grasped_object.object.id = "empty";
      setOutput<moveit_msgs::msg::AttachedCollisionObject>("grasped_object", grasped_object);
    }

    if (!is_grasped_obj_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }


  void GraspStateCB(std_msgs::msg::Bool::SharedPtr msg)
  {
    is_grasped_obj_ = msg->data;  
  }

private:
  rclcpp::Node::SharedPtr ros2_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_state_sub_;
  std::string grasp_state_topic_;

  bool is_grasped_obj_; //  何か把持していればTrue
  bool block_grasp_check_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__IS_HAND_FREE_NODE_HPP_
