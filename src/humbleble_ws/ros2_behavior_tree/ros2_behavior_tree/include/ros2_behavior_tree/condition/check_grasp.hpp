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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__CHECK_GRASP_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__CHECK_GRASP_HPP_

#include <string>
#include <memory>
#include <math.h>
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "moveit_msgs/msg/attached_collision_object.hpp"
// #include "amir_interfaces/msg/amir_cmd.hpp" //amirとのやり取りをするためのpub用型
// #include "amir_interfaces/msg/amir_sensor.hpp" 
#include "rclcpp/wait_for_message.hpp"
#include <std_msgs/msg/float32.hpp>

namespace ros2_behavior_tree
{

class CheckGraspNode : public BT::ConditionNode
{
public:
  CheckGraspNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    gripper_check("/gripper_value")
    // grasp_state_topic_("gripper_state"),
    // is_grasped_obj_(false),
    // block_grasp_check_(false)
  {
    node_=BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;

  }

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<std::shared_ptr<rclcpp::Node>>("node_handle", "The ROS2 node to use"),
      // BT::InputPort<std::string>(
      //   "grasp_state_topic", std::string("gripper_state"), "gripper_state"),
      // BT::InputPort<std::string>("obj_id", "grasped obj id"),
      // BT::InputPort<moveit_msgs::msg::AttachedCollisionObject>("grasped_object", ""), //  pick_nodeの把持物を取得
      // BT::InputPort<bool>("block_grasp_check", " "),  //  trueなら判定しないで以前の判定を用いる
      // BT::InputPort<amir_interfaces::msg::AmirSensor>("/encoder_pub")
      BT::InputPort<std_msgs::msg::Float32>("/gripper_value")

    };
  }

  BT::NodeStatus tick() override
  {
    // while (!rclcpp::wait_for_message(encoder_sub_,ros2_node_,"/encoder_pub",server_timeout_))
    // {
    //   // RCLCPP_DEBUG(node_->get_logger(),"timeout /encoder topic");
    //   return BT::NodeStatus::FAILURE;
    // }
    // current_angle6 = encoder_sub_.angle[5];

    // while (!rclcpp::wait_for_message(gripper_topic_,node_,"/gripper_value",server_timeout_))
    // {
    //   RCLCPP_DEBUG(node_->get_logger(),"timeout /gripper_value topic");
    //   return BT::NodeStatus::FAILURE;
    // }
    while (!rclcpp::wait_for_message(gripper_topic_,node_,"/gripper_value"))
    {
      RCLCPP_DEBUG(node_->get_logger(),"timeout /gripper_value topic");
      return BT::NodeStatus::FAILURE;
    }

    current_angle6 = gripper_topic_.data;

    if (current_angle6 < -600)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      // return BT::NodeStatus::FAILURE;
      return BT::NodeStatus::SUCCESS;
    } 
    // std::string source_frame;
    // if (!getInput<std::string>("source_frame", source_frame)) {
    //   throw BT::RuntimeError("Missing parameter [source_frame] in CanTransform node");
    // }

    // RCLCPP_INFO(ros2_node_->get_logger(),"grasped_object is %s",grasped_object.object.id.c_str());
    
    // if (is_grasped_obj_ && grasped_object.object.id == obj_id) {
    //   return BT::NodeStatus::SUCCESS;
    // }
    // return BT::NodeStatus::FAILURE;



  }

private:
  // rclcpp::Node::SharedPtr ros2_node_;
  rclcpp::Node::SharedPtr node_;
  // rclcpp::CallbackGroup::SharedPtr callback_group_;
  // rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_state_sub_;
  // rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr encoder_sub_;
  // std::string encoder_check;
  double current_angle6;
  // std::string grasp_state_topic_;
  BT::NodeStatus b_status_;
  std::chrono::milliseconds server_timeout_;
  std::string gripper_check;
  std_msgs::msg::Float32 gripper_topic_;
  // bool is_grasped_obj_; //  何か把持していればTrue
  // bool block_grasp_check_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__IS_GRASPED_OBJ_NODE_HPP_
