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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__CHECK_HELP_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__CHECK_HELP_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include <math.h>
// #include "geometry/msg/posestamped.hpp"
// #include "observation_msgs/msg/object.hpp"


namespace ros2_behavior_tree
{

class CheckHelpNode : public BT::ConditionNode
{
public:
  CheckHelpNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    help_pub_drone("/bebop/drone_help")
    //task_topic_("task")
    //drone_goal_path_topic_("local_plan")
  {
    node_=BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
    // self.create_subscription(Float64,"/bebop/drone_help",self)
  }

  static BT::PortsList providedPorts()
  {
    return {
      //BT::InputPort<std::string>("task_topic_",std::string("task") ,"task"),
      // BT::InputPort<std::string>("drone_goal_path_topic_",std::string("local_plan") ,"local_plan"),//経路生成時起動用
      BT::InputPort<std::string>("help_pub_drone",std::string("/bebop/drone_help") ,"drone_help"),

    };
  }

  BT::NodeStatus tick() override
  {
    
    //rclcpp::Duration(1.0);
    //     while (!rclcpp::wait_for_message(help_topic,node_,"/bebop/drone_help"))
    // {
    //   RCLCPP_DEBUG(node_->get_logger(),"timeout /bebop/drone_help topic");
    //   return BT::NodeStatus::FAILURE;
    
    while (!rclcpp::wait_for_message(help_topic,node_,"/bebop/drone_help",server_timeout_))
    {
      RCLCPP_DEBUG(node_->get_logger(),"timeout /bebop/drone_help topic");
      return BT::NodeStatus::FAILURE;
    }

    number = help_topic.data;
    

    rclcpp::Rate loop_rate(1);
  
    if(number == 1.0)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else 
    {
      return BT::NodeStatus::FAILURE;

    }

    

  }

private:
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped drone_plan_;
  std_msgs::msg::Float64 drone_task_;
  std::string task_topic_;
  std::string drone_goal_path_topic_;

  //std_msgs::msg::drone_help;
  std_msgs::msg::Float64 help_topic;
  std::string help_pub_drone;

  double number;
  double path_x;

  BT::NodeStatus b_status_;

  std::chrono::milliseconds server_timeout_;



};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__CHECK_ALTITUDE_HIGH_NODE_HPP_
