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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__SEARCH_OBJ_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__SEARCH_OBJ_NODE_HPP_

#include <string>

#include "ros2_behavior_tree/ros2_action_client_node.hpp"
#include "behavior_tree_msgs/action/search.hpp"

namespace ros2_behavior_tree
{

using Search = behavior_tree_msgs::action::Search;

//  アームを回転させて複数の物体を取得する（情報収集用）
class SearchObjNode : public ros2_behavior_tree::ROS2ActionClientNode<Search>
{
public:
  explicit SearchObjNode(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2ActionClientNode<Search>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        BT::InputPort<std::string>("label", ""),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", ""),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", ""),
      });
  }

  void read_input_ports(Search::Goal & goal) override
  {
    // if (!getInput<nav_msgs::msg::Path>("path", goal.path)) {
    //   throw BT::RuntimeError("Missing parameter [path] in SearchObjNode node");
    // }

    if (!getInput<std::string>("label", goal.label)) {
      throw BT::RuntimeError("Missing parameter [label] in SearchObjNode node");
    }
  }

  void write_output_ports(
    rclcpp_action::ClientGoalHandle<Search>::WrappedResult & result) override
  {
    /*発見した物体位置を出力*/
    if (result.result->poses.size() > 0)
    {
      setOutput<geometry_msgs::msg::PoseStamped>("pose", result.result->poses.front());
    }
  }

  // アクション結果から最終的なBTの状態を返す
  BT::NodeStatus set_bt_result(
    typename rclcpp_action::ClientGoalHandle<Search>::WrappedResult & result) override
  {
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result_.result->poses.size() > 0)
        {
          geometry_msgs::msg::PoseStamped founded_pose;
          // founded_pose = result_.result->poses.front();
          founded_pose = result_.result->poses.front();
          
          RCLCPP_INFO(ros2_node_->get_logger(),"(x,y,z)=(%lf, %lf, %lf)", founded_pose.pose.position.x, founded_pose.pose.position.y, founded_pose.pose.position.z);
          setOutput<geometry_msgs::msg::PoseStamped>("pose", result_.result->poses.front());
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          RCLCPP_INFO(ros2_node_->get_logger(),"error_string = %s", result_.result->error_string.c_str());
          return BT::NodeStatus::FAILURE;
        }
        // write_output_ports(result_);

        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        return BT::NodeStatus::SUCCESS;

      default:
        throw std::logic_error("ROS2ActionClientNode::tick: invalid status value");
    }
  }

  // bool read_new_goal(Search::Goal & goal) override
  // {
  //   nav_msgs::msg::Path path;
  //   if (!getInput<nav_msgs::msg::Path>("path", path)) {
  //     throw BT::RuntimeError("Missing parameter [path] in SearchObjNode node");
  //   }

  //   // If it's not the same as the goal we're currently working on, update the goal
  //   // and return true
  //   return (path != goal_.path) ? goal_.path = path, true : false;
  // }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__SEARCH_OBJ_NODE_HPP_
