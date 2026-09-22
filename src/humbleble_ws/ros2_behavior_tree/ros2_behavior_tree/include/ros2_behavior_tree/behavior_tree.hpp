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

#ifndef ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_HPP_
#define ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <ctime>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <ros2_behavior_tree/bt_status_pub.hpp>
#include "rclcpp/rclcpp.hpp"

namespace ros2_behavior_tree
{

// The possible return values from the execution of a Behavior Tree
enum class BtStatus { SUCCEEDED, FAILED, HALTED };

class BehaviorTree 
{
public:
  explicit BehaviorTree(
    const std::string & bt_xml,
    const std::vector<std::string> & plugin_library_names = {"ros2_behavior_tree_nodes"},
    const std::string & bt_name = ""
  );
  BehaviorTree() = delete;
  virtual ~BehaviorTree() {}


  void register_tree(const std::string & bt_xml_path);  //  サブツリーの登録用

  void update_xml(const std::string & bt_xml);
  // void addGrootMonitoring(
  //   BT::Tree * tree,
  //   uint16_t publisher_port,
  //   uint16_t server_port,
  //   uint16_t max_msg_per_second);

  void addStatusPub();
  void getStatus(behavior_tree_msgs::msg::BTStatus& bt_status);

  void executeTick(BT::NodeStatus &result, std::vector<BT::TreeNode::Ptr> &nodes);
  void haltTree();
  BT::NodeStatus tickRoot();

  BtStatus execute(
    std::function<bool()> should_halt = []() {return false;},
    std::function<void()> on_loop_iteration = []() {},
    std::chrono::milliseconds tick_period = std::chrono::milliseconds(10));

  BT::Blackboard::Ptr blackboard() {return blackboard_;}
  BT::BehaviorTreeFactory & factory() {return factory_;}

  bool set_tree;

protected:
  // The factory to use when dynamically constructing the Behavior Tree
  BT::BehaviorTreeFactory factory_;

  // XML parser to parse the supplied BT XML input
  BT::XMLParser xml_parser_;

  // The blackboard to be shared by all of the Behavior Tree's nodes
  BT::Blackboard::Ptr blackboard_;

  std::unique_ptr<BT::StatusPub> status_pub_;

  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;

  std::unique_ptr<BT::FileLogger> file_logger_;
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;

  std::string filename_;

  // BT::Tree tree_;
  std::unique_ptr<BT::Tree> tree_;
  BT::NodeStatus root_status_;

  std::string bt_name_;
  int update_cnt_=0;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_HPP_
