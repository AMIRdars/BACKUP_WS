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

#include "ros2_behavior_tree/behavior_tree.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"


namespace ros2_behavior_tree
{

BehaviorTree::BehaviorTree(
  const std::string & bt_xml,
  const std::vector<std::string> & plugin_library_names,
  const std::string & bt_name)
: xml_parser_(factory_), bt_name_(bt_name)
{
  // Load any specified BT plugins
  for (const auto & library_name : plugin_library_names) {
    factory_.registerFromPlugin(std::string{"lib" + library_name + ".so"});
  }
  // BT::SharedLibrary loader;
  // for (const auto & p : plugin_library_names) {
  //   factory_.registerFromPlugin(loader.getOSName(p));
  // }

  // Parse the input XML
  // xml_parser_.loadFromText(bt_xml);

  // Create a blackboard for this Behavior Tree
  blackboard_ = BT::Blackboard::create();

  set_tree=false;
  // update_xml(bt_xml);
}

void BehaviorTree::register_tree(const std::string & bt_xml_path)
{
  factory_.registerBehaviorTreeFromFile(bt_xml_path);

  
}

void BehaviorTree::update_xml(const std::string & bt_xml)
{
  update_cnt_++;
  std::cout << "cc" << std::endl;
  groot_monitor_.reset();
  status_pub_.reset();
  file_logger_.reset();
  cout_logger_.reset();
  tree_.reset();
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(bt_xml, blackboard_));
  cout_logger_ = std::make_unique<BT::StdCoutLogger>(*tree_);


  // factory_.createTreeを使うと登録済みのxmlの変更ができないので没
  // factory_.registerBehaviorTreeFromText(bt_xml);
  // //Check that the BTs have been registered correctly
  // std::cout << "Registered BehaviorTrees:" << std::endl;
  // for(const std::string& bt_name: factory_.registeredBehaviorTrees())
  // {
  //     std::cout << " - " << bt_name << std::endl;
  // }

  
  // tree_ = std::make_unique<BT::Tree>(factory_.createTree("MainTree", blackboard_));

  // tree_ = factory_.createTreeFromText(bt_xml, blackboard_);
  // xml_parser_.loadFromText(bt_xml);

  

  // time_t now = time(0);
  // tm *ltm = localtime(&now);
  filename_ = "BT_Log_" + bt_name_;
  filename_.append(std::to_string(update_cnt_));
  // filename_.append("_");
  // filename_.append(std::to_string(ltm->tm_mday));
  // filename_.append("_");
  // filename_.append(std::to_string(ltm->tm_hour));
  // filename_.append("_");
  // filename_.append(std::to_string(ltm->tm_min));
  
  filename_.append(".fbl");
  
  set_tree = true;
}

// void BehaviorTree::addGrootMonitoring(
//   BT::Tree * tree,
//   uint16_t publisher_port,
//   uint16_t server_port,
//   uint16_t max_msg_per_second)
// {
//   // This logger publish status changes using ZeroMQ. Used by Groot
//   groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
//     *tree, max_msg_per_second, publisher_port,
//     server_port);
// }

void BehaviorTree::addStatusPub()
{
  status_pub_ = std::make_unique<BT::StatusPub>(*tree_);
  // groot_monitor_ = std::make_unique<BT::PublisherZMQ>(*tree_, 25, 1668, 1669);
  file_logger_ = std::make_unique<BT::FileLogger>(*tree_, filename_.c_str());
}

void BehaviorTree::getStatus(behavior_tree_msgs::msg::BTStatus& bt_status)
{
  status_pub_->get_bt_status(bt_status);
  bt_status.root_status = status_pub_->convert_status_msg(root_status_);
  
}



void BehaviorTree::executeTick(BT::NodeStatus &result, std::vector<BT::TreeNode::Ptr> &nodes)
{
  result = tree_->tickRoot();
  root_status_ = result;
  nodes = tree_->nodes;
}

BT::NodeStatus BehaviorTree::tickRoot()
{
  return tree_->tickRoot();
}

void BehaviorTree::haltTree()
{ 
  tree_->haltTree();
}


BtStatus
BehaviorTree::execute(
  std::function<bool()> should_halt,
  std::function<void()> on_loop_iteration,
  std::chrono::milliseconds tick_period)
{
  // Create the corresponding Behavior Tree
  // BT::Tree tree = xml_parser_.instantiateTree(blackboard_);

  // auto tree = std::make_unique<BT::Tree>( factory.createTreeFromText( xml_text, my_blackboard) );
  
  // auto logger_file = std::make_unique<BT::FileLogger>(*tree, filename1.c_str());
  // addGrootMonitoring(&tree, 1666, 1667, 25);
  // groot_monitor_ = std::make_unique<BT::PublisherZMQ>(&tree);

  
  // Set up a loop rate controller based on the desired tick period
  rclcpp::WallRate loop_rate(tick_period);

  // Loop until something happens with ROS or the node completes
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (should_halt()) {
      tree_->rootNode()->halt();
      return BtStatus::HALTED;
    }

    // Execute one tick of the tree
    result = tree_->rootNode()->executeTick();

    for(auto node : tree_->nodes)
    {
      std::cout <<  node->UID() << std::endl;
      std::cout <<  node->name() << std::endl;
      switch (node->status()) {
        case BT::NodeStatus::IDLE:
          // std::cout << "BT was halted\n";
          break;
        case BT::NodeStatus::RUNNING:
          break;

        case BT::NodeStatus::SUCCESS:
          std::cout << "SUCCESS\n";
          break;

        case BT::NodeStatus::FAILURE:
          std::cout << "FAILURE\n";
          break;

        default:
          throw std::logic_error("Invalid return value from the BT");
      }
    }
    

    // Give the caller a chance to do something on each loop iteration
    on_loop_iteration();

    // Throttle the BT loop rate, based on the provided tick period value
    loop_rate.sleep();
  }

  groot_monitor_.reset();

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

}  // namespace ros2_behavior_tree
