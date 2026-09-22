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

#ifndef ROS2_BEHAVIOR_TREE__CONTROL__PRIORITY_SEQUENCE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONTROL__PRIORITY_SEQUENCE_NODE_HPP_

#include <sstream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <iostream>
#include <numeric>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "behaviortree_cpp_v3/control_node.h"


/*設計方針
*優先順位順にこのノードを実行する
*優先順位が変更された場合、実行中の子ノードの優先順位が下がったら停止する（変更が繰り返されて振動する可能性があるが、優先順位を送るプランナ側で管理して防止する）
*子ノードが失敗した場合、次の優先順位のノードを実行する
*/


namespace ros2_behavior_tree
{

class PrioritySequenceNode : public BT::ControlNode
{
public:
  explicit PrioritySequenceNode(const std::string & name)
  : BT::ControlNode(name, {})
  {
  }

  PrioritySequenceNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config)
  ,current_child_idx_(0)
  {
    executed_children_ = {};
  }

  static BT::PortsList providedPorts() 
  {
    return {
      BT::InputPort<std_msgs::msg::Float64MultiArray>("priority", "priority_array")
    };
  }

  void halt() override
  {
    BT::ControlNode::halt();
  }

protected:
  

  BT::NodeStatus tick() override
  {
    const size_t children_count = children_nodes_.size();
    setStatus(BT::NodeStatus::RUNNING);


    std_msgs::msg::Float64MultiArray priority_array;
    if (!getInput<std_msgs::msg::Float64MultiArray>("priority", priority_array)) {
      throw BT::RuntimeError("Missing parameter [priority] in PrioritySequenceNode");
    }
    //  優先度の配列サイズと子ノードのサイズが異なる場合エラー
    if (priority_array.data.size() != children_nodes_.size()){
      throw BT::RuntimeError("Invalid parameter [priority] in PrioritySequenceNode");
    }

    std::vector<double> array = priority_array.data;

    // 配列のインデックス indiecs を作成する。
    std::vector<size_t> indices(array.size());
    std::iota(indices.begin(), indices.end(), 0);

    // ソートする。
    std::sort(indices.begin(), indices.end(), [&array](size_t i1, size_t i2) {
        return array[i1] < array[i2];
    });

    for (auto index : indices){
      if (array[index] == -1) continue; //  優先度が-1の場合、実行しない
      //  実行済みの場合、実行しない
      if (std::any_of(executed_children_.begin(), executed_children_.end(), [index](size_t n){ return index == n; })){ 
        continue;
      }

      
      TreeNode* current_child_node = children_nodes_[index];
      const BT::NodeStatus child_status = current_child_node->executeTick();
      
      

      switch (child_status)
      {
          case BT::NodeStatus::RUNNING:
          {
              return child_status;
          }
          case BT::NodeStatus::SUCCESS:
          {
              haltChildren();
              current_child_idx_ = 0;
              return child_status;
          }
          case BT::NodeStatus::FAILURE:
          {
              executed_children_.push_back(index); //  実行済みリストに追加
          }
          break;

          case BT::NodeStatus::IDLE:
          {
              throw std::runtime_error("A child node must never return IDLE");
          }
      }   // end switch

    }

    if (children_count == executed_children_.size())
    {
      haltChildren();
      current_child_idx_ = 0;
    } 


    // // Wrap up
    // haltChildren();
    // last_child_ticked_ = 0;  // reset
    return BT::NodeStatus::FAILURE;
  }

  std::size_t last_child_ticked_ = 0;
  size_t current_child_idx_;  //  実行中の子ノードインデックス
  std::vector<size_t> executed_children_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONTROL__PRIORITY_SEQUENCE_NODE_HPP_
