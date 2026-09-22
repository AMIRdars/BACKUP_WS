#ifndef ROS2_BEHAVIOR_TREE__ACTION__SET_MECA_GOAL_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__SET_MECA_GOAL_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"

namespace ros2_behavior_tree
{

// XML に直接書いた x/y 座標を posi_x / posi_y としてブラックボードに書き込む。
// ReceiveMecaGoal の「座標を手動指定したい」版。
//
// 使用例:
//   <SetMecaGoal x="1.5" y="0.0" posi_x="{posi_x}" posi_y="{posi_y}"/>
class SetMecaGoalNode : public BT::SyncActionNode
{
public:
  SetMecaGoalNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", "目標 X 座標 [m]"),
      BT::InputPort<double>("y", "目標 Y 座標 [m]"),
      BT::OutputPort<double>("posi_x", ""),
      BT::OutputPort<double>("posi_y", "")
    };
  }

  BT::NodeStatus tick() override
  {
    double x, y;
    if (!getInput<double>("x", x) || !getInput<double>("y", y)) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("posi_x", x);
    setOutput("posi_y", y);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__SET_MECA_GOAL_HPP_
