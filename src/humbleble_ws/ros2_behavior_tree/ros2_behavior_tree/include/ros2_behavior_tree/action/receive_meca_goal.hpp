#ifndef ROS2_BEHAVIOR_TREE__ACTION__RECEIVE_MECA_GOAL_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__RECEIVE_MECA_GOAL_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_behavior_tree
{

// /amir/destination (geometry_msgs/PoseStamped) を受け取り、
// posi_x / posi_y をブラックボードに書き込む。
//
// StatefulActionNode を使い、メッセージが届くまで RUNNING を返し続ける。
// サブスクリプションはコンストラクタで一度だけ作成するため、
// `ros2 topic pub --once` のような単発 publish でも確実に受信できる。
class ReceiveMecaGoalNode : public BT::StatefulActionNode
{
public:
  ReceiveMecaGoalNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // 永続的なサブスクリプション: ノードが生きている間は常に受信待機
    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/amir/destination",
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_msg_ = *msg;
        received_.store(true);
        RCLCPP_INFO(node_->get_logger(),
          "[ReceiveMecaGoal] 目標受信: x=%.3f y=%.3f",
          msg->pose.position.x, msg->pose.position.y);
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>("posi_x", "目標 X 座標 [m]"),
      BT::OutputPort<double>("posi_y", "目標 Y 座標 [m]")
    };
  }

  // BT ティックごとに呼ばれる開始処理
  BT::NodeStatus onStart() override
  {
    // 既にメッセージが届いていればすぐに SUCCESS
    if (received_.load()) {
      return applyMsg();
    }
    RCLCPP_INFO(node_->get_logger(),
      "[ReceiveMecaGoal] /amir/destination 待機中...");
    return BT::NodeStatus::RUNNING;
  }

  // RUNNING 中に毎ティック呼ばれる
  BT::NodeStatus onRunning() override
  {
    if (!received_.load()) {
      return BT::NodeStatus::RUNNING;
    }
    return applyMsg();
  }

  void onHalted() override {}

private:
  BT::NodeStatus applyMsg()
  {
    geometry_msgs::msg::PoseStamped msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_msg_;
      received_.store(false);  // 次の呼び出しに備えてリセット
    }
    setOutput("posi_x", msg.pose.position.x);
    setOutput("posi_y", msg.pose.position.y);
    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped latest_msg_;
  std::atomic<bool> received_{false};
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__RECEIVE_MECA_GOAL_HPP_
