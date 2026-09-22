#ifndef ROS2_BEHAVIOR_TREE__ACTION__CREATE_BT_GEN_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CREATE_BT_GEN_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp" 
#include "behavior_tree_msgs/srv/create_bt.hpp"

namespace ros2_behavior_tree
{

class CreateBTGenNode : public BT::AsyncActionNode
{
  public:
    CreateBTGenNode(const std::string & name, const BT::NodeConfiguration & config)
    : BT::AsyncActionNode(name, config)
    {
      request_ = std::make_shared<behavior_tree_msgs::srv::CreateBT::Request>();
      response_ = std::make_shared<behavior_tree_msgs::srv::CreateBT::Response>();
      server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
      ros2_node_ =  BT::TreeNode::config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("node");
      callback_group_ = ros2_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
      callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());
    }

    CreateBTGenNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("service_name", "The name of the service to call"),
        BT::InputPort<std::string>("bt_name", ""),
        BT::InputPort<std::string>("command", "add, remove"),
      };
    }

  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }

    
    if (!getInput("command", command_)) {
      throw BT::RuntimeError("Missing parameter [command] in ROS2ServiceClientNode");
    }

    if (!getInput("bt_name", bt_name_)) {
      throw BT::RuntimeError("Missing parameter [bt_name] in ROS2ServiceClientNode");
    }

    if (service_client_ == nullptr) {
      service_client_ = ros2_node_->create_client<behavior_tree_msgs::srv::CreateBT>(service_name_);
    }


    // Make sure the server is actually there before continuing
    if (!service_client_->wait_for_service(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Timed out waiting for service \"%s\" to become available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }


    //  リクエスト生成
    request_->bt_name = bt_name_;
    if(command_=="add")
      request_->command = "add";
    else if(command_=="remove")
      request_->command = "remove";
    else
      RCLCPP_ERROR(ros2_node_->get_logger(), "invalid input");
      
    //結果を受信するまで待機
    // Send the request to the server
    auto future_result_ = service_client_->async_send_request(request_);

    std::cout << server_timeout_.count() << std::endl;
    rclcpp::FutureReturnCode rc;
    rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);


    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      response_ = future_result_.get();
      return BT::NodeStatus::SUCCESS;
    } else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to set param.");
        return BT::NodeStatus::FAILURE;
    } else {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Call to \"%s\" server failed", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

  }


  protected:
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::chrono::milliseconds server_timeout_;
    std::string node_name_;
    rclcpp::Client<behavior_tree_msgs::srv::CreateBT>::SharedPtr service_client_;
    std::shared_ptr<behavior_tree_msgs::srv::CreateBT::Request> request_;
    std::shared_ptr<behavior_tree_msgs::srv::CreateBT::Response> response_;
    std::string service_name_;
    std::string command_;
    std::string bt_name_;
    std::shared_future<std::shared_ptr<behavior_tree_msgs::srv::CreateBT_Response>> future_result_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CREATE_BT_GEN_NODE_HPP_
