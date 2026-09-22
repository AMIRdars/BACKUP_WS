#ifndef ROS2_BEHAVIOR_TREE__ACTION__SET_PARAM_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__SET_PARAM_HPP_

#include <string>
#include <memory>
 
#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

namespace ros2_behavior_tree
{

class SetParam : public BT::ActionNodeBase
{
  public:
    SetParam(const std::string & name, const BT::NodeConfiguration & config)
    : BT::ActionNodeBase(name, config)
    {
      bt_loop_duration_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
      server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
      ros2_node_ =  BT::TreeNode::config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("node");
      callback_group_ = ros2_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
      callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());

      if (!getInput("node_name", node_name_)) {
      throw BT::RuntimeError("Missing parameter [node_name] in SetParamNode");
      }

      RCLCPP_INFO(ros2_node_->get_logger(), "%s",node_name_.c_str());

      parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
          ros2_node_, 
          node_name_,
          rmw_qos_profile_parameters, 
          callback_group_);

      parameters_client_->wait_for_service();
    }

    SetParam() = delete;

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("node_name", "The name of the service to call"),
        BT::InputPort<std::string>("param_name", "param_name"),
        BT::InputPort<bool>("bool_value", ""),
        BT::InputPort<int>("int_value", ""),
        BT::InputPort<double>("double_value", ""),
        BT::InputPort<std::string>("string_value", ""),
      };
    }

  BT::NodeStatus tick() override
  {
    std::string param_name;
    if (!getInput<std::string>("param_name", param_name)) {
      throw BT::RuntimeError("Missing parameter [param_name] in SetParamNode");
    }

    bool bool_value;
    int int_value;
    double double_value;
    std::string string_value;
    
    if (getInput<bool>("bool_value", bool_value))
    {
      param_ = rclcpp::Parameter(param_name, bool_value);
    }
    else if (getInput<int>("int_value", int_value))
    {
      param_ = rclcpp::Parameter(param_name, int_value);
    }
    else if (getInput<double>("double_value", double_value))
    {
      param_ = rclcpp::Parameter(param_name, double_value);
    }
    else if (getInput<std::string>("string_value", string_value))
    {
      param_ = rclcpp::Parameter(param_name, string_value);
    }
    else
    {
      throw BT::RuntimeError("Missing parameter [~value] in SetParamNode");
    }


    if (!request_sent_) {
      future_result_ = parameters_client_->set_parameters({param_});
      sent_time_ = ros2_node_->now();
      request_sent_ = true;
    }
    return check_future();
  }

  void halt() override
  {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  virtual BT::NodeStatus check_future()
  {
    auto elapsed = (ros2_node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        return BT::NodeStatus::SUCCESS;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        elapsed = (ros2_node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }


    request_sent_ = false;
    for (auto & result : future_result_.get()) {
      if (!result.successful) {
        std::cerr << "Failed to set parameter: " << result.reason << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::FAILURE;
  }



  protected:
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::chrono::milliseconds server_timeout_;
    std::string node_name_;
    rclcpp::Parameter param_;

    // The timeout value for BT loop execution
    std::chrono::milliseconds bt_loop_duration_;

    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    bool request_sent_{false};
    rclcpp::Time sent_time_;
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future_result_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__SET_PARAM_HPP_
