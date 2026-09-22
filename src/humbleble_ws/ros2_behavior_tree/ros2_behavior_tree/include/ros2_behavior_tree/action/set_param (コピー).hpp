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

class SetParam : public BT::AsyncActionNode
{
  public:
    SetParam(const std::string & name, const BT::NodeConfiguration & config)
    : BT::AsyncActionNode(name, config)
    {
      server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
      ros2_node_ =  BT::TreeNode::config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("node");
      callback_group_ = ros2_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
      callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());

      
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("node_name", "The name of the service to call"),
        BT::InputPort<BT::RosParam>("param", ""),
        // BT::InputPort<std::chrono::milliseconds>("server_timeout",
        //   "The timeout value, in milliseconds, to use when waiting for service responses"),
        // BT::InputPort<std::shared_ptr<rclcpp::Node>>("ros2_node",
        //   "The ROS2 node to use when when creating the service")
      };
    }

  BT::NodeStatus tick() override
  {
    if (!getInput("node_name", node_name_)) {
      throw BT::RuntimeError("Missing parameter [node_name] in ROS2AsyncServiceClientNode");
    }

    if (parameters_client_ == nullptr) {
      parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        ros2_node_, 
        node_name_,
        rmw_qos_profile_parameters, 
        callback_group_);
    }

    // getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    // if (!getInput<std::chrono::milliseconds>("server_timeout", server_timeout_)) {
    //   throw BT::RuntimeError("Missing parameter [server_timeout] in ROS2AsyncServiceClientNode");
    // }
    // getInput<std::shared_ptr<rclcpp::Node>>("ros2_node", ros2_node_);
    // if (!getInput<std::shared_ptr<rclcpp::Node>>("ros2_node", ros2_node_)) {
    //   throw BT::RuntimeError("Missing parameter [ros2_node] in ROS2AsyncServiceClientNode");
    // }

    

    if (!getInput<BT::RosParam>("param", param_)) {
      throw BT::RuntimeError("Missing parameter [param] in ROS2AsyncServiceClientNode");
    }

    

    // auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(ros2_node_, "global_costmap/global_costmap");

    if (!parameters_client_->wait_for_service(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Timed out waiting for service \"%s\" to become available", node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }


    while(!parameters_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(ros2_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return BT::NodeStatus::FAILURE;
        }
            
        RCLCPP_INFO(ros2_node_->get_logger(), "service not available, waiting again...");
    }

    // パラメータの設定
    auto set_parameters_results = parameters_client_->set_parameters({
      rclcpp::Parameter("static_layer.enabled", false)
      // rclcpp::Parameter("parameter_string", "hello"),
      // rclcpp::Parameter("parameter_boolean", true),
    });

  

    //結果を受信するまで待機
    if(rclcpp::spin_until_future_complete(ros2_node_->get_node_base_interface(), set_parameters_results) != rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to set param.");
        return BT::NodeStatus::FAILURE;
    }
    
    for (auto & result : set_parameters_results.get()) {
      if (!result.successful) {
        std::cerr << "Failed to set parameter: " << result.reason << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::SUCCESS;
    // auto future_result = service_client_->async_send_request(request_);
    // for (;; ) {
    //   switch (future_result.wait_for(server_timeout_)) {
    //     case std::future_status::ready:
    //       response_ = future_result.get();
    //       write_output_ports(response_);
    //       return BT::NodeStatus::SUCCESS;

    //     case std::future_status::timeout:
    //       break;

    //     default:
    //       return BT::NodeStatus::FAILURE;
    //   }
    // }
  }

  protected:
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::chrono::milliseconds server_timeout_;
    std::string node_name_;
    std::string param_name_;
    BT::RosParam param_;
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__SET_PARAM_HPP_
