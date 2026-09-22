#ifndef ROS2_BEHAVIOR_TREE__ACTION__CHANGE_NODE_STATUS_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CHANGE_NODE_STATUS_HPP_

#include "ros2_behavior_tree/behavior_tree.hpp"
// #include "ros2_behavior_tree/node_thread.hpp"
#include "ros2_behavior_tree/ros2_async_service_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace ros2_behavior_tree
{

class ChangeNodeStatus : public ROS2AsyncServiceClientNode<lifecycle_msgs::srv::ChangeState>
{
public:
  ChangeNodeStatus(const std::string & name, const BT::NodeConfiguration & config)
  : ROS2AsyncServiceClientNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return augment_basic_ports({
        BT::InputPort<std::string>("target_state", ""),
      });
  }

    void read_input_ports(std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request) override
    {
        if (!getInput<std::string>("target_state", request->transition.label)) {
        throw BT::RuntimeError("Missing parameter [target_state] in ChangeNodeStatus node");
        }
    }

    
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CHANGE_NODE_STATUS_HPP_