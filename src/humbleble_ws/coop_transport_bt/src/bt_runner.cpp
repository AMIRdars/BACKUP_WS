#include <chrono>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "coop_transport_bt/cooperative_motion_bt_node.hpp"
#include "coop_transport_bt/transport_safety_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cooperative_transport_bt_runner");
  const auto default_xml = ament_index_cpp::get_package_share_directory("coop_transport_bt") +
    "/behavior_trees/transport_sequence.xml";
  node->declare_parameter<std::string>("bt_xml_path", default_xml);
  node->declare_parameter<double>("tick_rate", 10.0);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<coop_transport_bt::CooperativeMotionBTNode>("CooperativeMotion");
  factory.registerNodeType<coop_transport_bt::CheckTransportSafe>("CheckTransportSafe");
  factory.registerNodeType<coop_transport_bt::EmergencyStop>("EmergencyStop");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  const auto xml_path = node->get_parameter("bt_xml_path").as_string();
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_path, blackboard);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Unable to load BT XML '%s': %s", xml_path.c_str(), exception.what());
    rclcpp::shutdown();
    return 1;
  }
  const auto tick_rate = node->get_parameter("tick_rate").as_double();
  rclcpp::WallRate rate(tick_rate > 0.0 ? tick_rate : 10.0);
  RCLCPP_INFO(node->get_logger(), "Executing cooperative transport BT: %s", xml_path.c_str());
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickRoot();
    rate.sleep();
  }
  tree.haltTree();
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Cooperative transport BT completed successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Cooperative transport BT failed or was interrupted.");
  }
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
