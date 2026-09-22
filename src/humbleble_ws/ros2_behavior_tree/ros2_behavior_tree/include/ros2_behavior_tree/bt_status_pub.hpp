// #ifndef BEHAVIOR_TREE_BT_STATUS_PUB_HPP_
// #define BEHAVIOR_TREE_BT_STATUS_PUB_HPP_

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
// #include "behavior_tree_msgs/msg/bt_status.hpp"
#include "behavior_tree_msgs/msg/bt_status.hpp"
#include "rclcpp/rclcpp.hpp"

//rclcpp::Node::SharedPtr node, 


namespace BT
{
class StatusPub : public StatusChangeLogger
{
  public:
    StatusPub(const BT::Tree& tree);

    ~StatusPub() override;

    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;
    behavior_tree_msgs::msg::NodeStatus convert_status_msg(const BT::NodeStatus& bt_node_status);
    behavior_tree_msgs::msg::NodeStatus convert_status_msg(const TreeNode& bt_node);
    void get_bt_status(behavior_tree_msgs::msg::BTStatus& bt_status);

  private:
    behavior_tree_msgs::msg::BTStatus bt_status_;
};

// StatusPub::StatusPub(rclcpp::Node::SharedPtr node, const BT::Tree& tree) : StatusChangeLogger(tree.rootNode())
// {
//     publisher_ = node->create_publisher<behavior_tree_msgs::msg::BTStatus>("bt_status", 10);
//     // bool expected = false;
//     // if (!ref_count.compare_exchange_strong(expected, true))
//     // {
//     //     throw LogicError("Only one instance of StdCoutLogger shall be created");
//     // }
// }

// StatusPub::~StatusPub(){};

// void StatusPub::flush(){};

// void StatusPub::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
//                              NodeStatus status)
// { 
//     behavior_tree_msgs::msg::BTStatus msg;
//     publisher_->publish(msg);
//     // using namespace std::chrono;

//     // constexpr const char* whitespaces = "                         ";
//     // constexpr const size_t ws_count = 25;

//     // double since_epoch = duration<double>(timestamp).count();
//     // printf("[%.3f]: %s%s %s -> %s",
//     //        since_epoch, node.name().c_str(),
//     //        &whitespaces[std::min(ws_count, node.name().size())],
//     //        toStr(prev_status, true).c_str(),
//     //        toStr(status, true).c_str() );
//     // std::cout << std::endl;
// }

}   // end namespace

// #endif  // BEHAVIOR_TREE_BT_STATUS_PUB_HPP_