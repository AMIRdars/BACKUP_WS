#include <ros2_behavior_tree/bt_status_pub.hpp>


namespace BT
{

StatusPub::StatusPub(const BT::Tree& tree) : StatusChangeLogger(tree.rootNode())
{
}

StatusPub::~StatusPub(){};

void StatusPub::flush(){};



behavior_tree_msgs::msg::NodeStatus StatusPub::convert_status_msg(const TreeNode& bt_node)
{
  behavior_tree_msgs::msg::NodeStatus node_status;
  node_status = convert_status_msg(bt_node.status());

  switch (bt_node.type()) {
    case BT::NodeType::UNDEFINED:
      node_status.node_type = behavior_tree_msgs::msg::NodeStatus::UNDEFINED;
      break;
    case BT::NodeType::ACTION:
      node_status.node_type = behavior_tree_msgs::msg::NodeStatus::ACTION;
      break;

    case BT::NodeType::CONDITION:
      node_status.node_type = behavior_tree_msgs::msg::NodeStatus::CONDITION;
      break;

    case BT::NodeType::CONTROL:
      node_status.node_type = behavior_tree_msgs::msg::NodeStatus::CONTROL;
      break;

    case BT::NodeType::DECORATOR:
      node_status.node_type = behavior_tree_msgs::msg::NodeStatus::DECORATOR;
      break;

    case BT::NodeType::SUBTREE:
      node_status.node_type = behavior_tree_msgs::msg::NodeStatus::SUBTREE;
      break;

    default:
      throw std::logic_error("Invalid return value from the BT");
  }
  
  // if (bt_node.type() == BT::NodeType::SUBTREE) node_status.node_name = bt_node.getCustomName();
  node_status.node_name = bt_node.name(); //  xml内のname=""で指定した名前
  //node.registrationName() //  ノードの名前
  node_status.node_id = bt_node.UID();
  return node_status;
}

behavior_tree_msgs::msg::NodeStatus StatusPub::convert_status_msg(const BT::NodeStatus& bt_node_status)
{
  behavior_tree_msgs::msg::NodeStatus node_status;
  switch (bt_node_status) {
    case BT::NodeStatus::IDLE:
      node_status.status = behavior_tree_msgs::msg::NodeStatus::IDLE;
      break;
    case BT::NodeStatus::RUNNING:
      node_status.status = behavior_tree_msgs::msg::NodeStatus::RUNNING;
      break;

    case BT::NodeStatus::SUCCESS:
      node_status.status = behavior_tree_msgs::msg::NodeStatus::SUCCESS;
      break;

    case BT::NodeStatus::FAILURE:
      node_status.status = behavior_tree_msgs::msg::NodeStatus::FAILURE;
      break;

    default:
      throw std::logic_error("Invalid return value from the BT");
  }
  return node_status;
}

void StatusPub::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                             NodeStatus status)
{ 
    behavior_tree_msgs::msg::NodeStatus msg;
    // std::cout <<  node.name() << std::endl;
    // std::cout <<  node.registrationName() << std::endl;
    // std::cout <<  "--------------------" << std::endl;
    msg = convert_status_msg(node);
    bt_status_.bt_status.push_back(msg);
}

void StatusPub::get_bt_status(behavior_tree_msgs::msg::BTStatus& bt_status)
{
  bt_status = bt_status_;
  bt_status_.bt_status.clear();  //コピーとクリアの間で追加されたら漏れが発生すると思う
}

}   // end namespace