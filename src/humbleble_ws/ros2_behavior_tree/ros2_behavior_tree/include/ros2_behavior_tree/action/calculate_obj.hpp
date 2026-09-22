#ifndef ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_OBJ_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_OBJ_HPP_

#include <string>
#include <memory>
#include <math.h>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/float64.hpp"

namespace ros2_behavior_tree
{

class CalculateObj : public BT::SyncActionNode
{
public:
  CalculateObj(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    obj_topic_("/amir1/object_position")
    // obj_topic_("/amir1/score_position2")
  {
    node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    b_status_ = BT::NodeStatus::SUCCESS;
    // test_pub_ = node_->create_publisher<std_msgs::msg::Float64>("btl_posi",1);
  }

  static BT::PortsList providedPorts()
  {
    return {
 
      BT::InputPort<std::string>("obj_topic_",std::string("/amir1/object_position")),
      // BT::InputPort<std::string>("obj_topic_",std::string("/amir1/score_position2")),
      // BT::OutputPort<double>("btl_x", ""),
      // BT::OutputPort<double>("btl_y", ""
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_obj", "")
    };
  }

  BT::NodeStatus tick() override
  {

    while (!rclcpp::wait_for_message(obj_posi_,node_,"/amir1/object_position",server_timeout_))
    {
      RCLCPP_DEBUG(node_ ->get_logger(),"timeout /btl_poses");
    }

    // while (!rclcpp::wait_for_message(obj_posi_,node_,"/amir1/score_position2",server_timeout_))
    // {
    //   RCLCPP_DEBUG(node_ ->get_logger(),"timeout /btl_poses");
    // }



    geometry_msgs::msg::PoseStamped pose_obj;
    pose_obj.header.frame_id = "map";
    // pose_obj.pose = obj_posi_.pose;
    pose_obj.pose.position.x = obj_posi_.pose.position.x - 0.08;
    pose_obj.pose.position.y = obj_posi_.pose.position.y + 0.03;

    // if (obj_posi_.pose.position.x > 0){
    //     pose_obj.pose.position.x = obj_posi_.pose.position.x + 0.03;
    //   }
    // else{
    //     pose_obj.pose.position.x = obj_posi_.pose.position.x - 0.03;
    //   }

    // if (obj_posi_.pose.position.y > 0){
    //     pose_obj.pose.position.y = obj_posi_.pose.position.y + 0.03;
    //   }
    // else{
    //     pose_obj.pose.position.y = obj_posi_.pose.position.y - 0.03;
    //   }

        pose_obj.pose.position.z = obj_posi_.pose.position.z + 0.03;
    
    setOutput<geometry_msgs::msg::PoseStamped>("pose_obj",pose_obj);

    return BT::NodeStatus::SUCCESS;
  }

private:
rclcpp::Node::SharedPtr node_;
BT::NodeStatus b_status_;
std::chrono::milliseconds server_timeout_;

geometry_msgs::msg::PoseStamped obj_posi_;
std::string obj_topic_;

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub_;


};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__CALCULATE_OBJ_HPP_
