// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_BEHAVIOR_TREE__ACTION__MARKER_PUBLISHER_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__MARKER_PUBLISHER_NODE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <cmath>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"


namespace ros2_behavior_tree
{

class MarkerPublisherNode : public BT::SyncActionNode
{
public:
  MarkerPublisherNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("bt_pose_marker", 1);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose",
        "The current pose of the robot"),
    };
  }

  BT::NodeStatus tick() override
  {


    geometry_msgs::msg::PoseStamped pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("pose", pose)) {
      throw BT::RuntimeError("Missing parameter [pose] in GetPosesNearRobot node");
    }
    // RCLCPP_INFO(node_->get_logger(),"(x,y) = (%lf, %lf)",pose.pose.position.x, pose.pose.position.y);
    // RCLCPP_INFO(node_->get_logger(),"w = %lf",pose.pose.orientation.w);

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.push_back(pose);
    publishPosesMarker(poses);

    return BT::NodeStatus::SUCCESS;
  }


  void publishPosesMarker(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    visualization_msgs::msg::Marker marker;

    builtin_interfaces::msg::Time time;
    time.sec = 0;
    time.nanosec = 0;
    marker.header.stamp = time;
    marker.header.frame_id = "map";

    // Set the namespace and id for this marker. This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "follower_goal";
    marker.id = 1;

    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // 0 indicates the object should last forever
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    marker.frame_locked = false;

    marker.points.resize(poses.size());
    marker.colors.resize(poses.size());

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.a = 0.5;

    for (const auto & p : poses) {
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = p.pose.position.x;
      pose.position.y = p.pose.position.y;
      marker.points.push_back(pose.position);
      marker.colors.push_back(color);
      color.r = color.r >= 0.25 ? color.r - 0.25 : 1.0;
    }

    marker_pub_->publish(marker);
  }

protected:
  bool initialized_{false};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__MARKER_PUBLISHER_NODE_HPP_
