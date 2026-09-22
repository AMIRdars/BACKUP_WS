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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__IS_ROBOT_CLOSE_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__IS_ROBOT_CLOSE_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"


namespace ros2_behavior_tree
{

class IsRobotCloseNode : public BT::ConditionNode
{
public:
  IsRobotCloseNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    ros2_node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = BT::TreeNode::config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "source_frame", std::string("map"), "source_frame"),
      BT::InputPort<std::string>(
        "target_frame", std::string("base_link"), "target_frame"),


      BT::InputPort<geometry_msgs::msg::PoseStamped>("obj_pose", ""),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("location_pose", ""),
      BT::InputPort<double>("outer_radius", "threshold"),
      BT::InputPort<double>("inner_radius", "threshold"),
      BT::InputPort<std::string>("obj_id", ""), //xmlに余計なものがあるとエラーが出るので
      BT::InputPort<std::string>("location_id", ""),
      BT::InputPort<bool>("position_only", ""), //  位置条件だけで姿勢は考えない（未実装）
      BT::OutputPort<double>("distance", "distance"),
    };
  }

  BT::NodeStatus tick() override
  {

    std::string source_frame, target_frame;
    getInput<std::string>("source_frame", source_frame);
    getInput<std::string>("target_frame", target_frame);

    geometry_msgs::msg::PoseStamped location_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("location_pose", location_pose)) {
      if (!getInput<geometry_msgs::msg::PoseStamped>("obj_pose", location_pose))
      {
        throw BT::RuntimeError("Missing parameter [location_pose or obj_pose] in IsRobotClose node");
      }
    }

    double outer_radius;
    if (!getInput<double>("outer_radius", outer_radius)) {
      throw BT::RuntimeError("Missing parameter [outer_radius] in IsRobotClose node");
    }

    double inner_radius;
    if (!getInput<double>("inner_radius", inner_radius)) {
      inner_radius = 0;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (transform_pose(current_pose, tf_buffer_, source_frame, target_frame)) {
      // if (!setOutput<geometry_msgs::msg::PoseStamped>("pose", current_pose)) {
      //   throw BT::RuntimeError("Failed to set output port value [pose] for IsRobotClose");
      // }
      
    }




    double distance 
    = sqrt(pow(current_pose.pose.position.x - location_pose.pose.position.x,2) 
        + pow(current_pose.pose.position.y - location_pose.pose.position.y, 2));

    setOutput<double>("distance", distance);


    if(distance >= inner_radius && distance <= outer_radius)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }

  }

  private:
    bool transform_pose(
      geometry_msgs::msg::PoseStamped & target_pose,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      const std::string source_frame,
      const std::string target_frame,
      const double transform_timeout = 0.1)
    {
      rclcpp::Logger logger = rclcpp::get_logger("transform_pose");

      // Initialize the robot pose
      geometry_msgs::msg::PoseStamped robot_pose;
      tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
      robot_pose.header.frame_id = source_frame;
      robot_pose.header.stamp = rclcpp::Time();

      try {
        // Get the pose in the target frame
        tf2::toMsg(tf2::Transform::getIdentity(), target_pose.pose);
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped = tf_buffer_->lookupTransform(source_frame, target_frame,
            tf2::TimePointZero);
        target_pose.pose.position.x = transformStamped.transform.translation.x;
        target_pose.pose.position.y = transformStamped.transform.translation.y;
        target_pose.pose.position.z = transformStamped.transform.translation.z;
        target_pose.pose.orientation = transformStamped.transform.rotation;

        // RCLCPP_INFO(logger, "x,y: %f,%f", target_pose.pose.position.x, target_pose.pose.position.y);
        return true;
      } catch (tf2::LookupException & ex) {
        RCLCPP_WARN(logger,
          "Failed to get robot pose: %s\n", ex.what());
      } catch (tf2::ConnectivityException & ex) {
        RCLCPP_WARN(logger,
          "Failed to get robot pose: %s\n", ex.what());
      } catch (tf2::ExtrapolationException & ex) {
        RCLCPP_WARN(logger,
          "Failed to get robot pose: %s\n", ex.what());
      } catch (tf2::TimeoutException & ex) {
        RCLCPP_WARN(logger,
          "Transform timeout with tolerance: %.4f", transform_timeout);
      } catch (...) {
        RCLCPP_WARN(logger, "Failed to transform from %s to %s",
          source_frame.c_str(), target_frame.c_str());
      }

      return false;
    }

  
    rclcpp::Node::SharedPtr ros2_node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__IS_ROBOT_CLOSE_NODE_HPP_
