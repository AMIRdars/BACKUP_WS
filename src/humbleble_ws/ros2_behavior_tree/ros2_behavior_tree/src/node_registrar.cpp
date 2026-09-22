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

#include "ros2_behavior_tree/node_registrar.hpp"

#include <string>

#include "ros2_behavior_tree/action/async_wait_node.hpp"
#include "ros2_behavior_tree/action/compute_path_to_pose_node.hpp"
#include "ros2_behavior_tree/action/create_ros2_node.hpp"
#include "ros2_behavior_tree/action/create_transform_buffer_node.hpp"
// #include "ros2_behavior_tree/action/follow_path_node.hpp"
#include "ros2_behavior_tree/action/get_poses_near_robot_node.hpp"
#include "ros2_behavior_tree/action/pure_pursuit_node.hpp"
#include "ros2_behavior_tree/action/transform_pose_node.hpp"
#include "ros2_behavior_tree/condition/can_transform_node.hpp"
#include "ros2_behavior_tree/control/first_result_node.hpp"
#include "ros2_behavior_tree/control/pipeline_sequence_node.hpp"
#include "ros2_behavior_tree/control/recovery_node.hpp"
#include "ros2_behavior_tree/control/round_robin_node.hpp"
#include "ros2_behavior_tree/decorator/distance_constraint_node.hpp"
#include "ros2_behavior_tree/decorator/for_each_pose_node.hpp"
#include "ros2_behavior_tree/decorator/forever_node.hpp"
#include "ros2_behavior_tree/decorator/repeat_until_node.hpp"
#include "ros2_behavior_tree/decorator/throttle_tick_rate_node.hpp"
#include "ros2_behavior_tree/action/calculate_goal.hpp"
#include "ros2_behavior_tree/action/change_node_status.hpp"
#include "ros2_behavior_tree/action/set_param.hpp"
#include "ros2_behavior_tree/action/search_obj_node.hpp"
#include "ros2_behavior_tree/condition/close_to_node.hpp"
#include "ros2_behavior_tree/condition/is_robot_close_node.hpp"
#include "ros2_behavior_tree/condition/is_grasped_obj_node.hpp"
#include "ros2_behavior_tree/condition/is_hand_free_node.hpp"
// #include "ros2_behavior_tree/action/place_node.hpp"
// #include "ros2_behavior_tree/action/pick_node.hpp"
#include "ros2_behavior_tree/action/marker_publisher_node.hpp"
#include "ros2_behavior_tree/control/priority_fallback.hpp"
#include "ros2_behavior_tree/action/create_bt_gen_node.hpp"
#include "ros2_behavior_tree/action/approach_node.hpp"
#include "ros2_behavior_tree/decorator/once_node.hpp"
#include "ros2_behavior_tree/action/camera_search.hpp"
#include "ros2_behavior_tree/condition/check_help.hpp"
#include "ros2_behavior_tree/decorator/utility_choice.hpp"

#include "ros2_behavior_tree/condition/task_clear.hpp"
#include "ros2_behavior_tree/condition/check_odom.hpp"

#include "ros2_behavior_tree/action/pick_amir.hpp"
#include "ros2_behavior_tree/action/move_meca.hpp"
#include "ros2_behavior_tree/action/receive_meca_goal.hpp"
#include "ros2_behavior_tree/action/set_meca_goal.hpp"
#include "ros2_behavior_tree/action/place_amir.hpp"
// #include "ros2_behavior_tree/condition/after_pick.hpp"
#include "ros2_behavior_tree/condition/check_grasp.hpp"
#include "ros2_behavior_tree/action/calculate_btl_posi.hpp"
#include "ros2_behavior_tree/action/calculate_obj.hpp"
#include "ros2_behavior_tree/action/search_btl.hpp"
#include "ros2_behavior_tree/condition/check_vel.hpp"
#include "ros2_behavior_tree/action/calculate_place_posi.hpp"
#include "ros2_behavior_tree/action/search_amir.hpp"

///////////////////

BT_REGISTER_NODES(factory)
{
  ros2_behavior_tree::NodeRegistrar::RegisterNodes(factory);
}

namespace ros2_behavior_tree
{

void
NodeRegistrar::RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  const BT::PortsList message_params {BT::InputPort<std::string>("msg")};
  factory.registerSimpleAction("Message",
    std::bind(&NodeRegistrar::message, std::placeholders::_1), message_params);

  factory.registerNodeType<ros2_behavior_tree::AsyncWaitNode>("AsyncWait");
  factory.registerNodeType<ros2_behavior_tree::CanTransformNode>("CanTransform");
  // factory.registerNodeType<ros2_behavior_tree::ComputePathToPoseNode>("ComputePathToPose");
  factory.registerNodeType<ros2_behavior_tree::CreateROS2Node>("CreateROS2Node");
  factory.registerNodeType<ros2_behavior_tree::CreateTransformBufferNode>("CreateTransformBuffer");
  factory.registerNodeType<ros2_behavior_tree::DistanceConstraintNode>("DistanceConstraint");
  factory.registerNodeType<ros2_behavior_tree::FirstResultNode>("FirstResult");
  // factory.registerNodeType<ros2_behavior_tree::FollowPathNode>("FollowPath");
  factory.registerNodeType<ros2_behavior_tree::ForeverNode>("Forever");
  factory.registerNodeType<ros2_behavior_tree::ForEachPoseNode>("ForEachPose");
  factory.registerNodeType<ros2_behavior_tree::GetPosesNearRobotNode>("GetPosesNearRobot");
  // factory.registerNodeType<ros2_behavior_tree::PipelineSequenceNode>("PipelineSequence");
  factory.registerNodeType<ros2_behavior_tree::PurePursuitController>("PurePursuit"); // TODO: name
  factory.registerNodeType<ros2_behavior_tree::RecoveryNode>("Recovery");
  factory.registerNodeType<ros2_behavior_tree::RepeatUntilNode>("RepeatUntil");
  // factory.registerNodeType<ros2_behavior_tree::RoundRobinNode>("RoundRobin");
  factory.registerNodeType<ros2_behavior_tree::ThrottleTickRateNode>("ThrottleTickRate");
  factory.registerNodeType<ros2_behavior_tree::TransformPoseNode>("TransformPose");
  factory.registerNodeType<ros2_behavior_tree::CalculateGoal>("CalculateGoal");
  factory.registerNodeType<ros2_behavior_tree::ChangeNodeStatus>("ChangeNodeStatus");
  factory.registerNodeType<ros2_behavior_tree::SetParam>("SetParam");
  factory.registerNodeType<ros2_behavior_tree::SearchObjNode>("SearchObj");
  factory.registerNodeType<ros2_behavior_tree::CloseToNode>("IsObjectAt");
  factory.registerNodeType<ros2_behavior_tree::IsRobotCloseNode>("IsRobotCloseTo");
  factory.registerNodeType<ros2_behavior_tree::IsGraspedObjNode>("IsGraspedObj");
  factory.registerNodeType<ros2_behavior_tree::IsHandFreeNode>("IsHandFree");
  // factory.registerNodeType<ros2_behavior_tree::PlaceNode>("Place");
  // factory.registerNodeType<ros2_behavior_tree::PickNode>("Pick");
  factory.registerNodeType<ros2_behavior_tree::MarkerPublisherNode>("MarkerPublish");
  factory.registerNodeType<ros2_behavior_tree::CreateBTGenNode>("CreateBTGen");
  factory.registerNodeType<ros2_behavior_tree::ApproachNode>("Approach");
  factory.registerNodeType<ros2_behavior_tree::PriorityFallbackNode>("PriorityFallback");
  factory.registerNodeType<ros2_behavior_tree::OnceNode>("Once");
  factory.registerNodeType<ros2_behavior_tree::CameraSearchNode>("CameraSearch");
  factory.registerNodeType<ros2_behavior_tree::CheckHelpNode>("Checkhelp");
  factory.registerNodeType<ros2_behavior_tree::UtilityChoice>("UtilityChoice");
  factory.registerNodeType<ros2_behavior_tree::TaskClearNode>("TaskClear");

  factory.registerNodeType<ros2_behavior_tree::CheckOdomNode>("CheckOdom");
  factory.registerNodeType<ros2_behavior_tree::PickAmirNode>("PickAmir");
  factory.registerNodeType<ros2_behavior_tree::MoveMecaNode>("MoveMeca");
  factory.registerNodeType<ros2_behavior_tree::ReceiveMecaGoalNode>("ReceiveMecaGoal");
  factory.registerNodeType<ros2_behavior_tree::SetMecaGoalNode>("SetMecaGoal");
  factory.registerNodeType<ros2_behavior_tree::PlaceAmirNode>("PlaceAmir");
  // factory.registerNodeType<ros2_behavior_tree::AfterPickNode>("AfterPick");
  factory.registerNodeType<ros2_behavior_tree::CheckGraspNode>("CheckGrasp");
  // factory.registerNodeType<ros2_behavior_tree::CalculateReceiveGoal>("CalculateReceiveGoal");
  // factory.registerNodeType<ros2_behavior_tree::ReceiveMoveMecaNode>("ReceiveMoveMeca");
  // factory.registerNodeType<ros2_behavior_tree::MoveMecaNode>("MoveMeca");
  factory.registerNodeType<ros2_behavior_tree::CalculateBtl>("CalculateBtl"); 
  factory.registerNodeType<ros2_behavior_tree::SearchBtl>("SearchBtl"); 
  factory.registerNodeType<ros2_behavior_tree::CheckVelNode>("CheckVel");
  factory.registerNodeType<ros2_behavior_tree::CalculateObj>("CalculateObj"); 
  factory.registerNodeType<ros2_behavior_tree::CalculatePlacePosi>("CalculatePlacePosi");
  factory.registerNodeType<ros2_behavior_tree::SearchAmirNode>("SearchAmir"); 

  
  /////////////

}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

BT::NodeStatus
NodeRegistrar::message(BT::TreeNode & tree_node)
{
  std::string msg;
  tree_node.getInput<std::string>("msg", msg);

  printf(ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET "\n", msg.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ros2_behavior_tree