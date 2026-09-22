cmake_minimum_required(VERSION 3.5)
project(behavior_tree_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function
#  declarations
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


set(dependencies
  builtin_interfaces
  std_msgs
  action_msgs
  geometry_msgs
  moveit_msgs
  nav2_msgs
)

set(msg_files
  "msg/NodeStatus.msg"
  "msg/BTStatus.msg"
  "msg/BBPose.msg"
  "msg/BBPath.msg"
  "msg/BBFloat64Array.msg"
  "msg/BBMessage.msg" 
  "msg/Pixel.msg"
  "msg/Object.msg"
  "msg/ObjectArray.msg"
  "msg/Vector13.msg"
  "msg/Data.msg"

)

set(action_files
  "action/SendGoal.action"
  "action/ExecuteTree.action"
  "action/PrintMessage.action"
  
  "action/GenApproach.action"
  "action/Approach.action"
  
  "action/Place.action"
  "action/Search.action"
  "action/CheckIK.action"
  "action/CameraSearch.action"
  
  # amir
  "action/Pick.action"
  "action/SearchObj.action"
  "action/MoveArm.action"
  "action/Grasp.action"
  "action/MoveMeca.action"
  "action/BeforeArm.action"
  "action/AfterArm.action"
  "action/SearchAmir.action"
)

set(srv_files
  "srv/GetBT.srv"
  "srv/SetBlackBoard.srv"
  "srv/GetBlackBoard.srv"
  "srv/CreateBT.srv"

  
  # bridge srv
  "srv/FovCheck.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES ${dependencies}
)

install(
  FILES bridge_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
