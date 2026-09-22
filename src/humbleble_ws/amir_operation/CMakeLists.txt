cmake_minimum_required(VERSION 3.8)
project(amir_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(jointfix
  src/jointfix_pick_place.cpp
)

add_executable(jointfix10
  src/jointfix_10_pick_place.cpp
)

add_executable(pick_server
  src/pick_server.cpp
)

add_executable(initial_posi_gz
  src/initial_posi_gz.cpp
)

add_executable(move_meca_server
  src/move_meca_server.cpp
)

add_executable(place_server
  src/place_server.cpp
)

#add_executable(pick_place_humble_10
#  src/pick_place_humble_10.cpp
#)

add_executable(jointfix_cm
  src/jointfix_cm_pick_place.cpp
)

# VR/MR テレオペ (MoveIt Servo) 用ノード
add_executable(servo_test_twist
  src/servo_test_twist.cpp
)

add_executable(vr_twist_bridge
  src/vr_twist_bridge.cpp
)

add_executable(servo_keyboard_twist
  src/servo_keyboard_twist.cpp
)

add_executable(servo_keyboard_jog
  src/servo_keyboard_jog.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  behavior_tree_msgs
)

set(dependencies_10
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  shape_msgs
)

set(dependencies_cm
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  shape_msgs
)

ament_target_dependencies(jointfix ${dependencies})
ament_target_dependencies(jointfix10 ${dependencies})
ament_target_dependencies(pick_server ${dependencies})
ament_target_dependencies(initial_posi_gz ${dependencies})
ament_target_dependencies(move_meca_server ${dependencies})
ament_target_dependencies(place_server ${dependencies})
#ament_target_dependencies(pick_place_humble_10 ${dependencies_10})
#target_include_directories(pick_place_humble_10 PUBLIC ${EIGEN3_INCLUDE_DIRS})
#target_link_libraries(pick_place_humble_10 Eigen3::Eigen)
ament_target_dependencies(jointfix_cm ${dependencies_cm})
target_include_directories(jointfix_cm PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(jointfix_cm Eigen3::Eigen)

set(dependencies_servo
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(servo_test_twist ${dependencies_servo})
ament_target_dependencies(vr_twist_bridge ${dependencies_servo})
ament_target_dependencies(servo_keyboard_twist ${dependencies_servo})

ament_target_dependencies(servo_keyboard_jog
  rclcpp
  sensor_msgs
  control_msgs
  std_srvs
  moveit_core
  srdfdom
  urdf
  urdfdom
  ament_index_cpp
)
target_include_directories(servo_keyboard_jog PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(servo_keyboard_jog Eigen3::Eigen)

install(TARGETS
  jointfix
  jointfix10
  pick_server
  initial_posi_gz
  move_meca_server
  place_server
  #pick_place_humble_10
  jointfix_cm
  servo_test_twist
  vr_twist_bridge
  servo_keyboard_twist
  servo_keyboard_jog
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
