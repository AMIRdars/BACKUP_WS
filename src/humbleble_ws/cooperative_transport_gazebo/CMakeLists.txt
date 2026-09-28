cmake_minimum_required(VERSION 3.8)
project(cooperative_transport_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_library(cooperative_friction_grip-system SHARED
  src/friction_grip_system.cpp)
target_compile_features(cooperative_friction_grip-system PUBLIC cxx_std_17)
target_link_libraries(cooperative_friction_grip-system
  ignition-gazebo6::core)

install(TARGETS cooperative_friction_grip-system
  LIBRARY DESTINATION lib)

add_executable(transport_pose_filter src/transport_pose_filter.cpp)
target_compile_features(transport_pose_filter PUBLIC cxx_std_17)
ament_target_dependencies(transport_pose_filter rclcpp tf2_msgs)
install(TARGETS transport_pose_filter DESTINATION lib/${PROJECT_NAME})

add_executable(finger_contact_aggregator src/finger_contact_aggregator.cpp)
target_compile_features(finger_contact_aggregator PUBLIC cxx_std_17)
ament_target_dependencies(finger_contact_aggregator rclcpp std_msgs ros_gz_interfaces)
target_link_libraries(finger_contact_aggregator ignition-transport11::core ignition-msgs8::core)
install(TARGETS finger_contact_aggregator DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY worlds config DESTINATION share/${PROJECT_NAME})

ament_package()
