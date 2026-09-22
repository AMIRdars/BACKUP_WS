cmake_minimum_required(VERSION 3.8)
project(coop_transport_bt)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(coop_transport_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_library(coop_transport_bt_nodes SHARED
  src/cooperative_motion_bt_node.cpp
  src/transport_safety_nodes.cpp)
target_include_directories(coop_transport_bt_nodes PUBLIC include)
ament_target_dependencies(coop_transport_bt_nodes
  behaviortree_cpp_v3 coop_transport_interfaces geometry_msgs rclcpp rclcpp_action std_msgs)

add_executable(bt_runner src/bt_runner.cpp)
target_include_directories(bt_runner PRIVATE include)
target_link_libraries(bt_runner coop_transport_bt_nodes)
ament_target_dependencies(bt_runner ament_index_cpp behaviortree_cpp_v3 rclcpp)

install(TARGETS coop_transport_bt_nodes bt_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees/ DESTINATION share/${PROJECT_NAME}/behavior_trees)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
