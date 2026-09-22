cmake_minimum_required(VERSION 3.8)
project(cooperative_transport_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

add_library(cooperative_friction_grip-system SHARED
  src/friction_grip_system.cpp)
target_compile_features(cooperative_friction_grip-system PUBLIC cxx_std_17)
target_link_libraries(cooperative_friction_grip-system
  ignition-gazebo6::core)

install(TARGETS cooperative_friction_grip-system
  LIBRARY DESTINATION lib)

install(DIRECTORY worlds config DESTINATION share/${PROJECT_NAME})

ament_package()
