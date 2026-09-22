cmake_minimum_required(VERSION 3.8)
project(mecanumrover_description)

find_package(ament_cmake REQUIRED)

message(STATUS ">>> Installing rover_twist_relay.py into lib/${PROJECT_NAME}")

# Python nodes
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rover_twist_relay.py
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rover_twist_relay_ign.py
  DESTINATION lib/${PROJECT_NAME}
)

# YAML for rover_twist_relay (explicit install)
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rover_twist_relay.yaml
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Resource directories
install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

