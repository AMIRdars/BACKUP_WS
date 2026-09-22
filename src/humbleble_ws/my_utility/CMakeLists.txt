cmake_minimum_required(VERSION 3.5)
project(my_utility)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amir_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(realsense2_camera_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rgb_pc_
 src/rgb_pc_.cpp)

add_executable(rgb_ali_
 src/rgb_ali_.cpp)

add_executable(align_pc
 src/align_pc.cpp)

add_executable(odom_tf2_broadcaster
 src/odom_tf2_broadcaster.cpp)

add_executable(camera_to_amirbase
 src/camera_to_amirbase.cpp)

add_executable(convertpc_
 src/convertpc_.cpp)

add_executable(compc_oct
 src/compc_oct.cpp)

add_executable(gz_pose_filter
 src/gz_pose_filter.cpp)

set(dependencies
  tf2_msgs
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  amir_interfaces
  std_msgs
  geometry_msgs
  rclcpp_action
  behavior_tree_msgs
  Eigen3
  sensor_msgs
  cv_bridge
  pcl_conversions
  pcl_msgs
  PCL
  OpenCV
  image_transport
  moveit_ros_planning_interface
  message_filters
  octomap
  octomap_msgs

)

ament_target_dependencies(odom_tf2_broadcaster ${dependencies})
ament_target_dependencies(camera_to_amirbase ${dependencies})
ament_target_dependencies(rgb_pc_ ${dependencies})
ament_target_dependencies(rgb_ali_ ${dependencies})
ament_target_dependencies(align_pc ${dependencies})
ament_target_dependencies(convertpc_ ${dependencies})
ament_target_dependencies(compc_oct ${dependencies})
ament_target_dependencies(gz_pose_filter ${dependencies})



install(TARGETS
    rgb_pc_
    rgb_ali_
    align_pc
    odom_tf2_broadcaster
    camera_to_amirbase
    convertpc_
    compc_oct
    gz_pose_filter
    DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
