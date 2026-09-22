import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_behavior_tree",
            executable="set_cost_around_point",
            name="set_cost_around_point",
            parameters=[
                # {"x": 0.0},
                # {"y": 0.0},
            ],
        ),
    ])