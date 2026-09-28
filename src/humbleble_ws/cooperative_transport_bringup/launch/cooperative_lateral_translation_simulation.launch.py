"""Grasp the payload with two AMIRs, then translate it right by 0.5 m."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory(
        'cooperative_transport_bringup')
    friction_launch = os.path.join(
        bringup_share, 'launch', 'friction_simulation.launch.py')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(friction_launch),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'auto_grasp': 'true',
            'auto_grasp_delay': '0.0',
            'contact_only': 'true',
            'max_linear_speed': '0.060',
            'max_linear_acceleration': '0.025',
            'yaw_tolerance': '0.00872665',
            'maximum_base_speed': '0.120',
            'odometry_timeout': '1.0',
            'enable_formation_distance_control': 'true',
            'formation_distance_kp': '0.8',
            'formation_distance_max_correction_speed': '0.012',
        }.items(),
    )
    goal_node = Node(
        package='cooperative_transport_control',
        executable='cooperative_lateral_goal',
        output='screen',
        parameters=[{
            'right_distance_m': LaunchConfiguration('right_distance_m'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('right_distance_m', default_value='0.5'),
        simulation,
        # The friction bringup starts the grasp/coordinator nodes at 29 s.
        TimerAction(period=30.0, actions=[goal_node]),
    ])
