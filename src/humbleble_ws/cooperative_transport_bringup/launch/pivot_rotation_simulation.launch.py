"""Grasp a payload and rotate it around AMIR A (amir1)."""

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
            'enable_coordinator': 'false',
            'enable_formation_distance_control': 'true',
            'formation_distance_kp': '0.8',
            'formation_distance_max_correction_speed': '0.012',
        }.items(),
    )
    controller = Node(
        package='cooperative_transport_control',
        executable='pivot_rotation_controller',
        output='screen',
        parameters=[{
            'target_angle_deg': LaunchConfiguration('target_angle_deg'),
            'angular_velocity_deg_s': LaunchConfiguration(
                'angular_velocity_deg_s'),
            'angular_acceleration_deg_s2': LaunchConfiguration(
                'angular_acceleration_deg_s2'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('target_angle_deg', default_value='90.0'),
        DeclareLaunchArgument(
            'angular_velocity_deg_s', default_value='1.0'),
        DeclareLaunchArgument(
            'angular_acceleration_deg_s2', default_value='0.25'),
        simulation,
        # The grasp/support nodes are started by friction_simulation at 29 s.
        TimerAction(period=30.0, actions=[controller]),
    ])
