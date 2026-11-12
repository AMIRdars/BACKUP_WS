"""Grasp a payload and rotate it around AMIR A (amir1)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            'target_normal_force': LaunchConfiguration('target_normal_force'),
            'holding_force_tolerance': LaunchConfiguration('holding_force_tolerance'),
            'holding_force_release_threshold': LaunchConfiguration('holding_force_release_threshold'),
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
            'use_sim_time': True,
            'target_angle_deg': ParameterValue(
                LaunchConfiguration('target_angle_deg'), value_type=float),
            'angular_velocity_deg_s': ParameterValue(
                LaunchConfiguration('angular_velocity_deg_s'),
                value_type=float),
            'angular_acceleration_deg_s2': ParameterValue(
                LaunchConfiguration('angular_acceleration_deg_s2'),
                value_type=float),
        }],
    )
    data_recorder = Node(
        package='cooperative_rotation',
        executable='rotation_data_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_data')),
        parameters=[{
            'use_sim_time': True,
            'pivot_mode': True,
            'target_angle_deg': ParameterValue(
                LaunchConfiguration('target_angle_deg'), value_type=float),
            'sample_rate': ParameterValue(
                LaunchConfiguration('recording_rate'), value_type=float),
            'output_directory': LaunchConfiguration('excel_output_directory'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('target_normal_force', default_value='20.0'),
        DeclareLaunchArgument('holding_force_tolerance', default_value='0.5'),
        DeclareLaunchArgument('holding_force_release_threshold', default_value='23.0'),
        DeclareLaunchArgument('target_angle_deg', default_value='90.0'),
        DeclareLaunchArgument(
            'angular_velocity_deg_s', default_value='1.0'),
        DeclareLaunchArgument(
            'angular_acceleration_deg_s2', default_value='0.25'),
        DeclareLaunchArgument('record_data', default_value='true'),
        DeclareLaunchArgument('recording_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'excel_output_directory',
            default_value=os.path.expanduser(
                '~/ros2_humble_ws/rotation_measurements')),
        simulation,
        data_recorder,
        # The grasp/support nodes are started by friction_simulation at 29 s.
        TimerAction(period=30.0, actions=[controller]),
    ])
