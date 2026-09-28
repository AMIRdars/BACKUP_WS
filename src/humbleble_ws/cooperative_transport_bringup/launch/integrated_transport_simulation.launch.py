"""Grasp, move forward, turn in place, strafe right, then pivot around A."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory(
        'cooperative_transport_bringup')
    rotation_share = get_package_share_directory('cooperative_rotation')
    friction_launch = os.path.join(
        bringup_share, 'launch', 'friction_simulation.launch.py')
    rotation_parameters = os.path.join(
        rotation_share, 'config', 'rotation_params.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(friction_launch),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'slip_monitor_rate': LaunchConfiguration('slip_monitor_rate'),
            'aggregate_contacts': LaunchConfiguration('aggregate_contacts'),
            'filtered_poses': LaunchConfiguration('filtered_poses'),
            'finger_collision_boxes': LaunchConfiguration('finger_collision_boxes'),
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
            'allow_external_command_handoff': 'true',
        }.items(),
    )
    rotation_controller = Node(
        package='cooperative_rotation',
        executable='rotation_controller',
        output='screen',
        parameters=[rotation_parameters, {
            'use_sim_time': True,
            'pre_rotation_translation_distance_m': ParameterValue(
                LaunchConfiguration('forward_distance_m'), value_type=float),
            'target_angle_deg': ParameterValue(
                LaunchConfiguration('rotation_angle_deg'), value_type=float),
            'post_rotation_translation_y_distance_m': 0.0,
            'post_rotation_translation_right_distance_m': ParameterValue(
                LaunchConfiguration('right_distance_m'), value_type=float),
            'translation_velocity_m_s': ParameterValue(
                LaunchConfiguration('translation_velocity_m_s'),
                value_type=float),
            'translation_acceleration_m_s2': ParameterValue(
                LaunchConfiguration('translation_acceleration_m_s2'),
                value_type=float),
            'angular_velocity_deg_s': ParameterValue(
                LaunchConfiguration('angular_velocity_deg_s'),
                value_type=float),
            'angular_acceleration_deg_s2': ParameterValue(
                LaunchConfiguration('angular_acceleration_deg_s2'),
                value_type=float),
            'pre_rotation_target_normal_force': 20.0,
            'auto_start': True,
            'require_grasp': True,
            'require_support_removed': True,
            'pre_rotation_force_stability_duration': 0.0,
            'tracking_pause_orientation_error_deg': 5.0,
            'tracking_resume_orientation_error_deg': 2.0,
            'settling_timeout': 20.0,
            'pose_timeout': 1.0,
        }],
    )
    pivot_controller = Node(
        package='cooperative_transport_control',
        executable='pivot_rotation_controller',
        output='screen',
        parameters=[{
            'target_angle_deg': ParameterValue(
                LaunchConfiguration('pivot_angle_deg'), value_type=float),
            'angular_velocity_deg_s': ParameterValue(
                LaunchConfiguration('angular_velocity_deg_s'),
                value_type=float),
            'angular_acceleration_deg_s2': ParameterValue(
                LaunchConfiguration('angular_acceleration_deg_s2'),
                value_type=float),
            'wait_for_rotation_complete': True,
            'take_over_coordinator': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('aggregate_contacts', default_value='true'),
        DeclareLaunchArgument('filtered_poses', default_value='true'),
        DeclareLaunchArgument('finger_collision_boxes', default_value='9'),
        DeclareLaunchArgument('slip_monitor_rate', default_value='50.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('forward_distance_m', default_value='0.5'),
        DeclareLaunchArgument('rotation_angle_deg', default_value='30.0'),
        DeclareLaunchArgument('right_distance_m', default_value='0.3'),
        DeclareLaunchArgument('pivot_angle_deg', default_value='30.0'),
        DeclareLaunchArgument(
            'translation_velocity_m_s', default_value='0.05'),
        DeclareLaunchArgument(
            'translation_acceleration_m_s2', default_value='0.025'),
        DeclareLaunchArgument(
            'angular_velocity_deg_s', default_value='1.0'),
        DeclareLaunchArgument(
            'angular_acceleration_deg_s2', default_value='0.25'),
        simulation,
        TimerAction(period=21.5, actions=[rotation_controller]),
        TimerAction(period=30.0, actions=[pivot_controller]),
    ])
