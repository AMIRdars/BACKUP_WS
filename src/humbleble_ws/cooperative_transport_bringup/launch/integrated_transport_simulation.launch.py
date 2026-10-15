"""Grasp, move forward, turn, move right or along world Y, then optionally pivot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
            'event_startup': LaunchConfiguration('event_startup'),
            'slip_monitor_rate': LaunchConfiguration('slip_monitor_rate'),
            'aggregate_contacts': LaunchConfiguration('aggregate_contacts'),
            'contact_input_transport': LaunchConfiguration('contact_input_transport'),
            'filtered_poses': LaunchConfiguration('filtered_poses'),
            'finger_collision_boxes': LaunchConfiguration('finger_collision_boxes'),
            'auto_grasp': 'true',
            'auto_grasp_delay': '0.0',
            'contact_only': 'true',
            'holding_force_tolerance': LaunchConfiguration('holding_force_tolerance'),
            'holding_force_release_threshold': LaunchConfiguration('holding_force_release_threshold'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_linear_acceleration': LaunchConfiguration(
                'translation_acceleration_m_s2'),
            'enable_translation_feedforward': LaunchConfiguration(
                'enable_translation_feedforward'),
            'enable_rotation_feedforward': LaunchConfiguration(
                'enable_rotation_feedforward'),
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
            'post_rotation_translation_y_distance_m': ParameterValue(
                LaunchConfiguration('world_y_distance_m'), value_type=float),
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
            'pre_rotation_force_stability_duration': 2.0,
            'pre_rotation_target_tolerance': 3.0,
            'tracking_pause_orientation_error_deg': 5.0,
            'tracking_resume_orientation_error_deg': 2.0,
            'settling_timeout': 20.0,
            'pose_timeout': 1.0,
        }],
    )
    data_recorder = Node(
        package='cooperative_rotation',
        executable='rotation_data_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_data')),
        parameters=[{
            'use_sim_time': True,
            'target_angle_deg': ParameterValue(
                LaunchConfiguration('rotation_angle_deg'), value_type=float),
            'sample_rate': ParameterValue(
                LaunchConfiguration('recording_rate'), value_type=float),
            'output_directory': LaunchConfiguration('excel_output_directory'),
        }],
    )
    pivot_controller = Node(
        package='cooperative_transport_control',
        executable='pivot_rotation_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_pivot')),
        parameters=[{
            'target_angle_deg': ParameterValue(
                LaunchConfiguration('pivot_angle_deg'), value_type=float),
            'angular_velocity_deg_s': ParameterValue(
                LaunchConfiguration('angular_velocity_deg_s'),
                value_type=float),
            'angular_acceleration_deg_s2': ParameterValue(
                LaunchConfiguration('angular_acceleration_deg_s2'),
                value_type=float),
            'use_sim_time': True,
            'wait_for_rotation_complete': True,
            'take_over_coordinator': True,
        }],
    )

    rotation_controller_if_ready = GroupAction([rotation_controller], condition=IfCondition(LaunchConfiguration('event_startup')))
    pivot_controller_if_ready = GroupAction([pivot_controller], condition=IfCondition(LaunchConfiguration('event_startup')))

    return LaunchDescription([
        DeclareLaunchArgument('event_startup', default_value='true'),
        DeclareLaunchArgument('aggregate_contacts', default_value='true'),
        DeclareLaunchArgument('contact_input_transport', default_value='ros', choices=['ros', 'gazebo']),
        DeclareLaunchArgument('filtered_poses', default_value='true'),
        DeclareLaunchArgument('finger_collision_boxes', default_value='9'),
        DeclareLaunchArgument('slip_monitor_rate', default_value='50.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('holding_force_tolerance', default_value='0.5'),
        DeclareLaunchArgument('holding_force_release_threshold', default_value='23.0'),
        DeclareLaunchArgument('forward_distance_m', default_value='0.5'),
        DeclareLaunchArgument('rotation_angle_deg', default_value='30.0'),
        DeclareLaunchArgument('right_distance_m', default_value='0.3'),
        DeclareLaunchArgument(
            'world_y_distance_m', default_value='0.0',
            description='Signed world-Y distance after rotation; set right_distance_m=0 to use it.'),
        DeclareLaunchArgument('pivot_angle_deg', default_value='30.0'),
        DeclareLaunchArgument(
            'enable_pivot', default_value='true',
            description='Run the final pivot rotation after transport.'),
        DeclareLaunchArgument('record_data', default_value='false'),
        DeclareLaunchArgument('recording_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'excel_output_directory',
            default_value=os.path.expanduser(
                '~/ros2_humble_ws/rotation_measurements')),
        DeclareLaunchArgument(
            'translation_velocity_m_s', default_value='0.10',
            description='Cruise speed; acceleration/deceleration lower the mean speed.'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.110'),
        DeclareLaunchArgument('enable_translation_feedforward', default_value='true'),
        DeclareLaunchArgument('enable_rotation_feedforward', default_value='true'),
        DeclareLaunchArgument(
            'translation_acceleration_m_s2', default_value='0.025'),
        DeclareLaunchArgument(
            'angular_velocity_deg_s', default_value='1.0'),
        DeclareLaunchArgument(
            'angular_acceleration_deg_s2', default_value='0.25'),
        simulation,
        data_recorder,
        TimerAction(period=21.5, actions=[rotation_controller], condition=UnlessCondition(LaunchConfiguration('event_startup'))),
        TimerAction(period=30.0, actions=[pivot_controller], condition=UnlessCondition(LaunchConfiguration('event_startup'))),
        rotation_controller_if_ready,
        pivot_controller_if_ready,
    ])
