"""Grasp the payload, then translate it to the payload's right only."""

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
    rotation_share = get_package_share_directory('cooperative_rotation')
    friction_launch = os.path.join(
        bringup_share, 'launch', 'friction_simulation.launch.py')
    rotation_parameters = os.path.join(
        rotation_share, 'config', 'rotation_params.yaml')

    headless = LaunchConfiguration('headless')
    distance = LaunchConfiguration('right_distance_m')
    velocity = LaunchConfiguration('translation_velocity_m_s')
    acceleration = LaunchConfiguration('translation_acceleration_m_s2')
    auto_start = LaunchConfiguration('auto_start')
    record_data = LaunchConfiguration('record_data')
    recording_rate = LaunchConfiguration('recording_rate')
    output_directory = LaunchConfiguration('excel_output_directory')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(friction_launch),
        launch_arguments={
            'headless': headless,
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

    lateral_controller = Node(
        package='cooperative_rotation',
        executable='rotation_controller',
        name='lateral_translation_controller',
        output='screen',
        parameters=[
            rotation_parameters,
            {
                'use_sim_time': True,
                # No pre-translation and no rotation in this trajectory.
                'pre_rotation_translation_distance_m': 0.0,
                'target_angle_deg': 0.0,
                'post_rotation_translation_y_distance_m': 0.0,
                'post_rotation_translation_right_distance_m': ParameterValue(
                    distance, value_type=float),
                'translation_velocity_m_s': ParameterValue(
                    velocity, value_type=float),
                'translation_acceleration_m_s2': ParameterValue(
                    acceleration, value_type=float),
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'require_grasp': True,
                'require_support_removed': True,
                'pre_rotation_force_stability_duration': 0.0,
                'tracking_pause_orientation_error_deg': 5.0,
                'tracking_resume_orientation_error_deg': 2.0,
                'settling_timeout': 20.0,
                'pose_timeout': 1.0,
            },
        ],
    )

    data_recorder = Node(
        package='cooperative_rotation',
        executable='rotation_data_recorder',
        name='lateral_translation_data_recorder',
        output='screen',
        condition=IfCondition(record_data),
        parameters=[{
            'use_sim_time': True,
            'target_angle_deg': 0.0,
            'sample_rate': ParameterValue(recording_rate, value_type=float),
            'output_directory': output_directory,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('right_distance_m', default_value='0.5'),
        DeclareLaunchArgument(
            'translation_velocity_m_s', default_value='0.05'),
        DeclareLaunchArgument(
            'translation_acceleration_m_s2', default_value='0.025'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('record_data', default_value='true'),
        DeclareLaunchArgument('recording_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'excel_output_directory',
            default_value=os.path.expanduser(
                '~/ros2_humble_ws/rotation_measurements')),
        simulation,
        data_recorder,
        # friction_simulation starts the grasp/support nodes at t=29 s.
        TimerAction(period=31.0, actions=[lateral_controller]),
    ])
