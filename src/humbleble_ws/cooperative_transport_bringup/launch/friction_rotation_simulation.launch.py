"""Physically grasp the payload, then execute a specified-axis rotation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
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
    target_angle = LaunchConfiguration('target_angle_deg')
    angular_velocity = LaunchConfiguration('angular_velocity_deg_s')
    angular_acceleration = LaunchConfiguration('angular_acceleration_deg_s2')
    auto_start = LaunchConfiguration('auto_start')
    enable_wrench_safety = LaunchConfiguration('enable_wrench_safety')
    record_data = LaunchConfiguration('record_data')
    excel_output_directory = LaunchConfiguration('excel_output_directory')
    recording_rate = LaunchConfiguration('recording_rate')
    target_normal_force = LaunchConfiguration('target_normal_force')
    gripper_maximum_effort = LaunchConfiguration(
        'gripper_maximum_effort')
    translation_distance = LaunchConfiguration('translation_distance_m')
    translation_velocity = LaunchConfiguration('translation_velocity_m_s')
    translation_acceleration = LaunchConfiguration(
        'translation_acceleration_m_s2')
    post_rotation_translation_y_distance = LaunchConfiguration(
        'post_rotation_translation_y_distance_m')
    enable_formation_distance_control = LaunchConfiguration(
        'enable_formation_distance_control')
    formation_distance_kp = LaunchConfiguration('formation_distance_kp')
    formation_distance_max_correction_speed = LaunchConfiguration(
        'formation_distance_max_correction_speed')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(friction_launch),
        launch_arguments={
            'headless': headless,
            'auto_grasp': 'true',
            # Keep auto-close after the deliberately delayed payload spawn;
            # this removes a timing race with the second robot's controller.
            'auto_grasp_delay': '15.0',
            # Stage C uses Gazebo finger/payload contacts only. The custom
            # bristle-wrench helper is absent from this world.
            'contact_only': 'true',
            # At a 1.34 m base radius, 2 deg/s needs about 0.047 m/s.  The old
            # 0.030 m/s cap forced a persistent yaw lag and excessive internal
            # contact load, so retain bounded tracking margin here.
            'max_linear_speed': '0.060',
            'max_linear_acceleration': '0.025',
            # The generic friction transport accepts 2 deg yaw error. Rotation
            # completion requires 1 deg, so use 0.5 deg for this launch.
            'yaw_tolerance': '0.00872665',
            'maximum_base_speed': '0.120',
            # Compound-finger contact sensing creates substantially more
            # bridge traffic. Allow brief simulation scheduling stalls while
            # retaining a bounded odometry-loss stop.
            'odometry_timeout': '1.0',
            'enable_wrench_safety': enable_wrench_safety,
            'target_normal_force': target_normal_force,
            'gripper_maximum_effort': gripper_maximum_effort,
            'enable_formation_distance_control': (
                enable_formation_distance_control),
            'formation_distance_kp': formation_distance_kp,
            'formation_distance_max_correction_speed': (
                formation_distance_max_correction_speed),
        }.items(),
    )
    rotation_controller = Node(
        package='cooperative_rotation',
        executable='rotation_controller',
        output='screen',
        parameters=[
            rotation_parameters,
            {
                'use_sim_time': True,
                'target_angle_deg': ParameterValue(
                    target_angle, value_type=float),
                'angular_velocity_deg_s': ParameterValue(
                    angular_velocity, value_type=float),
                'angular_acceleration_deg_s2': ParameterValue(
                    angular_acceleration, value_type=float),
                'pre_rotation_target_normal_force': ParameterValue(
                    target_normal_force, value_type=float),
                'pre_rotation_translation_distance_m': ParameterValue(
                    translation_distance, value_type=float),
                'translation_velocity_m_s': ParameterValue(
                    translation_velocity, value_type=float),
                'translation_acceleration_m_s2': ParameterValue(
                    translation_acceleration, value_type=float),
                'post_rotation_translation_y_distance_m': ParameterValue(
                    post_rotation_translation_y_distance, value_type=float),
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'require_grasp': True,
                'require_support_removed': True,
                # The physical-contact plant settles more slowly than the
                # fixed-joint plant after the desired angle reaches its end.
                'settling_timeout': 20.0,
                'pose_timeout': 1.0,
            },
        ],
    )
    data_recorder = Node(
        package='cooperative_rotation',
        executable='rotation_data_recorder',
        name='rotation_data_recorder',
        output='screen',
        condition=IfCondition(record_data),
        parameters=[{
            'use_sim_time': True,
            'target_angle_deg': ParameterValue(
                target_angle, value_type=float),
            'sample_rate': ParameterValue(recording_rate, value_type=float),
            'output_directory': excel_output_directory,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('target_angle_deg', default_value='5.0'),
        DeclareLaunchArgument(
            'angular_velocity_deg_s', default_value='1.0'),
        DeclareLaunchArgument(
            'angular_acceleration_deg_s2', default_value='0.25'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('record_data', default_value='true'),
        DeclareLaunchArgument('recording_rate', default_value='10.0'),
        DeclareLaunchArgument('translation_distance_m', default_value='1.0'),
        DeclareLaunchArgument('translation_velocity_m_s', default_value='0.05'),
        DeclareLaunchArgument(
            'translation_acceleration_m_s2', default_value='0.025'),
        DeclareLaunchArgument(
            'post_rotation_translation_y_distance_m', default_value='1.0'),
        DeclareLaunchArgument(
            'enable_formation_distance_control', default_value='true'),
        DeclareLaunchArgument('formation_distance_kp', default_value='0.8'),
        DeclareLaunchArgument(
            'formation_distance_max_correction_speed', default_value='0.012'),
        DeclareLaunchArgument('target_normal_force', default_value='14.0'),
        DeclareLaunchArgument(
            'gripper_maximum_effort', default_value='0.8'),
        DeclareLaunchArgument(
            'excel_output_directory',
            default_value=os.path.expanduser(
                '~/ros2_humble_ws/rotation_measurements')),
        # The present Gazebo wrist wrench signal includes large grasp-model
        # moments. Keep it observable but opt-in until a rotation-specific
        # bias and threshold have been validated.
        DeclareLaunchArgument('enable_wrench_safety', default_value='false'),
        simulation,
        # Start recording from launch so robot/payload generation and grasp
        # preparation are included in the report.  The controller itself may
        # still start later while it waits for HOLDING and support removal.
        data_recorder,
        TimerAction(period=21.5, actions=[rotation_controller]),
    ])
