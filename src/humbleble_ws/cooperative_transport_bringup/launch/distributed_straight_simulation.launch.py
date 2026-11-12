"""Physical-grasp straight transport with an implicit force-based follower."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('cooperative_transport_bringup')
    friction = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'friction_simulation.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'auto_grasp': 'true',
            'contact_only': 'true',
            'enable_coordinator': 'false',
            'target_normal_force': '20.0',
            'gripper_maximum_effort': '1.2',
        }.items())
    leader = Node(
        package='cooperative_transport_control',
        executable='distributed_straight_leader',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'cruise_speed': ParameterValue(
                LaunchConfiguration('cruise_speed'), value_type=float),
        }])
    follower = Node(
        package='cooperative_transport_control',
        executable='distributed_straight_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'force_sign': ParameterValue(
                LaunchConfiguration('follower_force_sign'), value_type=float),
            'force_gain': ParameterValue(
                LaunchConfiguration('follower_force_gain'), value_type=float),
        }])
    recorder = Node(
        package='cooperative_rotation',
        executable='distributed_data_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_data')),
        parameters=[{
            'use_sim_time': True,
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'sample_rate': ParameterValue(
                LaunchConfiguration('recording_rate'), value_type=float),
            'output_directory': LaunchConfiguration('excel_output_directory'),
            'output_file': LaunchConfiguration('excel_output_file'),
            'target_normal_force': 20.0,
        }])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('goal_x', default_value='0.5'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('cruise_speed', default_value='0.025'),
        DeclareLaunchArgument('follower_force_sign', default_value='1.0'),
        DeclareLaunchArgument('follower_force_gain', default_value='0.0001'),
        DeclareLaunchArgument('record_data', default_value='true'),
        DeclareLaunchArgument('recording_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'excel_output_directory',
            default_value='~/ros2_humble_ws/rotation_measurements'),
        DeclareLaunchArgument('excel_output_file', default_value=''),
        friction,
        leader,
        follower,
        recorder,
    ])
