"""Start fixed-grasp simulation and execute specified-axis payload rotation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory(
        'cooperative_transport_bringup')
    rotation_share = get_package_share_directory('cooperative_rotation')
    simulation_launch = os.path.join(
        bringup_share, 'launch', 'simulation.launch.py')
    rotation_parameters = os.path.join(
        rotation_share, 'config', 'rotation_params.yaml')

    headless = LaunchConfiguration('headless')
    target_angle = LaunchConfiguration('target_angle_deg')
    angular_velocity = LaunchConfiguration('angular_velocity_deg_s')
    auto_start = LaunchConfiguration('auto_start')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'headless': headless,
            'auto_attach': 'true',
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
                'auto_start': ParameterValue(auto_start, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('target_angle_deg', default_value='10.0'),
        DeclareLaunchArgument(
            'angular_velocity_deg_s', default_value='2.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        simulation,
        # The base simulation starts control nodes after both robots and the
        # payload have spawned. The rotation controller then waits for fresh
        # odometry, TF, and both logical grasp states before auto-starting.
        TimerAction(period=12.5, actions=[rotation_controller]),
    ])
