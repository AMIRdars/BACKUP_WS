"""Launch the cooperative-motion Action server and a Behavior Tree program."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('cooperative_transport_bringup')
    bt_share = get_package_share_directory('coop_transport_bt')
    simulation_launch = os.path.join(bringup_share, 'launch', 'simulation.launch.py')
    default_tree = os.path.join(bt_share, 'behavior_trees', 'transport_sequence.xml')
    start_simulation = LaunchConfiguration('start_simulation')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        condition=IfCondition(start_simulation),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_simulation', default_value='false'),
        DeclareLaunchArgument('bt_start_delay', default_value='14.0'),
        DeclareLaunchArgument('bt_xml_path', default_value=default_tree),
        simulation,
        Node(
            package='coop_transport_controller',
            executable='cooperative_motion_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        # The default lets Gazebo spawn both robots, the payload, and the
        # existing coordinator before the first BT Action is sent.
        TimerAction(
            period=LaunchConfiguration('bt_start_delay'),
            actions=[Node(
                package='coop_transport_bt',
                executable='bt_runner',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'bt_xml_path': LaunchConfiguration('bt_xml_path'),
                }],
            )]),
    ])
