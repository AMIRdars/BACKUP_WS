import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    this_dir = get_package_share_directory('mecanum_navigation2')
    gazebo_dir = get_package_share_directory('amir_gazebo')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'multi_robot.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    def navigation(robot, delay, rviz):
        return TimerAction(
            period=delay,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(this_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': robot,
                    'use_namespace': 'true',
                    'use_sim_time': 'true',
                    'mode': mode,
                    'map': map_file,
                    'use_rviz': rviz,
                }.items(),
            )],
        )

    fleet = TimerAction(
        period=25.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_dir, 'launch', 'fleet_manager.launch.py')),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'localization']),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(this_dir, 'map', 'amir_world.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        simulator,
        navigation('amir1', 15.0, use_rviz),
        navigation('amir2', 20.0, 'false'),
        fleet,
    ])
