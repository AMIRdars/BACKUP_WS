from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_navigation2',
            executable='fleet_manager.py',
            name='fleet_manager',
            parameters=[{'robots': ['amir1', 'amir2']}],
            output='screen',
        )
    ])
