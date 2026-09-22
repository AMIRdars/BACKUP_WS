import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('amir_gazebo')
    model_file = os.path.join(
        package_share,
        'models',
        'slalom_stage',
        'model.sdf',
    )

    spawn_stage = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', LaunchConfiguration('world'),
            '-file', model_file,
            '-name', LaunchConfiguration('entity_name'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-allow_renaming', 'false',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='default',
            description='Gazebo world name',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='slalom_stage',
            description='Entity name used in Gazebo',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        spawn_stage,
    ])
