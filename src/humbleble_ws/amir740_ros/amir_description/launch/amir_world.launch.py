from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import os
import subprocess


def generate_launch_description():
    # urdf_file_path = os.path.join(
    #     os.getenv('HOME'),
    #     'chujo_ws', 'src', 'amir740_ros', 'amir_description', 'urdf', 'amir_mecanum3.urdf'
    # )

    # with open(urdf_file_path, 'r')as infp:
    #     robot_desc = infp.read()

    xacro_file_path = os.path.join(
        os.getenv('HOME'),
        'chujo_ws', 'src', 'amir740_ros', 'amir_description', 'urdf', 'amir_for_rover.xacro'
    )

    # xacro → urdf に変換
    result = subprocess.run(['xacro', xacro_file_path], capture_output=True, text=True)
    robot_desc = result.stdout


    return LaunchDescription([
        # Gazebo 起動
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # robot_state_publisher 起動
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # spawn_entity.py でスポーン
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'amir',
                '-topic', '/robot_description',
                '-x', '0', '-y', '0', '-z', '0.3'
            ],
            output='screen'
        )
    ])
