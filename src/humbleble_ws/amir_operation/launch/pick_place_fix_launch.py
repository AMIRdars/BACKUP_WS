"""
pick_place_hum 用 launch ファイル

MoveGroupInterface クライアントノードに robot_description_kinematics を渡すことで
"No kinematics plugins defined" 警告を解消し、setStartStateToCurrentState() などが
クライアント側で正しく動作するようにする。

使用方法:
  ros2 launch amir_operation pick_place_hum_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .robot_description_semantic(file_path="config/amir_mecanum3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package="amir_operation",
        executable="jointfix",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_node])