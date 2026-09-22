"""
survey_ik 調査ツール用 launch ファイル

Gazebo + move_group が起動済みの状態で実行する。
ターゲット位置は引数で変更可能。

使用例:
  ros2 launch amir_operation survey_ik_launch.py
  ros2 launch amir_operation survey_ik_launch.py x:=0.4 y:=0.0 z:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.4",
                              description="Target X position [m]"),
        DeclareLaunchArgument("y", default_value="0.0",
                              description="Target Y position [m]"),
        DeclareLaunchArgument("z", default_value="0.4",
                              description="Target Z position [m]"),

        Node(
            package="amir_operation",
            executable="survey_ik",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
                {"x": LaunchConfiguration("x")},
                {"y": LaunchConfiguration("y")},
                {"z": LaunchConfiguration("z")},
            ],
        ),
    ])
