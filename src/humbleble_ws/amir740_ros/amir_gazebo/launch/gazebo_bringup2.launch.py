"""単体ロボット bringup (倉庫 husky_depot.sdf)。従来どおり namespace なしで起動。
実体は robot_bringup.launch.py。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("amir_gazebo"), "launch", "robot_bringup.launch.py"])
            ),
            launch_arguments={
                "namespace": "",
                "world": "husky_depot.sdf",
                "world_name": "world_demo",
                "pose_bridge": "true",
                "launch_sim": "true",
            }.items(),
        ),
    ])
