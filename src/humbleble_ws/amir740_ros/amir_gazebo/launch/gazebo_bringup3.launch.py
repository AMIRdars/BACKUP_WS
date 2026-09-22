"""単体ロボット bringup (壁 warehouse_world.sdf)。従来どおり namespace なしで起動。
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
                "world": "warehouse_world.sdf",
                "world_name": "warehouse_world",
                "pose_bridge": "true",
                "launch_sim": "true",
            }.items(),
        ),
    ])
