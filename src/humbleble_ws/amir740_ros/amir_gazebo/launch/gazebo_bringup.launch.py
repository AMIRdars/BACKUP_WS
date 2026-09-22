"""単体ロボット bringup (amir_world.sdf)。従来どおり namespace なしで起動する。
実体はパラメータ化された robot_bringup.launch.py。マルチロボットは
multi_robot.launch.py を参照。"""
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
                "world": "amir_world.sdf",
                "world_name": "default",
                "launch_sim": "true",
            }.items(),
        ),
    ])
