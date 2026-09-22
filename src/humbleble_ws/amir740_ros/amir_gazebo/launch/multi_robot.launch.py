"""複数ロボットの Gazebo シミュレーション (基盤層)。

先頭ロボット (amir1) が gz_sim 本体 + /clock を起動し、後続 (amir2) は
launch_sim:=false で同じ world に spawn を追加するだけ。台数を増やすときは
_ROBOTS に (namespace, x, y, yaw) を足す。

各ロボットは /<ns>/scan, /<ns>/odom, /<ns>/joint_states,
/<ns>/controller_manager を持つ。TF frame ID は map / odom / base_footprint
などの共通名を使い、TF topicを /<ns>/tf と /<ns>/tf_static に分離する。

  ros2 launch amir_gazebo multi_robot.launch.py
  ros2 launch amir_gazebo multi_robot.launch.py world:=warehouse_world.sdf
    world_name:=warehouse_world
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# (namespace, x, y, yaw) — 台数を増やすなら行を追加する
_ROBOTS = [
    ("amir1", "0.0", "0.0", "0.0"),
    ("amir2", "0.0", "1.0", "0.0"),
]


def generate_launch_description():
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    headless = LaunchConfiguration("headless")

    def robot_include(ns, x, y, yaw, launch_sim):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("amir_gazebo"),
                    "launch",
                    "robot_bringup.launch.py",
                ])
            ),
            launch_arguments={
                "namespace": ns,
                "x": x, "y": y, "yaw": yaw,
                "world": world,
                "world_name": world_name,
                "launch_sim": launch_sim,
                "headless": headless,
            }.items(),
        )

    actions = [
        DeclareLaunchArgument("world", default_value="amir_world.sdf"),
        DeclareLaunchArgument("world_name", default_value="default"),
        DeclareLaunchArgument("headless", default_value="false"),
    ]

    for i, (ns, x, y, yaw) in enumerate(_ROBOTS):
        if i == 0:
            # 先頭: sim 本体ごと起動
            actions.append(robot_include(ns, x, y, yaw, "true"))
        else:
            # 後続: gz 起動を待ってから spawn (launch_sim=false)
            actions.append(TimerAction(
                period=float(6 + 3 * i),
                actions=[robot_include(ns, x, y, yaw, "false")],
            ))

    return LaunchDescription(actions)
