"""
3DOF 位置ジョグ版キーボード操作の起動 launch。

servo_keyboard_jog は KDL でヤコビアンを計算するため URDF (robot_description) が必要。
ここで MoveItConfigsBuilder から robot_description を取得しノードへ渡す。

前提: gazebo_bringup.launch.py と vr_servo_launch.py が起動済みで
  - servo_node が立ち上がっている
  - forward_position_controller が active (switch_controllers 済み)

注意: このノードは端末でキー入力を読むので、必ず前面の端末で
      `ros2 launch ... ` ではなく `ros2 run` 的に前面実行する。
      launch だと出力がまとまり操作しにくい場合は README の手動手順を参照。
"""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .to_moveit_configs()
    )

    jog_node = Node(
        package="amir_operation",
        executable="servo_keyboard_jog",
        name="servo_keyboard_jog",
        output="screen",
        emulate_tty=True,          # 端末入力を扱えるように
        parameters=[
            moveit_config.robot_description,   # {"robot_description": "<urdf xml>"}
            {"use_sim_time": True},
            {"base_link": "base_footprint"},
            {"tip_link": "tcp_link"},
        ],
    )
    return [jog_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
