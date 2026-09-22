"""
VR/MR テレオペ用 MoveIt Servo 起動 launch

前提: gazebo_bringup.launch.py が起動済みで以下が稼働中
  - robot_state_publisher (use_sim_time=true)
  - gz_ros2_control (controller_manager, update_rate 250Hz)
  - joint_state_broadcaster
  - arm_controller (JointTrajectoryController)  ← Servo 運用時は deactivate する
  - gripper_controller

このlaunchが起動するもの
  - joint_state_filter      : /joint_states → /joint_states_filtered (*_mimic 除去)
  - forward_position_controller : JointGroupPositionController を --inactive で spawn
  - servo_node              : 手先Twist → 関節角ストリーム

使い方 (起動後):
  1) コントローラ切替で Servo 出力先を有効化
     ros2 control switch_controllers \
       --deactivate arm_controller --activate forward_position_controller
  2) Servo 開始
     ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
  3) 手先Twistを流す (テスト or VRブリッジ)
     ros2 run amir_operation servo_test_twist
     または ros2 run amir_operation vr_twist_bridge

  自律動作(JTC)へ戻す:
     ros2 control switch_controllers \
       --deactivate forward_position_controller --activate arm_controller
"""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    absolute_file_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .robot_description_semantic(file_path="config/amir_mecanum3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Servo パラメータ (moveit_servo 名前空間配下に入れる必要がある)
    servo_yaml = load_yaml("amir_operation", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # *_mimic ジョイントを除いた /joint_states_filtered を提供
    joint_state_filter = Node(
        package="amir_moveit_config",
        executable="joint_state_filter.py",
        name="joint_state_filter",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    # Servo の出力先コントローラ。arm_controller と command interface を奪い合うため
    # inactive で spawn し、運用時に switch_controllers で切り替える。
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "-t", "position_controllers/JointGroupPositionController",
            "-p", os.path.join(
                get_package_share_directory("amir_gazebo"),
                "config", "arm_controllers.yaml"),
            "--inactive",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return [
        joint_state_filter,
        forward_position_controller_spawner,
        servo_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
