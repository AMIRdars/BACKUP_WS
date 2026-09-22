import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # MoveIt configs の構築
    # NOTE: robot_description (URDF) は amir_bringup.launch.py 側の
    #       robot_state_publisher がすでに /robot_description トピックに配信済み。
    #       ここでは MoveIt が必要とする semantic / kinematics / pipeline のみ使用する。
    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .robot_description_semantic(file_path="config/amir_mecanum3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # move_group アクションサーバー
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz ノード
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("amir_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    wheel_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="wheel_joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states"],
            "zeros": {
                "front_left_joint": 0.0,
                "front_right_joint": 0.0,
                "back_left_joint": 0.0,
                "back_right_joint": 0.0,
            }
        }],
    )

    # Static TF: world → base_footprint
    # N bringup 側で発行していない場合はここで発行する。
    #       bringup 側にも同じ static_tf がある場合はどちらか一方を削除すること。
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_footprint"],
    )

    # --- 削除したノード (amir_bringup.launch.py 側で起動するため) ---
    # - robot_state_publisher   → bringup 側で起動済み
    # - ros2_control_node       → bringup 側で起動済み
    # - joint_state_broadcaster_spawner → bringup 側で起動済み
    # - arm_controller_spawner          → bringup 側で起動済み
    # - hand_controller_spawner         → bringup 側で起動済み

    return [
        wheel_state_publisher,
        static_tf,
        run_move_group_node,
        rviz_node,
    ]
