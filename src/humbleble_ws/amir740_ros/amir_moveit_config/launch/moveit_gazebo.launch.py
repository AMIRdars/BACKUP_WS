import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _namespaced_rviz_config(base_path, ns):
    """MotionPlanning ディスプレイの 'Move Group Namespace' を /<ns> に設定した
    一時 rviz 設定を生成する。

    RViz の MotionPlanning プラグインはルート namespace の内部ノードで
    MoveGroupInterface を作り、接続先 move_group はこのプロパティだけで決まる
    (RViz ノードの namespace は使われない)。空のままだとルートの /move_group を
    探し、Plan/Execute が 'MoveGroup action client/server not ready' で即失敗する。
    """
    with open(base_path) as f:
        cfg = yaml.safe_load(f)
    for disp in cfg.get('Visualization Manager', {}).get('Displays', []):
        if isinstance(disp, dict) and disp.get('Class') == 'moveit_rviz_plugin/MotionPlanning':
            disp['Move Group Namespace'] = '/' + ns
    fd, path = tempfile.mkstemp(prefix='moveit_%s_' % ns, suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False)
    return path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config", default_value="moveit.rviz",
            description="RViz configuration file",
        ),
        # マルチロボット: namespace 付きで起動 (例 namespace:=amir1)。
        # 空なら従来どおり単体 (グローバル名)。
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # Nav2/slam_toolbox と同時起動するときは true。slam が map->odom を
        # 発行するため、MoveIt 側の静的 TF を world->odom ではなく world->map に
        # して odom の親競合 (二重親) を避ける。単体 (MoveIt のみ) は false のまま。
        DeclareLaunchArgument("nav2", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)

    # Gazebo シミュレーション用 MoveIt2 launch
    # 前提: gazebo_bringup / multi_robot が起動済みで、対象ロボットの
    #   robot_state_publisher / controller_manager / arm_controller 等が
    #   namespace 配下で稼働していること。
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

    # TF はロボットごとの /<ns>/tf に載っているため、MoveIt 系ノードも
    # 絶対 /tf を相対 tf に remap して namespace 配下を購読する。
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # gazebo_ros2_control が生成するミミック拘束 "<joint>_mimic" を
    # /<ns>/joint_states から除外し /<ns>/joint_states_filtered として再配信する。
    joint_state_filter = Node(
        package="amir_moveit_config",
        executable="joint_state_filter.py",
        namespace=ns,
        name="joint_state_filter",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        remappings=[("joint_states", "joint_states_filtered")] + tf_remaps,
    )

    rviz_base = LaunchConfiguration("rviz_config").perform(context)
    rviz_config_path = os.path.join(
        get_package_share_directory("amir_moveit_config"), "config", rviz_base
    )
    # namespace 起動時は MotionPlanning の接続先 move_group を /<ns> に向ける
    # (これをしないと RViz はルートの move_group を探し Plan/Execute が失敗する)。
    if ns:
        rviz_config_path = _namespaced_rviz_config(rviz_config_path, ns)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        name="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        remappings=[("joint_states", "joint_states_filtered")] + tf_remaps,
    )

    # SRDF virtual_joint は world→base_footprint。sim は odom→base_footprint を
    # 発行するため MoveIt 側で world を木に繋ぐ静的 TF を出す。
    #   単体 (nav2=false): world→odom (chain: world→odom→base_footprint)
    #   Nav2 同時 (nav2=true): slam が map→odom を出すので world→map にする
    #     (chain: world→map→odom→base_footprint)。world→odom にすると odom の親が
    #     world(静的) と map(slam動的) で競合し TF がフラップして Nav2 が壊れる。
    # ロボットごとの /<ns>/tf(_static) に載せる。
    with_nav2 = LaunchConfiguration("nav2").perform(context).lower() in ("true", "1", "yes")
    world_child = "map" if with_nav2 else "odom"
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ns,
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", world_child],
        parameters=[{"use_sim_time": True}],
        remappings=tf_remaps,
    )

    return [
        joint_state_filter,
        static_tf,
        run_move_group_node,
        rviz_node,
    ]
