"""パラメータ化した単一ロボット bringup (マルチロボット対応の基盤)。

引数:
  namespace   ロボットの namespace (例 amir1)。空なら従来どおり単体・無 prefix。
  x, y, z, yaw  Gazebo へのスポーン位置・向き。
  world       world SDF ファイル名 (amir_gazebo/worlds/ 配下)。
  world_name  world SDF 内の <world name="..."> (pose_bridge 用)。
  pose_bridge  true で /world/<world_name>/pose/info を TF へブリッジ。
  launch_sim  true で gz_sim 本体 + /clock ブリッジ + EGL 環境 + 終了処理を起動。
              マルチロボットでは先頭 1 台だけ true、残りは false にする。

namespace を付けると:
  - robot_description は `xacro ... namespace:=<ns>` で生成 (Ignition 側の
    sensor topic / gz_ros2_control namespace が prefix される)。
  - TF frame ID は共通名のまま、/<ns>/tf と /<ns>/tf_static へ分離する。
  - controller spawner は /<ns>/controller_manager を対象にする。
  - bridge は /<ns>/scan, /<ns>/odom, /<ns>/d435/* の完全修飾名で張る。
"""
import os
import re
import subprocess
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _safe_kill_ignition(context, *args, **kwargs):
    os.system("pkill -9 -f 'ign gazebo' 2>/dev/null")
    os.system("pkill -9 -f 'ruby.*ign' 2>/dev/null")
    os.system("rm -rf /dev/shm/fastrtps_port* 2>/dev/null")
    return [LogInfo(msg="[amir_gazebo] Safe kill done")]


def _robot_sdf(urdf_xml):
    """Convert URDF and retain fixed-frame mecanum friction directions."""
    with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", encoding="utf-8", delete=False) as file:
        file.write(urdf_xml)
        urdf_path = file.name
    try:
        result = subprocess.run(
            ["ign", "sdf", "-p", urdf_path],
            check=True, capture_output=True, text=True)
    finally:
        os.unlink(urdf_path)

    # libsdformat's URDF converter preserves fdir1 but drops Ignition's
    # expressed_in attribute. Without the attribute, the direction rotates
    # with each wheel and eventually couples lateral motion into yaw.
    sdf_xml, count = re.subn(
        r"<fdir1>(1 [-]?1 0)</fdir1>",
        r'<fdir1 ignition:expressed_in="base_footprint">\1</fdir1>',
        result.stdout)
    if count != 4:
        raise RuntimeError(
            f"Expected four mecanum friction directions, found {count}.")
    return sdf_xml


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    world = LaunchConfiguration("world").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    pose_bridge = (
        LaunchConfiguration("pose_bridge").perform(context).lower()
        in ("true", "1", "yes")
    )
    launch_sim = LaunchConfiguration("launch_sim").perform(context).lower() in ("true", "1", "yes")
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1", "yes")
    enable_d435 = LaunchConfiguration("enable_d435").perform(context).lower() in ("true", "1", "yes")
    enable_lidar = LaunchConfiguration("enable_lidar").perform(context).lower() in ("true", "1", "yes")
    enable_wrist_ft = LaunchConfiguration("enable_wrist_ft").perform(context).lower() in ("true", "1", "yes")

    finger_collision_boxes = int(LaunchConfiguration("finger_collision_boxes").perform(context))
    if finger_collision_boxes not in (1, 3, 9):
        raise ValueError("finger_collision_boxes must be 1, 3, or 9")

    # prefix 文字列: ns 空→"" , "amir1"→"amir1/"
    prefix = (ns + "/") if ns else ""
    # namespace 空のときは絶対名を無 prefix にして従来挙動を保つ
    cm = ("/" + ns + "/controller_manager") if ns else "/controller_manager"
    model_name = ns if ns else "amir_mecanum3"

    amir_description_dir = get_package_share_directory("amir_description")
    amir_gazebo_dir = get_package_share_directory("amir_gazebo")
    world_file = os.path.join(amir_gazebo_dir, "worlds", world)
    arm_controllers_yaml = os.path.join(amir_gazebo_dir, "config", "arm_controllers.yaml")
    xacro_file = os.path.join(amir_description_dir, "urdf", "amir_mecanum3_sim.xacro")

    # xacro → robot_description (namespace を渡して Ignition 側を prefix)
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            "namespace": ns,
            "finger_collision_boxes": str(finger_collision_boxes),
            "enable_d435": str(enable_d435).lower(),
            "enable_lidar": str(enable_lidar).lower(),
            "enable_wrist_ft": str(enable_wrist_ft).lower(),
        },
    ).toxml()
    robot_description_content = ParameterValue(
        robot_description_xml,
        value_type=str,
    )
    robot_sdf_xml = _robot_sdf(robot_description_xml)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": True,
        }],
        # TF はフレーム prefix せず、ロボットごとの /<ns>/tf に載せる
        # (絶対 /tf を相対 tf に remap → namespace 配下へ)。
        # ns="" なら /tf のまま (従来どおり単体)。
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=ns,
        output="screen",
        arguments=[
            "-string", robot_sdf_xml,
            "-name", model_name,
            "-x", x, "-y", y, "-z", z, "-Y", yaw,
            "-allow_renaming", "false",
        ],
    )

    # ── bridge (完全修飾名。Ignition transport は ROS namespace と別バス) ──
    scan_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="scan_bridge", output="screen",
        condition=IfCondition(str(enable_lidar).lower()),
        arguments=[f"/{prefix}scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
    )

    d435_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="d435_bridge", output="screen",
        condition=IfCondition(str(enable_d435).lower()),
        arguments=[
            f"/{prefix}d435/image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{prefix}d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{prefix}d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )

    # 真値オドメトリブリッジ。/<ns>/odom/tf をロボットごとの /<ns>/tf へ転送
    # (フレームは無 prefix の標準名 odom→base_footprint。tf topic が分離されるので
    #  複数ロボットでも衝突しない)
    odom_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="odom_bridge", output="screen",
        arguments=[
            f"/{prefix}odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/{prefix}odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[(f"/{prefix}odom/tf", "tf")],
        parameters=[{"use_sim_time": True}],
    )

    wrist_ft_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="wrist_ft_bridge", output="screen",
        condition=IfCondition(str(enable_wrist_ft).lower()),
        arguments=[
            f"/{prefix}ft_sensor@geometry_msgs/msg/WrenchStamped"
            "[gz.msgs.Wrench",
        ],
    )
    # Finger loads regulate the physical grasp even when wrist safety is off.
    finger_ft_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="finger_ft_bridge", output="screen",
        arguments=[
            f"/{prefix}finger_left_wrench@geometry_msgs/msg/WrenchStamped"
            "[gz.msgs.Wrench",
            f"/{prefix}finger_right_wrench@geometry_msgs/msg/WrenchStamped"
            "[gz.msgs.Wrench",
        ],
    )

    # Contact sensor topics are expanded by Gazebo below the world / model /
    # link hierarchy.  Bridge those transport names and remap them to the
    # stable per-robot ROS API consumed by friction_grasp_manager.
    contact_arguments = []
    contact_remappings = []
    for side in ("left", "right"):
        ros_contact = f"/{prefix}finger_{side}_contact"
        for index in range(finger_collision_boxes):
            suffix = "" if index == 0 else f"_{index}"
            sensor = f"finger_{side}_contact_sensor{suffix}"
            gz_contact = (
                f"/world/{world_name}/model/{model_name}/"
                f"link/finger_{side}_1/sensor/{sensor}/contact"
            )
            contact_arguments.append(
                f"{gz_contact}@ros_gz_interfaces/msg/Contacts"
                "[gz.msgs.Contacts")
            # All segments of one finger form one logical ROS contact stream.
            contact_remappings.append((gz_contact, ros_contact))
    contact_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="finger_contact_bridge", output="screen",
        arguments=contact_arguments,
        remappings=contact_remappings,
    )

    # /<ns>/rover_twist → /<ns>/mecanum_drive_controller/reference_unstamped
    rover_twist_relay = Node(
        package="mecanumrover_description",
        executable="rover_twist_relay_ign.py",
        namespace=ns, name="rover_twist_relay_ign", output="log",
        parameters=[{"use_sim_time": True}],
    )

    # ── コントローラ (OnProcessExit で順番に起動、対象 CM を明示) ──
    def spawner(controller, extra=None):
        args = [controller, "--controller-manager", cm,
                "--controller-manager-timeout", "60", "--service-call-timeout", "60.0"]
        if extra:
            args[1:1] = extra
        return Node(package="controller_manager", executable="spawner",
                    namespace=ns, output="screen", arguments=args)

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("arm_controller",
                  extra=["-t", "joint_trajectory_controller/JointTrajectoryController",
                         "-p", arm_controllers_yaml])
    mecanum = spawner("mecanum_drive_controller")
    mecanum_retry_1 = spawner("mecanum_drive_controller")
    mecanum_retry_2 = spawner("mecanum_drive_controller")
    gripper = spawner("gripper_controller",
                      extra=["-t", "position_controllers/GripperActionController",
                             "-p", arm_controllers_yaml])

    def retry_mecanum_or_start_gripper(event, _context):
        """Start the gripper once and retry a transient base failure.

        Gazebo's embedded controller manager occasionally rejects the first
        mecanum controller activation while its wheel interfaces are still
        being registered.  Retrying the same spawner also activates a
        controller that was loaded but left inactive by that transient. The
        spawner may return failure even when activation completed, so the
        rotation controller remains the authority that gates actual motion.
        """
        if event.returncode == 0:
            return [gripper]
        return [
            gripper,
            LogInfo(msg=(
                f"[{model_name}] mecanum_drive_controller start failed; "
                "retrying in 2 s (attempt 2/3).")),
            TimerAction(period=2.0, actions=[mecanum_retry_1]),
        ]

    def final_mecanum_retry(event, _context):
        if event.returncode == 0:
            return []
        return [
            LogInfo(msg=(
                f"[{model_name}] mecanum_drive_controller retry failed; "
                "retrying in 3 s (attempt 3/3).")),
            TimerAction(period=3.0, actions=[mecanum_retry_2]),
        ]

    def start_gripper_after_final_mecanum_attempt(event, _context):
        if event.returncode == 0:
            return []
        return [LogInfo(msg=(
            f"[{model_name}] mecanum_drive_controller could not be started "
            "after three attempts; transport motion remains blocked."))]

    actions = [SetParameter(name="use_sim_time", value=True)]

    # ── sim 本体 (先頭ロボットのみ) ──
    if launch_sim:
        gz_args = " -r -v 1 " + world_file
        if headless:
            gz_args = " -s" + gz_args
        actions += [
            SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES",
                                   "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),
            SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ])
                ),
                launch_arguments=[
                    ("gz_args", gz_args),
                    ("gz_version", "6"),
                    ("on_exit_shutdown", "true"),
                ],
            ),
            Node(package="ros_gz_bridge", executable="parameter_bridge",
                 name="clock_bridge", output="screen",
                 arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[OpaqueFunction(function=_safe_kill_ignition)]
                )
            ),
        ]

    if pose_bridge:
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="pose_bridge", output="screen",
            arguments=[f"/world/{world_name}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        ))

    actions += [
        robot_state_publisher,
        spawn_robot,
        scan_bridge,
        d435_bridge,
        odom_bridge,
        wrist_ft_bridge,
        finger_ft_bridge,
        contact_bridge,
        rover_twist_relay,
        # Controller-manager service calls are serialized. Gazebo Fortress can
        # otherwise drop a response while two spawners load controllers at once.
        # スポーン完了 → jsb → arm → mecanum (成功時のみ) → gripper
        # Entity creation returns before the embedded controller manager has
        # completely initialized.  A short grace period avoids an intermittent
        # first load failure (most often the second robot's JS broadcaster).
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=2.0, actions=[jsb])])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[mecanum])),
        RegisterEventHandler(OnProcessExit(
            target_action=mecanum,
            on_exit=retry_mecanum_or_start_gripper)),
        RegisterEventHandler(OnProcessExit(
            target_action=mecanum_retry_1,
            on_exit=final_mecanum_retry)),
        RegisterEventHandler(OnProcessExit(
            target_action=mecanum_retry_2,
            on_exit=start_gripper_after_final_mecanum_attempt)),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("finger_collision_boxes", default_value="9"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.03"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("world", default_value="amir_world.sdf"),
        DeclareLaunchArgument("world_name", default_value="default"),
        DeclareLaunchArgument("pose_bridge", default_value="false"),
        DeclareLaunchArgument("launch_sim", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("enable_d435", default_value="true"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_wrist_ft", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
