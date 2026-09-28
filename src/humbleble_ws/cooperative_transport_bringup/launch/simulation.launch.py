"""Spawn two AMIRs at Q_HOME, then spawn and grasp one payload."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    headless = (
        LaunchConfiguration('headless').perform(context).lower()
        in ('1', 'true', 'yes'))

    gazebo_share = get_package_share_directory('cooperative_transport_gazebo')
    description_share = get_package_share_directory(
        'cooperative_transport_description')
    amir_gazebo_share = get_package_share_directory('amir_gazebo')

    world_file = os.path.join(gazebo_share, 'worlds', 'cooperative_transport.sdf')
    payload_file = os.path.join(
        description_share, 'models', 'cooperative_payload', 'model.sdf')
    robot_launch = os.path.join(
        amir_gazebo_share, 'launch', 'robot_bringup.launch.py')

    gz_args = f'-r -v 2 {world_file}'
    if headless:
        gz_args = '-s ' + gz_args

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': gz_args,
            'gz_version': '6',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cooperative_transport_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/amir1/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amir1/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amir1/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg',
            '/amir2/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amir2/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amir2/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg',
            '/world/cooperative_transport/remove'
            '@ros_gz_interfaces/srv/DeleteEntity',
        ],
    )

    def robot(namespace, x, yaw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'namespace': namespace,
                'x': x,
                'y': '0.0',
                'z': '0.03',
                'yaw': yaw,
                'world_name': 'cooperative_transport',
                'launch_sim': 'false',
                'pose_bridge': 'false',
                'headless': str(headless).lower(),
            }.items(),
        )

    spawn_payload = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cooperative_payload',
        output='screen',
        arguments=[
            '-world', 'cooperative_transport',
            '-name', 'cooperative_payload',
            '-file', payload_file,
            # Roll about the 1.20 m longitudinal axis so the 1.20 x 0.08 m
            # (largest) payload face is horizontal.  Its vertical thickness
            # is then 0.03 m, so its centre is 0.645 + 0.03 / 2 m above the
            # temporary support.
            '-x', '0.0', '-y', '0.0', '-z', '0.660',
            '-R', '1.5707963267948966',
            '-allow_renaming', 'false',
        ],
    )

    attach_manager = Node(
        package='cooperative_transport_control',
        executable='attach_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # dual_base_approach requests grasp only after both bases arrive.
            'auto_attach': False,
            'close_grippers_before_attach': True,
            'gripper_close_position': 0.20,
            'gripper_maximum_effort': 0.8,
            'require_gripper_contacts_before_attach': True,
        }],
    )

    base_approach = Node(
        package='cooperative_transport_control',
        executable='dual_base_approach',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': ['amir1', 'amir2'],
            'open_grippers_before_approach': True,
            # -1.0 rad is the widest reliable command: the exact lower
            # mechanical limit can leave Gazebo's gripper joint stalled.
            'gripper_open_position': -1.0,
            'gripper_maximum_effort': 0.8,
            # Each AMIR travels 10 mm farther toward the payload after
            # opening its gripper, compared with the previous 0.10 m move.
            'approach_distance': 0.110,
            'maximum_speed': 0.035,
            'position_tolerance': 0.001,
        }],
    )

    support_remover = Node(
        package='cooperative_transport_control',
        executable='grasp_support_remover',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_name': 'cooperative_transport',
            'support_model_name': 'grasp_support',
            'removal_delay': 1.0,
            # This stage uses the existing fixed DetachableJoint grasp rather
            # than the separate contact/friction grasp manager.
            'require_physical_contact': False,
            'require_friction_holding': False,
        }],
    )

    arm_home_positioner = Node(
        package='coop_transport_controller',
        executable='arm_home_positioner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': ['amir1', 'amir2'],
            'motion_duration': LaunchConfiguration('home_motion_duration'),
            'compensate_base': False,
        }],
    )

    return [
        SetEnvironmentVariable(
            '__EGL_VENDOR_LIBRARY_FILENAMES',
            '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        simulation,
        bridge,
        # Spawn 0.10 m farther from the payload than the grasp positions.
        # After the supported payload appears, both bases approach together.
        TimerAction(period=2.0, actions=[robot('amir1', '-1.174987', '0.0')]),
        TimerAction(
            period=6.0,
            actions=[robot('amir2', '1.174987', '3.141592653589793')]),
        # Reassert the common posture after all controllers become available.
        TimerAction(
            period=float(LaunchConfiguration('home_start_delay').perform(context)),
            actions=[arm_home_positioner]),
        # Start the grasp nodes just before payload creation. The approach
        # node repeatedly sends detach so Fortress cannot keep the payload's
        # DetachableJoint in its default attached state during positioning.
        TimerAction(
            period=19.0,
            actions=[attach_manager, support_remover, base_approach]),
        # Only spawn after the two arm controllers have reached Q_HOME; base
        # motion begins after both initial grasp joints report detached.
        TimerAction(period=20.0, actions=[spawn_payload]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('home_start_delay', default_value='11.0'),
        DeclareLaunchArgument('home_motion_duration', default_value='8.0'),
        OpaqueFunction(function=_launch_setup),
    ])
