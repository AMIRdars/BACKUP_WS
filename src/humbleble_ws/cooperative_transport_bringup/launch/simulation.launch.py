"""Start Gazebo Fortress, two AMIRs, payload, bridges, and control nodes."""

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
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')
    auto_attach = (
        LaunchConfiguration('auto_attach').perform(context).lower()
        in ('1', 'true', 'yes'))

    gazebo_share = get_package_share_directory('cooperative_transport_gazebo')
    description_share = get_package_share_directory('cooperative_transport_description')
    control_share = get_package_share_directory('cooperative_transport_control')
    amir_gazebo_share = get_package_share_directory('amir_gazebo')

    world_file = os.path.join(gazebo_share, 'worlds', 'cooperative_transport.sdf')
    payload_file = os.path.join(
        description_share, 'models', 'cooperative_payload', 'model.sdf')
    control_parameters = os.path.join(
        control_share, 'config', 'cooperative_transport.yaml')
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
            '/amir1/grasp/state@std_msgs/msg/Bool[gz.msgs.Boolean',
            '/amir2/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amir2/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/amir2/grasp/state@std_msgs/msg/Bool[gz.msgs.Boolean',
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
            '-x', '0.0', '-y', '0.0', '-z', '0.443',
            '-allow_renaming', 'false',
        ],
    )

    common_parameters = [
        control_parameters,
        {'use_sim_time': True},
    ]
    attach_manager = Node(
        package='cooperative_transport_control',
        executable='attach_manager',
        output='screen',
        parameters=common_parameters + [{'auto_attach': auto_attach}],
    )
    coordinator = Node(
        package='cooperative_transport_control',
        executable='coordinator',
        output='screen',
        parameters=common_parameters,
    )
    safety_monitor = Node(
        package='cooperative_transport_control',
        executable='safety_monitor',
        output='screen',
        parameters=common_parameters,
    )

    return [
        SetEnvironmentVariable(
            '__EGL_VENDOR_LIBRARY_FILENAMES',
            '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        simulation,
        bridge,
        # Both arms are at zero joint position. Their TCPs face the payload ends.
        TimerAction(period=2.0, actions=[robot('amir1', '-1.34', '0.0')]),
        TimerAction(period=6.0, actions=[robot('amir2', '1.34', '3.141592653589793')]),
        TimerAction(period=11.0, actions=[spawn_payload]),
        TimerAction(
            period=12.0,
            actions=[attach_manager, coordinator, safety_monitor]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('auto_attach', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
