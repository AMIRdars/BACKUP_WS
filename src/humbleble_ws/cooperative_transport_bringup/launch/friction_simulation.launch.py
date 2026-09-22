"""Start stage B cooperative transport using contact and friction only."""

import os

from ament_index_python.packages import get_package_prefix
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
    headless = LaunchConfiguration('headless').perform(context).lower() in (
        '1', 'true', 'yes')
    auto_grasp = LaunchConfiguration('auto_grasp').perform(context).lower() in (
        '1', 'true', 'yes')
    auto_grasp_delay = float(
        LaunchConfiguration('auto_grasp_delay').perform(context))
    max_linear_speed = float(
        LaunchConfiguration('max_linear_speed').perform(context))
    max_linear_acceleration = float(
        LaunchConfiguration('max_linear_acceleration').perform(context))
    yaw_tolerance = float(
        LaunchConfiguration('yaw_tolerance').perform(context))
    position_feedback_weight = float(LaunchConfiguration(
        'measured_payload_position_weight').perform(context))
    formation_alignment_kp = float(
        LaunchConfiguration('formation_alignment_kp').perform(context))
    alignment_max_fraction = float(LaunchConfiguration(
        'formation_alignment_max_fraction').perform(context))
    alignment_tolerance = float(LaunchConfiguration(
        'formation_alignment_tolerance').perform(context))
    recovery_start = float(LaunchConfiguration(
        'formation_recovery_start').perform(context))
    recovery_stop = float(LaunchConfiguration(
        'formation_recovery_stop').perform(context))
    recovery_speed = float(LaunchConfiguration(
        'formation_recovery_speed').perform(context))
    horizontal_slip_limit = float(
        LaunchConfiguration('horizontal_slip_limit').perform(context))
    vertical_slip_limit = float(
        LaunchConfiguration('vertical_slip_limit').perform(context))
    maximum_base_speed = float(
        LaunchConfiguration('maximum_base_speed').perform(context))
    base_speed_fault_duration = float(
        LaunchConfiguration('base_speed_fault_duration').perform(context))
    odometry_timeout = float(
        LaunchConfiguration('odometry_timeout').perform(context))
    enable_wrench_safety = LaunchConfiguration(
        'enable_wrench_safety').perform(context).lower() in (
            '1', 'true', 'yes')
    enable_lateral_admittance = LaunchConfiguration(
        'enable_lateral_admittance').perform(context).lower() in (
            '1', 'true', 'yes')
    admittance_force_sign = float(
        LaunchConfiguration('admittance_force_sign').perform(context))
    open_world = LaunchConfiguration('open_world').perform(context).lower() in (
        '1', 'true', 'yes')
    contact_only = LaunchConfiguration(
        'contact_only').perform(context).lower() in ('1', 'true', 'yes')
    target_normal_force = float(
        LaunchConfiguration('target_normal_force').perform(context))
    gripper_maximum_effort = float(
        LaunchConfiguration('gripper_maximum_effort').perform(context))
    enable_formation_distance_control = LaunchConfiguration(
        'enable_formation_distance_control').perform(context).lower() in (
            '1', 'true', 'yes')
    formation_distance_kp = float(LaunchConfiguration(
        'formation_distance_kp').perform(context))
    formation_distance_max_correction_speed = float(LaunchConfiguration(
        'formation_distance_max_correction_speed').perform(context))

    gazebo_share = get_package_share_directory('cooperative_transport_gazebo')
    gazebo_plugin_path = os.path.join(
        get_package_prefix('cooperative_transport_gazebo'), 'lib')
    description_share = get_package_share_directory(
        'cooperative_transport_description')
    control_share = get_package_share_directory('cooperative_transport_control')
    amir_gazebo_share = get_package_share_directory('amir_gazebo')

    if contact_only:
        world_name = 'cooperative_transport_contact.sdf'
    elif open_world:
        world_name = 'cooperative_transport_open.sdf'
    else:
        world_name = 'cooperative_transport_friction.sdf'
    world_file = os.path.join(gazebo_share, 'worlds', world_name)
    payload_file = os.path.join(
        description_share, 'models', 'cooperative_payload_friction', 'model.sdf')
    parameters = os.path.join(
        control_share, 'config', 'friction_transport.yaml')
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

    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/cooperative_transport_friction/pose/info'
        '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/world/cooperative_transport_friction/remove'
        '@ros_gz_interfaces/srv/DeleteEntity',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cooperative_transport_friction_bridge',
        output='screen',
        arguments=bridge_arguments,
    )

    def robot(namespace, x, yaw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                'namespace': namespace,
                'x': x, 'y': '0.0', 'z': '0.03', 'yaw': yaw,
                'world_name': 'cooperative_transport_friction',
                'launch_sim': 'false', 'pose_bridge': 'false',
                'headless': str(headless).lower(),
            }.items(),
        )

    spawn_payload = Node(
        package='ros_gz_sim', executable='create',
        name='spawn_friction_payload', output='screen',
        arguments=[
            '-world', 'cooperative_transport_friction',
            '-name', 'cooperative_payload', '-file', payload_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.443',
            '-allow_renaming', 'false',
        ],
    )
    common = [parameters, {'use_sim_time': True}]
    grasp_manager = Node(
        package='cooperative_transport_control',
        executable='friction_grasp_manager', output='screen',
        parameters=common + [{
            'auto_grasp': auto_grasp,
            'auto_grasp_delay': auto_grasp_delay,
            'require_contact': contact_only,
            'target_normal_force': target_normal_force,
            'maximum_effort': gripper_maximum_effort,
        }],
    )
    support_remover = Node(
        package='cooperative_transport_control',
        executable='grasp_support_remover', output='screen',
        parameters=common,
    )
    coordinator = Node(
        package='cooperative_transport_control', executable='coordinator',
        output='screen',
        parameters=common + [{
            'max_linear_speed': max_linear_speed,
            'max_linear_acceleration': max_linear_acceleration,
            'odometry_timeout': odometry_timeout,
            'yaw_tolerance': yaw_tolerance,
            'measured_payload_position_weight': position_feedback_weight,
            'formation_alignment_kp': formation_alignment_kp,
            'formation_alignment_max_fraction': alignment_max_fraction,
            'formation_alignment_tolerance': alignment_tolerance,
            'formation_recovery_start': recovery_start,
            'formation_recovery_stop': recovery_stop,
            'formation_recovery_speed': recovery_speed,
            'enable_lateral_admittance': enable_lateral_admittance,
            'admittance_force_sign': admittance_force_sign,
            'enable_formation_distance_control': (
                enable_formation_distance_control),
            'formation_distance_kp': formation_distance_kp,
            'formation_distance_max_correction_speed': (
                formation_distance_max_correction_speed),
        }])
    safety_monitor = Node(
        package='cooperative_transport_control', executable='safety_monitor',
        output='screen', parameters=common + [{
            'maximum_base_speed': maximum_base_speed,
            'base_speed_fault_duration': base_speed_fault_duration,
            'odometry_timeout': odometry_timeout,
        }])
    slip_monitor = Node(
        package='cooperative_transport_control', executable='slip_monitor',
        output='screen',
        parameters=common + [{
            'horizontal_slip_limit': horizontal_slip_limit,
            'vertical_slip_limit': vertical_slip_limit,
        }])
    wrench_monitor = Node(
        package='cooperative_transport_control', executable='wrench_monitor',
        output='screen', parameters=common + [{
            'enable_threshold_stop': enable_wrench_safety,
        }])

    return [
        SetEnvironmentVariable(
            '__EGL_VENDOR_LIBRARY_FILENAMES',
            '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable(
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            gazebo_plugin_path + ':' + os.environ.get(
                'IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')),
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            gazebo_plugin_path + ':' + os.environ.get(
                'GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        simulation,
        bridge,
        TimerAction(period=2.0, actions=[robot('amir1', '-1.34', '0.0')]),
        TimerAction(
            period=9.0,
            actions=[robot('amir2', '1.34', '3.141592653589793')]),
        # Open the fingers before placing the full-length collision between
        # them.  Robot 2 is spawned later and its controllers can take several
        # seconds to activate, so keep a deterministic margin; spawning the
        # payload while the fingers are still opening creates a false impact
        # load instead of a controlled grasp.
        TimerAction(period=15.0, actions=[grasp_manager]),
        TimerAction(period=28.0, actions=[spawn_payload]),
        TimerAction(
            period=29.0,
            actions=[
                support_remover, coordinator, safety_monitor, slip_monitor,
                wrench_monitor]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('auto_grasp', default_value='true'),
        DeclareLaunchArgument('auto_grasp_delay', default_value='4.0'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.035'),
        DeclareLaunchArgument(
            'max_linear_acceleration', default_value='0.040'),
        DeclareLaunchArgument('yaw_tolerance', default_value='0.0349066'),
        DeclareLaunchArgument(
            'measured_payload_position_weight', default_value='1.0'),
        DeclareLaunchArgument('formation_alignment_kp', default_value='0.0'),
        DeclareLaunchArgument(
            'formation_alignment_max_fraction', default_value='1.0'),
        DeclareLaunchArgument(
            'formation_alignment_tolerance', default_value='0.008'),
        DeclareLaunchArgument('formation_recovery_start', default_value='0.0'),
        DeclareLaunchArgument('formation_recovery_stop', default_value='0.0'),
        DeclareLaunchArgument('formation_recovery_speed', default_value='0.0'),
        DeclareLaunchArgument('horizontal_slip_limit', default_value='0.030'),
        DeclareLaunchArgument('vertical_slip_limit', default_value='0.030'),
        DeclareLaunchArgument('maximum_base_speed', default_value='0.120'),
        DeclareLaunchArgument(
            'base_speed_fault_duration', default_value='0.0'),
        DeclareLaunchArgument('odometry_timeout', default_value='0.30'),
        DeclareLaunchArgument('enable_wrench_safety', default_value='false'),
        DeclareLaunchArgument(
            'enable_lateral_admittance', default_value='false'),
        DeclareLaunchArgument('admittance_force_sign', default_value='1.0'),
        DeclareLaunchArgument('open_world', default_value='false'),
        DeclareLaunchArgument('contact_only', default_value='false'),
        DeclareLaunchArgument('target_normal_force', default_value='14.0'),
        DeclareLaunchArgument(
            'gripper_maximum_effort', default_value='0.8'),
        DeclareLaunchArgument(
            'enable_formation_distance_control', default_value='true'),
        DeclareLaunchArgument('formation_distance_kp', default_value='0.8'),
        DeclareLaunchArgument(
            'formation_distance_max_correction_speed', default_value='0.012'),
        OpaqueFunction(function=_launch_setup),
    ])
