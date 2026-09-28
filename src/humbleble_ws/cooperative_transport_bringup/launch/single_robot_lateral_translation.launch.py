"""Launch one AMIR and translate it right without grasping a payload."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amir_gazebo_share = get_package_share_directory('amir_gazebo')
    robot_launch = os.path.join(
        amir_gazebo_share, 'launch', 'robot_bringup.launch.py')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            'namespace': 'amir1',
            'x': '-1.0',
            'y': '0.0',
            'z': '0.03',
            'yaw': '0.0',
            'world_name': 'cooperative_transport_friction',
            'launch_sim': 'false',
            'pose_bridge': 'false',
            'headless': LaunchConfiguration('headless'),
            'enable_d435': 'false',
        }.items(),
    )
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
            'gz_version': '6',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='single_robot_clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    mover = Node(
        package='cooperative_transport_control',
        executable='single_robot_lateral_translation',
        output='screen',
        parameters=[{
            'robot_namespace': 'amir1',
            'distance_m': LaunchConfiguration('right_distance_m'),
            'speed_m_s': LaunchConfiguration('speed_m_s'),
            'acceleration_m_s2': LaunchConfiguration('acceleration_m_s2'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'gz_args',
            default_value=(
                '-r -v 2 '
                + os.path.join(
                    get_package_share_directory('cooperative_transport_gazebo'),
                    'worlds', 'cooperative_transport_contact.sdf'))),
        DeclareLaunchArgument('right_distance_m', default_value='0.5'),
        DeclareLaunchArgument('speed_m_s', default_value='0.05'),
        DeclareLaunchArgument('acceleration_m_s2', default_value='0.025'),
        simulator,
        clock_bridge,
        TimerAction(period=2.0, actions=[robot]),
        TimerAction(period=12.0, actions=[mover]),
    ])
