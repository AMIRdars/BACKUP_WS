"""Ungrasped CP-5 fixture: five joint reactions and bounded physical excitation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('cooperative_transport_bringup'))/
            'launch/compliance_cp1.launch.py')), launch_arguments={
                'headless': 'true', 'force_torque_system': 'true', 'calibration_fixture': 'true',
                'joint_load_sensors': 'true', 'run_directory': LaunchConfiguration('run_directory')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='cp5_joint_load_bridge',
            parameters=[{'use_sim_time': True}], arguments=[
                f'/{r}/joint_load/Joint_{i}@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
                for r in ('leader', 'follower') for i in range(1, 5)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='cp5_excitation_bridge',
            parameters=[{'use_sim_time': True}], arguments=[
                '/experiment/follower/calibration_wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench'])])
