"""CP-9 identification only: identical robot generation, separate force fixture.

Never launch during grasp or transport. The native contact launch still refuses
calibration_fixture=true. Only this metrology scene has the bounded excitation.
"""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    path = Path(share('cooperative_transport_bringup'))/'launch/compliance_cp9_contact.launch.py'
    spec = importlib.util.spec_from_file_location('cp9_metrology_generator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.setup(context) + [Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='cp9_metrology_joint_load_bridge', parameters=[{'use_sim_time': True}],
        arguments=[f'/{r}/joint_load/Joint_{i}@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
            for r in ('leader', 'follower') for i in range(1, 5)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='cp9_identification_bridge', parameters=[{'use_sim_time': True}],
        arguments=['/experiment/follower/calibration_wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench'])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_directory'),
        *[DeclareLaunchArgument(k, default_value=v, choices=[v]) for k, v in {
            'headless': 'true', 'constraint_fixture': 'false', 'calibration_fixture': 'true',
            'force_torque_system': 'true', 'joint_load_sensors': 'true', 'weld_cfm': '0'}.items()],
        OpaqueFunction(function=setup)])
