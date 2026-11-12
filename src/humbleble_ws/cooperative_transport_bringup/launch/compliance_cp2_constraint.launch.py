"""Stage-A constraint fixture. Attachment is explicit, with support retained."""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_directory', default_value=str(Path(tempfile.gettempdir())/'amir_cp2_constraint')),
        DeclareLaunchArgument('joint_load_sensors', default_value='false'),
        DeclareLaunchArgument('weld_cfm', default_value='0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('cooperative_transport_bringup'))/'launch/compliance_cp1.launch.py')),
            launch_arguments={'headless': 'true', 'force_torque_system': 'true',
                'constraint_fixture': 'true', 'joint_load_sensors': LaunchConfiguration('joint_load_sensors'),
                'weld_cfm': LaunchConfiguration('weld_cfm'),
                'run_directory': LaunchConfiguration('run_directory')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='joint_load_bridge',
             condition=IfCondition(LaunchConfiguration('joint_load_sensors')),
             parameters=[{'use_sim_time': True}], arguments=[
                 f'/{r}/joint_load/Joint_{i}@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
                 for r in ('leader', 'follower') for i in range(1, 5)])])
