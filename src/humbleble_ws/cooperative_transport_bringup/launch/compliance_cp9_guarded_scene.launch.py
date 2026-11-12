"""Native CP-9 scene plus measured joint reaction bridges for existing gates."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),
        DeclareLaunchArgument('physics_step_divisor',default_value='1',choices=['1','2','4']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('cooperative_transport_bringup'))/
            'launch/compliance_cp9_contact.launch.py')), launch_arguments={
                'run_directory': LaunchConfiguration('run_directory'),
                'physics_step_divisor': LaunchConfiguration('physics_step_divisor')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='cp9_native_joint_load_bridge',
            parameters=[{'use_sim_time': True}], arguments=[
                f'/{r}/joint_load/Joint_{i}@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
                for r in ('leader', 'follower') for i in range(1, 5)])])
