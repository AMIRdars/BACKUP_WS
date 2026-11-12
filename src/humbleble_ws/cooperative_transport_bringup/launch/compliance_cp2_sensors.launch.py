"""CP-2 sensor-only fixture; no grasp or autonomous transport is enabled."""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_directory', default_value=str(Path(tempfile.gettempdir())/'amir_cp2_sensors')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('cooperative_transport_bringup'))/'launch/compliance_cp1.launch.py')),
            launch_arguments={'headless': 'true', 'force_torque_system': 'true',
                              'calibration_fixture': 'true',
                              'run_directory': LaunchConfiguration('run_directory')}.items()),
        # Experiment input: ROS -> Gazebo. No robot controller reads it.
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='cp2_calibration_wrench_bridge', parameters=[{'use_sim_time': True}],
             arguments=['/experiment/follower/calibration_wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench'])])
