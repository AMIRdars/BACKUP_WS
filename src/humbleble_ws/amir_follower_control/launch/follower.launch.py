"""Attach a disabled local controller to an existing audited robot."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    source = os.path.join(get_package_share_directory('amir_description'),
                          'urdf', 'amir_mecanum3_sim.xacro')
    xml = xacro.process_file(source, mappings={'namespace': robot}).toxml()
    description_file = LaunchConfiguration('robot_description_file').perform(context)
    if description_file:
        xml = Path(description_file).read_text()
    return [Node(package='amir_follower_control', executable='follower_controller',
                 namespace=robot, output='screen', parameters=[
                     LaunchConfiguration('config'),
                     {'robot_description': xml, 'use_sim_time': True,
                      'log_file': LaunchConfiguration('log_file')}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='amir2'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('amir_follower_control'), 'config', 'follower.yaml')),
        DeclareLaunchArgument('log_file', default_value=''),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        OpaqueFunction(function=setup)])
