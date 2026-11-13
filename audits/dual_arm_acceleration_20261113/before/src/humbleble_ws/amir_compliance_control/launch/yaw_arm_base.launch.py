"""Compatibility entry point routed through the mandatory chapter 23 gate."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    names=('run_directory','follower_policy_file','wrench_calibration_file','observer_policy_file','yaw_heading_frame')
    target=Path(get_package_share_directory('amir_compliance_control'))/'launch/rotation_follow_integrated.launch.py'
    arguments={name:LaunchConfiguration(name) for name in names}
    arguments['rotation_follow_mode']='XY_YAW_COMP'
    return LaunchDescription([DeclareLaunchArgument(name) for name in names[:3]]+
        [DeclareLaunchArgument('observer_policy_file',default_value=''),
         DeclareLaunchArgument('yaw_heading_frame',default_value='tcp_link'),
         IncludeLaunchDescription(PythonLaunchDescriptionSource(str(target)),launch_arguments=arguments.items())])
