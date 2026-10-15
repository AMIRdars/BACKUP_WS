"""Transport forward 1 m, turn 90 degrees, then move 1 m along world +Y."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    integrated_launch = os.path.join(
        get_package_share_directory('cooperative_transport_bringup'),
        'launch', 'integrated_transport_simulation.launch.py')
    defaults = {
        'headless': 'false',
        'forward_distance_m': '1.0',
        'rotation_angle_deg': '90.0',
        'world_y_distance_m': '1.0',
        'translation_velocity_m_s': '0.10',
        'translation_acceleration_m_s2': '0.025',
        'angular_velocity_deg_s': '1.0',
        'angular_acceleration_deg_s2': '0.25',
        'holding_force_tolerance': '0.5',
        'holding_force_release_threshold': '23.0',
        'record_data': 'true',
        'recording_rate': '10.0',
        'excel_output_directory': os.path.expanduser(
            '~/ros2_humble_ws/rotation_measurements'),
    }
    descriptions = {
        'forward_distance_m': 'Distance along the initial horizontal payload +X.',
        'world_y_distance_m': 'Signed distance along world Y after the turn; +1 is world +Y.',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(
            name, default_value=value,
            description=descriptions.get(name, ''))
          for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(integrated_launch),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in defaults},
                'right_distance_m': '0.0',
                'enable_pivot': 'false',
            }.items()),
    ])
