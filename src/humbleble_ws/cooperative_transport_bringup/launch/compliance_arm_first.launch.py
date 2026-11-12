"""Phase 1 only: measured local F/T arm following after native grasp."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def setup(context):
    if LaunchConfiguration('phase').perform(context) != '1':
        raise ValueError('Only Phase 1 is implemented; Phase 2 requires a separate qualification')
    if LaunchConfiguration('enable_base_assist').perform(context) != 'false':
        raise ValueError('Phase 1 requires enable_base_assist=false')
    path = Path(get_package_share_directory('cooperative_transport_bringup'))/'launch/compliance_round_trip.launch.py'
    names = ('enable_base_assist', 'wrench_source_mode', 'leader_base_distance_m',
             'arm_only_speed_m_s', 'headless', 'gui_nvidia_offload', 'keep_gui_open',
             'gui_review_mode', 'output_directory', 'ros_domain_id', 'start_delay_s')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments={
        **{name: LaunchConfiguration(name) for name in names},
        'control_mode': 'ARM_ONLY', 'arm_only_trial': 'native_stroke',
        'arm_only_direction': 'forward', 'record_data': 'true',
        # First micro trial uses raw own odom; no offline GT calibration needed.
        'calibrate_leader_distance': 'false'}.items())]


def generate_launch_description():
    defaults = {'phase': '1', 'enable_base_assist': 'false', 'wrench_source_mode': 'sensor',
                'leader_base_distance_m': '0.009', 'arm_only_speed_m_s': '0.005',
                'headless': 'false', 'gui_nvidia_offload': 'true', 'keep_gui_open': 'true',
                'gui_review_mode': 'paused', 'output_directory': '', 'ros_domain_id': '94',
                'start_delay_s': '10.0'}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value)
                                for name, value in defaults.items()], OpaqueFunction(function=setup)])
