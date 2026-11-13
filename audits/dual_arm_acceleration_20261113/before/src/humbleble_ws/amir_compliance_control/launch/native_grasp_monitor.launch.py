"""Explicit post-preparation local monitoring; no simulator/evaluator launch."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    maintain = LaunchConfiguration('grip_maintenance').perform(context) == 'true'
    hold_imbalance=LaunchConfiguration('hold_safe_force_imbalance').perform(context)=='true'
    source_watchdog=LaunchConfiguration('simulation_source_watchdog').perform(context)=='true'
    actions = [Node(package='amir_compliance_control', executable='native_grasp_monitor', namespace=r,
        parameters=[{'use_sim_time': True, 'urdf_file': str(folder/(r+'.urdf')), 'maintenance_required': maintain,
                     'simulation_source_watchdog':source_watchdog}],
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'},
        output='screen') for r in ('leader', 'follower')]
    if maintain:
        actions += [Node(package='amir_compliance_control', executable='local_grip_maintenance', namespace=r,
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/(r+'.urdf')),
                         'hold_safe_force_imbalance':hold_imbalance}],
            output='screen') for r in ('leader', 'follower')]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),
        DeclareLaunchArgument('grip_maintenance', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hold_safe_force_imbalance',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('simulation_source_watchdog',default_value='false',choices=['true','false']),
        OpaqueFunction(function=setup)])
