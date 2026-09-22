"""Run a repeatable trajectory evaluation with the stage-B friction grasp."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory(
        'cooperative_transport_bringup')
    control_share = get_package_share_directory(
        'cooperative_transport_control')
    friction_launch = os.path.join(
        bringup_share, 'launch', 'friction_simulation.launch.py')
    parameters = os.path.join(
        control_share, 'config', 'trajectory_evaluation.yaml')

    headless = LaunchConfiguration('headless').perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    auto_start = LaunchConfiguration('auto_start').perform(context).lower() in (
        '1', 'true', 'yes')
    enable_wrench_safety = LaunchConfiguration(
        'enable_wrench_safety').perform(context)
    enable_lateral_admittance = LaunchConfiguration(
        'enable_lateral_admittance').perform(context).lower() in (
            '1', 'true', 'yes')
    admittance_force_sign = LaunchConfiguration(
        'admittance_force_sign').perform(context)
    results_directory = LaunchConfiguration(
        'results_directory').perform(context)
    obstacle_slalom = scenario == 'slalom_obstacles'
    open_slalom = scenario == 'slalom_open'
    long_slalom = obstacle_slalom or open_slalom

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(friction_launch),
        launch_arguments={
            'headless': headless,
            'auto_grasp': 'true',
            'open_world': str(open_slalom).lower(),
            'enable_wrench_safety': enable_wrench_safety,
            'enable_lateral_admittance': str(
                long_slalom and enable_lateral_admittance).lower(),
            'admittance_force_sign': admittance_force_sign,
            # Long friction-grasp runs use the validated low-speed limit.  The
            # 0.060 m/s trial exceeded the vertical slip limit before obstacle 1.
            'max_linear_speed': (
                '0.025' if long_slalom else '0.035'),
            'max_linear_acceleration': (
                '0.025' if long_slalom else '0.040'),
            'measured_payload_position_weight': (
                '0.0' if long_slalom else '1.0'),
            'formation_alignment_kp': (
                '0.0'),
            'formation_alignment_max_fraction': (
                '0.5' if long_slalom else '1.0'),
            'formation_alignment_tolerance': (
                '0.015' if long_slalom else '0.008'),
            'formation_recovery_start': (
                '0.0'),
            'formation_recovery_stop': (
                '0.003' if obstacle_slalom else '0.0'),
            'formation_recovery_speed': (
                '0.0'),
            'horizontal_slip_limit': (
                '0.030'),
            'vertical_slip_limit': (
                '0.030'),
            'maximum_base_speed': (
                '0.200' if long_slalom else '0.120'),
            'base_speed_fault_duration': (
                '0.100' if long_slalom else '0.0'),
        }.items(),
    )
    evaluator = Node(
        package='cooperative_transport_control',
        executable='trajectory_evaluator',
        output='screen',
        parameters=[
            parameters,
            {
                'use_sim_time': True,
                'scenario': scenario,
                'auto_start': auto_start,
                'results_directory': results_directory,
                'require_support_removed': True,
                'enable_obstacle_evaluation': not open_slalom,
                'pass_through_waypoints': long_slalom,
                'pass_through_final_waypoint': long_slalom,
                'pass_through_position_tolerance': (
                    0.100 if long_slalom else 0.075),
                'pass_through_slip_tolerance': (
                    0.025 if long_slalom else 0.015),
            },
        ],
    )
    return [simulation, TimerAction(period=21.0, actions=[evaluator])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('scenario', default_value='turn_30'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('enable_wrench_safety', default_value='false'),
        DeclareLaunchArgument(
            'enable_lateral_admittance', default_value='true'),
        DeclareLaunchArgument('admittance_force_sign', default_value='1.0'),
        DeclareLaunchArgument(
            'results_directory',
            default_value='/tmp/cooperative_transport_results'),
        OpaqueFunction(function=_launch_setup),
    ])
