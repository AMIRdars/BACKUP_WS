"""Chapter 24 control only; no recentering, allocator or rotation node."""
from pathlib import Path
import json
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    if value('control_mode') != 'ARM_ONLY' or value('rotation_tracking_enabled') != 'false':
        raise ValueError('AF-0..3 requires ARM_ONLY and rotation_tracking_enabled=false')
    if context.launch_configurations.get('enable_base_assist','false') != 'false':
        raise ValueError('Phase 1 requires enable_base_assist=false')
    folder = Path(value('run_directory')).resolve()
    policy = json.loads(Path(value('experiment_policy')).read_text())
    if (policy['controller_parameters'].get('enable_base_assist',False) is not False
            or policy['controller_parameters'].get('wrench_source_mode','sensor') != 'sensor'):
        raise ValueError('Phase 1 policy requires enable_base_assist=false and wrench_source_mode=sensor')
    native=policy.get('native_arm_stroke') is True
    dual=policy.get('dual_arm_only') is True
    allowed_scopes={'simulation_micro_test_only_not_hardware_ratings'}
    if native:allowed_scopes.add('simulation_native_stroke_only_not_hardware_ratings')
    if policy.get('scope') not in allowed_scopes:
        raise ValueError('explicit simulation experiment envelope required')
    speed, distance = float(value('leader_speed')), float(value('leader_distance_limit'))
    if not (math.isfinite(speed) and 0 < speed <= (.005 if native else .01)
            and math.isfinite(distance) and 0 < distance <= (.5 if native else .01)):
        raise ValueError('AF leader envelope: <=10 mm and <=0.01 m/s')
    distance_scale=float(policy.get('leader_distance_metres_per_raw_odom_metre',1.)) if native else 1.
    if not math.isfinite(distance_scale) or distance_scale<=0:raise ValueError('invalid calibrated distance scale')
    env = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
    return [Node(package='amir_compliance_control', executable='leader_arm_only' if dual else 'leader_arm_compensation' if native else 'leader_trajectory_node', namespace='leader',
        parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'leader.urdf'),
                     'speed_m_s': speed*distance_scale, 'max_distance_m': distance,
                     'simulation_source_watchdog':native,
                     'acceleration_m_s2': float(policy.get('leader_acceleration_m_s2', .001))*distance_scale,
                     'straight_reference_acceleration_m_s2': float(policy.get('leader_reference_acceleration_m_s2', 0.))*distance_scale,
                     'goal_tolerance_m': float(policy.get('leader_goal_tolerance_m', .00001)),
                     'actuator_ack_required': True,
                     **({'arm_acceleration_m_s2':.0012,'arm_hold_after_s':2.} if dual else {}),
                     **({'arm_servo_response_file':policy['leader_servo_response_file'],
                         **({} if dual else {'base_distance_calibration_file':policy.get('leader_base_distance_calibration_file','')})} if native else {})}], additional_env=env, output='screen'),
        Node(package='amir_compliance_control', executable='follower_arm_only', namespace='follower',
            parameters=[{**policy['controller_parameters'], 'use_sim_time': True,
                'urdf_file': str(folder/'follower.urdf'), 'control_mode': 'ARM_ONLY',
                'rotation_tracking_enabled': False, 'simulation_micro_test': True,
                'enable_base_assist': False, 'wrench_source_mode': 'sensor',
                'actuator_ack_required': True, 'allow_ungrasped_test': value('ungrasped_test') == 'true'}],
            additional_env=env, output='screen'),
        Node(package='amir_compliance_control', executable='wrench_preprocessor', namespace='follower',
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'follower.urdf'),
                'calibration_file': value('calibration_file'),
                'cutoff_hz': float(policy.get('wrench_cutoff_hz',8.)),
                'simulation_source_watchdog':native,
                'process_on_measurement': policy.get('wrench_process_on_measurement',False)}], additional_env=env, output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'), DeclareLaunchArgument('calibration_file'),
        DeclareLaunchArgument('experiment_policy'), DeclareLaunchArgument('control_mode', default_value='ARM_ONLY'),
        DeclareLaunchArgument('rotation_tracking_enabled', default_value='false'),
        DeclareLaunchArgument('enable_base_assist', default_value='false'),
        DeclareLaunchArgument('ungrasped_test', default_value='false'),
        DeclareLaunchArgument('leader_speed', default_value='.005'),
        DeclareLaunchArgument('leader_distance_limit', default_value='.005'), OpaqueFunction(function=setup)])
