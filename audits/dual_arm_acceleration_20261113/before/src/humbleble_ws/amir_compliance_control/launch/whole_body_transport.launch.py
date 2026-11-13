"""CP-8 opt-in; no simulation, evaluator, ground truth or TF dependency."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    folder = Path(value('run_directory')).resolve()
    mode = value('mode')
    if mode not in ('WHOLE_BODY', 'ARM_BASE'): raise ValueError('explicit local mode required')
    profile = value('yaw_profile')
    if profile not in ('initial', 'stiff', 'filtered'): raise ValueError('explicit virtual yaw profile required')
    yaw_scale = 100. if profile in ('stiff', 'filtered') else 1.
    translation = value('translation_profile')
    if translation not in ('baseline', 'metre_trial', 'speed_trial'):
        raise ValueError('explicit translation profile required')
    if translation in ('metre_trial', 'speed_trial') and mode != 'WHOLE_BODY':
        raise ValueError('translation trials require WHOLE_BODY')
    # Simulation control tuning, bounded by the existing measured 0.01 m/s
    # safety gate. Do not alter force, joint, grasp or watchdog limits.
    metre = {'base_gains': [12., 12., 1.], 'base_cutoff_hz': 8.,
             'base_velocity_limits': [.01, .01, .001],
             'base_acceleration_limits': [.02, .02, .002],
             'allocation_base_filter_hz': 8.} if translation == 'metre_trial' else {}
    # Experimental straight-line simulation profile. The guarded trial must
    # derive a separate gate envelope from passing ungrasped speed evidence.
    # Neither public launch defaults nor force/pose/watchdog limits change.
    if translation == 'speed_trial':
        # Slow own-force velocity memory replaces deflection PI integration.
        # The arm keeps its existing faster F/T mechanics; no peer state enters.
        metre = {'base_gains': [12., 12., 1.], 'base_integral_gains': [0., 0., 0.],
                 'base_force_acceleration_gains': [.05, 0., 0.], 'base_force_filter_hz': .1,
                 'base_stationary_hold': True,
                 'arm_command_horizon_s': .02,
                 'allocation_on_wrench': True,
                 'allocation_use_clock': True,
                 'base_cutoff_hz': 8., 'base_velocity_limits': [1.05, .01, .001],
                 'base_acceleration_limits': [.05, .02, .002], 'allocation_base_filter_hz': 8.,
                 'allocation_base_cost_velocity': [.01, .01, .001]}
    env = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
    return [Node(package='amir_compliance_control', executable='leader_planar_trajectory', namespace='leader',
        parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'leader.urdf'),
            'speed_m_s': float(value('leader_speed')),
            'max_distance_m': float(value('leader_max_distance_m')),
            'acceleration_m_s2': .01 if translation == 'speed_trial' else (.0005 if translation == 'metre_trial' else .001),
            'reference_acceleration_m_s2': .005 if translation == 'speed_trial' else (.00025 if translation == 'metre_trial' else 0.),
            'terminal_timeout_s': 10. if translation in ('metre_trial', 'speed_trial') else 3.,
            'actuator_ack_required': True}], additional_env=env, output='screen'),
        Node(package='amir_compliance_control', executable='follower_whole_body' if mode == 'WHOLE_BODY' else 'follower_arm_base',
            namespace='follower', parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'follower.urdf'),
                'actuator_ack_required': True, 'allow_ungrasped_test': value('allow_ungrasped_test') == 'true',
                **({'monitor_pose': True} if mode == 'ARM_BASE' else {
                    'virtual_yaw_inertia': .2*yaw_scale, 'virtual_yaw_damping': 12.*yaw_scale,
                    'virtual_yaw_stiffness': 4.*yaw_scale,
                    'allocation_base_filter_hz': .2 if profile=='filtered' else 0.}),
                **metre}], additional_env=env, output='screen'),
        Node(package='amir_compliance_control', executable='wrench_preprocessor', namespace='follower',
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'follower.urdf'),
                'calibration_file': value('calibration_file')}], additional_env=env, output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'), DeclareLaunchArgument('calibration_file'),
        DeclareLaunchArgument('mode', default_value='WHOLE_BODY'), DeclareLaunchArgument('leader_speed', default_value='.00005'),
        DeclareLaunchArgument('leader_max_distance_m', default_value='.001'),
        DeclareLaunchArgument('translation_profile', default_value='baseline', choices=['baseline', 'metre_trial', 'speed_trial']),
        DeclareLaunchArgument('allow_ungrasped_test', default_value='false'),
        DeclareLaunchArgument('yaw_profile'),
        OpaqueFunction(function=setup)])
