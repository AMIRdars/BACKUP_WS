"""Chapter 23: one exclusive follower, own observer and measured FT pipeline.

Start this instead of yaw_arm_base/yaw_whole_body. Only the existing leader
grasp-priority launch may authorize rotation through the qualified-ready Bool.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from amir_compliance_control.rotation_recentering import MODES
import yaml


def setup(context):
    value=lambda name:LaunchConfiguration(name).perform(context)
    folder=Path(value('run_directory')).resolve()
    mode=value('rotation_follow_mode')
    if mode not in MODES:raise ValueError('exclusive A/B/C/WHOLE_BODY mode required')
    process_on_measurement=value('process_wrench_on_measurement').lower()
    if process_on_measurement not in ('true','false'):raise ValueError('explicit Boolean wrench processing policy required')
    policy=yaml.safe_load(Path(value('follower_policy_file')).read_text())
    candidates=[v['ros__parameters'] for k,v in policy.items() if k in
                ('/follower/follower_yaw_arm_base','/follower/follower_yaw_whole_body')]
    if len(candidates)!=1:raise ValueError('one existing follower parameter namespace required')
    params=candidates[0]
    # These are the existing YawArmBase declared software envelopes, not
    # new hardware/noise assumptions. No qualification is supplied by default.
    observer_params=[{'orientation_envelope_rad':params.get('rotation_error_limit_rad',.01),
                      'joint_margin_rad':params.get('joint_margin_rad',.01)}]
    if value('observer_policy_file'):observer_params.append(value('observer_policy_file'))
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
    common={'use_sim_time':True,'urdf_file':str(folder/'follower.urdf'),
            'yaw_heading_frame':value('yaw_heading_frame')}
    return [
        Node(package='amir_compliance_control',executable='rotation_observer',namespace='follower',
             parameters=[*observer_params,common],additional_env=env,output='screen'),
        Node(package='amir_compliance_control',executable='follower_rotation_integrated',namespace='follower',
             parameters=[params,common,{'rotation_follow_mode':mode,'actuator_ack_required':True}],
             additional_env=env,output='screen'),
        Node(package='amir_compliance_control',executable='wrench_preprocessor',namespace='follower',
             parameters=[common,{'calibration_file':value('wrench_calibration_file'),
                 'process_on_measurement':process_on_measurement=='true'}],additional_env=env,output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name) for name in
        ('run_directory','follower_policy_file','wrench_calibration_file')]+
        [DeclareLaunchArgument('observer_policy_file',default_value=''),
         DeclareLaunchArgument('process_wrench_on_measurement',default_value='false'),
         DeclareLaunchArgument('yaw_heading_frame',default_value='tcp_link'),
         DeclareLaunchArgument('rotation_follow_mode',default_value='XY_ONLY'),OpaqueFunction(function=setup)])
