"""CP-7 local transport modes; comparison and ground truth remain outside."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value=lambda name:LaunchConfiguration(name).perform(context)
    folder=Path(value('run_directory')).resolve();mode=value('mode')
    if mode not in ('ARM_BASE','ARM_ONLY'):raise ValueError('local autonomous mode required')
    return [Node(package='amir_compliance_control',executable='leader_trajectory_node',namespace='leader',
            parameters=[{'use_sim_time':True,'urdf_file':str(folder/'leader.urdf'),'speed_m_s':float(value('leader_speed')),
                'actuator_ack_required':True}],additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen'),
        Node(package='amir_compliance_control',executable='follower_arm_base',namespace='follower',
            parameters=[{'use_sim_time':True,'urdf_file':str(folder/'follower.urdf'),'monitor_pose':True,'actuator_ack_required':True,
                'command_delay_s':float(value('delay_s')),
                'base_gains':[.5,.5,1.] if mode=='ARM_BASE' else [0.,0.,0.]}],additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen'),
        Node(package='amir_compliance_control',executable='wrench_preprocessor',namespace='follower',
            parameters=[{'use_sim_time':True,'urdf_file':str(folder/'follower.urdf'),'calibration_file':value('calibration_file')}],additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),DeclareLaunchArgument('calibration_file'),
        DeclareLaunchArgument('mode',default_value='ARM_BASE'),DeclareLaunchArgument('delay_s',default_value='0.0'),
        DeclareLaunchArgument('leader_speed',default_value='.00005'),OpaqueFunction(function=setup)])
