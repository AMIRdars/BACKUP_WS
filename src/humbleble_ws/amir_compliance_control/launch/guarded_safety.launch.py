"""CP-7 final output ownership and independent controller deactivation path."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder=Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    arm_only=LaunchConfiguration('follower_arm_only').perform(context)=='true'
    both=LaunchConfiguration('both_bases_zero').perform(context)=='true'
    coherent=LaunchConfiguration('coherent_sensor_history').perform(context)=='true'
    source_watchdog=LaunchConfiguration('simulation_source_watchdog').perform(context)=='true'
    actions=[Node(package='amir_compliance_control',executable='safety_supervisor',
        parameters=[{'use_sim_time':True,'require_interlocks':True}],additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen')]
    for robot in ('leader','follower'):
        actions += [Node(package='amir_compliance_control',executable='safety_gate',namespace=robot,
            parameters=[{'use_sim_time':True,'urdf_file':str(folder/(robot+'.urdf')),
                         'profile_file':str(folder/(robot+'_safety.json')),
                         'coherent_sensor_history':coherent,
                         'simulation_source_watchdog':source_watchdog,
                         'base_zero_only':both or arm_only and robot=='follower'}],
            remappings=[('rover_twist','interlock/base_request'),
                        ('arm_controller/joint_trajectory','interlock/arm_request')],additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen'),
            Node(package='amir_compliance_control',executable='actuator_interlock',namespace=robot,
                 parameters=[{'use_sim_time':True,'urdf_file':str(folder/(robot+'.urdf')),
                              'profile_file':str(folder/(robot+'_safety.json')),
                              'simulation_source_watchdog':source_watchdog,
                              'base_zero_only':both or arm_only and robot=='follower'}],additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen'),
            Node(package='amir_compliance_control',executable='controller_audit',namespace=robot,
                 parameters=[{'request_writer_name':('leader_trajectory_node' if robot=='leader' else 'follower_arm_only') if both else 'follower_arm_only' if arm_only and robot=='follower' else '',
                              'require_gripper_active':arm_only}],
                 additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'},output='screen')]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),
        DeclareLaunchArgument('coherent_sensor_history',default_value='false',choices=['false','true']),
        DeclareLaunchArgument('simulation_source_watchdog',default_value='false',choices=['false','true']),
        DeclareLaunchArgument('both_bases_zero',default_value='false',choices=['false','true']),
        DeclareLaunchArgument('follower_arm_only',default_value='false',choices=['false','true']),OpaqueFunction(function=setup)])
