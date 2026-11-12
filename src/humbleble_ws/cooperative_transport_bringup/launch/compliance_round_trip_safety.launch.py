"""CP-13 shared stop and original interlocks, with exact timestamp gates."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder=Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    root=Path(__file__).resolve().parents[4]
    script=root/'tools/round_trip_safety_gate.py'
    if not script.is_file():
        raise ValueError('round-trip safety script missing: '+str(script))
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
    actions=[Node(package='amir_compliance_control',executable='safety_supervisor',
        parameters=[{'use_sim_time':True,'require_interlocks':True}],additional_env=env,output='screen')]
    for robot in ('leader','follower'):
        actions += [ExecuteProcess(name='round_trip_'+robot+'_safety_gate',
            cmd=['/usr/bin/python3',str(script),'--robot',robot,'--run-directory',str(folder)],
            additional_env=env,output='screen'),
            Node(package='amir_compliance_control',executable='actuator_interlock',namespace=robot,
                parameters=[{'use_sim_time':True,'urdf_file':str(folder/(robot+'.urdf')),
                    'profile_file':str(folder/(robot+'_safety.json'))}],additional_env=env,output='screen'),
            Node(package='amir_compliance_control',executable='controller_audit',namespace=robot,
                additional_env=env,output='screen')]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),OpaqueFunction(function=setup)])
