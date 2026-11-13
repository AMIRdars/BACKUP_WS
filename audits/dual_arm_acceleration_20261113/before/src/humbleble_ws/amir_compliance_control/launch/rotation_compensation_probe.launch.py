"""O2-only replacement for the integrated follower; never launch both."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    v=lambda name:LaunchConfiguration(name).perform(context)
    common={'use_sim_time':True,'urdf_file':str(Path(v('run_directory')).resolve()/'follower.urdf'),
            'yaw_heading_frame':'grasp_heading'}
    env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
    return [Node(package='amir_compliance_control',executable='rotation_observer',namespace='follower',
                 parameters=[v('observer_policy_file'),common],additional_env=env,output='screen'),
        Node(package='amir_compliance_control',executable='follower_compensation_probe',namespace='follower',
             parameters=[v('follower_policy_file'),common,{'rotation_follow_mode':'XY_YAW_COMP','actuator_ack_required':True}],
             additional_env=env,output='screen'),
        Node(package='amir_compliance_control',executable='wrench_preprocessor',namespace='follower',
             parameters=[common,{'calibration_file':v('wrench_calibration_file'),'process_on_measurement':True}],additional_env=env,output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n) for n in ('run_directory','observer_policy_file',
        'follower_policy_file','wrench_calibration_file')]+[OpaqueFunction(function=setup)])
