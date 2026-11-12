"""Opt-in leader-local retiming. No follower state, angle or centre exchange."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    return [Node(package='amir_compliance_control', executable='leader_grasp_priority', namespace='leader',
        parameters=[value('rotation_policy_file'), {'use_sim_time': True,
            'urdf_file': str(Path(value('run_directory')).resolve()/'leader.urdf'), 'actuator_ack_required': True}],
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}, output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'), DeclareLaunchArgument('rotation_policy_file'),
                             OpaqueFunction(function=setup)])
