"""CP-8R-1 replaces the leader executable; no scene, GT or follower controller."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    policy = Path(LaunchConfiguration('rotation_policy_file').perform(context)).resolve()
    return [Node(package='amir_compliance_control', executable='leader_grasp_priority', namespace='leader',
                 parameters=[str(policy), {'use_sim_time': True, 'urdf_file': str(folder/'leader.urdf'),
                                          'actuator_ack_required': True}],
                 additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'},
                 output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),
                              DeclareLaunchArgument('rotation_policy_file'), OpaqueFunction(function=setup)])
