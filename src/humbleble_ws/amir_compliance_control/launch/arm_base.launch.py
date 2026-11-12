"""CP-6 local control only; no experiment or ground-truth dependencies."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    test = LaunchConfiguration('ungrasped_test').perform(context).lower() == 'true'
    return [Node(package='amir_compliance_control', executable='leader_trajectory_node', namespace='leader',
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'leader.urdf'),
                'speed_m_s': float(LaunchConfiguration('leader_speed').perform(context))}], output='screen'),
        Node(package='amir_compliance_control', executable='follower_arm_base', namespace='follower',
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'follower.urdf'),
                'allow_ungrasped_test': test, 'preserve_orientation': not test, 'straight_only': not test,
                'base_gains': [float(v) for v in LaunchConfiguration('base_gains').perform(context).split(',')]}], output='screen'),
        Node(package='amir_compliance_control', executable='wrench_preprocessor', namespace='follower',
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'follower.urdf'),
                'calibration_file': LaunchConfiguration('calibration_file')}], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'), DeclareLaunchArgument('calibration_file'),
        DeclareLaunchArgument('ungrasped_test', default_value='false'), DeclareLaunchArgument('leader_speed', default_value='.00005'),
        DeclareLaunchArgument('base_gains', default_value='0.5,0.5,1.0'), OpaqueFunction(function=setup)])
