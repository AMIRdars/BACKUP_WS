"""CP-5 control only: calibrated local wrench, stationary bases, follower arm."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    return [Node(package='amir_compliance_control', executable=exe, namespace=robot,
        parameters=[{'use_sim_time': True, 'urdf_file': str(folder/(robot+'.urdf'))}], output='screen')
        for robot, exe in [('leader', 'leader_trajectory_node'), ('follower', 'follower_arm_admittance')]] + [
        Node(package='amir_compliance_control', executable='wrench_preprocessor', namespace='follower',
            parameters=[{'use_sim_time': True, 'urdf_file': str(folder/'follower.urdf'),
                'calibration_file': LaunchConfiguration('calibration_file')}], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'), DeclareLaunchArgument('calibration_file'),
                              OpaqueFunction(function=setup)])
