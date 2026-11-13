"""CP-4 only: leader goals, follower local hold. No simulator or evaluator."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    return [Node(package='amir_compliance_control', executable=exe, namespace=robot,
        parameters=[{'use_sim_time': True, 'urdf_file': str(folder/(robot+'.urdf')),
            **({key: float(LaunchConfiguration(key).perform(context)) for key in
               ('max_distance_m', 'speed_m_s', 'acceleration_m_s2')} if robot == 'leader' else {})}], output='screen')
        for robot, exe in [('leader', 'leader_trajectory_node'), ('follower', 'follower_hold')]]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),
        DeclareLaunchArgument('max_distance_m', default_value='.001'),
        DeclareLaunchArgument('speed_m_s', default_value='.00025'),
        DeclareLaunchArgument('acceleration_m_s2', default_value='.001'), OpaqueFunction(function=setup)])
