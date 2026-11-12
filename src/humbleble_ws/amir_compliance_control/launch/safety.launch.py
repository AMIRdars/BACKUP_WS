from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    return [Node(package='amir_compliance_control', executable='safety_supervisor',
                 parameters=[{'use_sim_time': True}], output='screen'),
            *[Node(package='amir_compliance_control', executable='safety_gate', namespace=robot,
                parameters=[{'use_sim_time': True, 'urdf_file': str(folder/(robot+'.urdf')),
                             'profile_file': str(folder/(robot+'_safety.json'))}], output='screen')
              for robot in ('leader', 'follower')]]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'), OpaqueFunction(function=setup)])
