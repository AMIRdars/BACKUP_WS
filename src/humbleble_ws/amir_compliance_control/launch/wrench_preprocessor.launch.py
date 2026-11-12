from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='follower'),
        DeclareLaunchArgument('urdf_file'), DeclareLaunchArgument('calibration_file'),
        Node(package='amir_compliance_control', executable='wrench_preprocessor',
             namespace=LaunchConfiguration('robot'), output='screen',
             parameters=[{'use_sim_time': True, 'urdf_file': LaunchConfiguration('urdf_file'),
                          'calibration_file': LaunchConfiguration('calibration_file')}])])
