"""Optional ground-truth output. Nothing here publishes robot commands."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='compliance_evaluation_bridge', parameters=[{'use_sim_time': True}],
        arguments=[f'/experiment/{robot}/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
                   for robot in ('leader', 'follower')])])
