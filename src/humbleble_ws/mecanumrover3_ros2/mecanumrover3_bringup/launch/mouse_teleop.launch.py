from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mouse_teleop_node = Node(
        package='mouse_teleop',
        executable='mouse_teleop',
        name='mouse_teleop',
        # Ignition: mecanum_drive_controller は /mecanum_drive_controller/cmd_vel を購読
        # gazebo_bringup の rover_twist_relay_ign が /rover_twist → /mecanum_drive_controller/cmd_vel を中継するため
        # /rover_twist に送るだけでよい
        remappings=[('/mouse_vel', '/rover_twist')]
    )

    return LaunchDescription([
        mouse_teleop_node
    ])