from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
              
    return LaunchDescription([
  
        
        Node(
            package='my_utility',
            executable='camera_to_amirbase',
            name='camera_to_amirbase',
            output='screen',
            emulate_tty = True,
            # namespace='amir',
            # remappings=[('/tf', '/amir/tf')],
            ),
        
    ])
