from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # start_static_tf_node = 
    #       Node(
    #           package = 'tf2_ros',
    #           executalble='static_transform_publisher',
    #           arguments=['0','1.5','0','3.14159','3.14159','0','base_link','horizontal_laser_link'],
    #         #   namespace='mecanum2',
    #         #   remapping=[('/tf_static','/mecanum2/tf_static')]
    #         ),
              
              
    return LaunchDescription([
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0.0', '0.0', '0.0', '0.0', '0', '0', 'base_footprint', 'base_link'],
        #     # namespace='amir',
        #     # remappings=[('/tf_static', '/amir/tf_static')],
        #     ),        
        
        Node(
            package='my_utility',
            executable='odom_tf2_broadcaster',
            name='odom_tf2_broadcaster',
            output='screen',
            emulate_tty = True,
            # namespace='amir',
            # remappings=[('/tf', '/amir/tf')],
            ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'baselink']),    
        
        # Node(
        #     package = 'tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['-1.0','0.0','0.0','-0.05','0','0','map','ar_marker_0'], #2.0
        #     # namespace='mecanum2',
        #     # remappings=[('/tf_static','/mecanum2/tf_static')]
        #     ),
              
        # add_action(start_static_tf_node)       
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.0', '0.0', '0.0', '0.0', '0', '0', 'base_footprint', 'base_link'],
            # namespace='amir',
            # remappings=[('/tf_static', '/amir/tf_static')],
            ),        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.2', '0.0', '0.08', '3.14159', '3.14159', '0', 'base_link', 'horizontal_laser_link'],
            # namespace='amir',
            # remappings=[('/tf_static', '/amir/tf_static')],
            ),

        


        # Node(
        #     package = 'tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.2','0.0','0.22','0','0','0','base_link','camera_color_optical_frame'],
        #     # namespace='mecanum2',
        #     # remappings=[('/tf_static','/mecanum2/tf_static')]
        #     ),
        Node(
        package = 'tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0','0.0','0.0','0','0','0','base_link','camera_color_optical_frame'],
        # namespace='mecanum2',
        # remappings=[('/tf_static','/mecanum2/tf_static')]
        ),
        
    ])
