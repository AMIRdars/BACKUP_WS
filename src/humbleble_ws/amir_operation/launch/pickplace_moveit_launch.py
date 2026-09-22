# import os
# # from rclpy.parameter import Parameter
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument

# def generate_launch_description():
#     ld = LaunchDescription()

#     # kinematics_file = os.path.join(
#     #     os.path.dirname(__file__),
#     #     'kinematics.yaml'
#     # )
#     # kinematics_file = os.path.join(os.path.dirname(__file__), 'kinematics.yaml')

    
#     pickplace_node = Node(
#         package='amir_operation',
#         executable='pick_place_hum',
#         output='screen',
#         # parameters=[kinematics_file],
#     )

#     return LaunchDescription([
        
#         pickplace_node
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # MoveItのコンフィグ（URDF, SRDF, kinematics.yamlなど）を読み込む
    moveit_config = (
        MoveItConfigsBuilder("amir_mecanum3", package_name="amir_moveit_config")
        .robot_description(file_path="config/amir_mecanum3.urdf.xacro")
        .robot_description_semantic(file_path="config/amir_mecanum3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # C++ノードの起動設定
    pick_place_node = Node(
        package="amir_operation",  # ← ここを pick_place をビルドしたパッケージ名に変更してください
        executable="pick_place_hum_3",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, # ← これがIKソルバーの読み込みに必須！
        ],
    )

    return LaunchDescription([pick_place_node])