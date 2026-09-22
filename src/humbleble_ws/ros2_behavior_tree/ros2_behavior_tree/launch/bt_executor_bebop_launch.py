import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="bebop", description="Top-level namespace"
    )


    bt_nodes = os.path.join(
        get_package_share_directory('ros2_behavior_tree'),
        'config',
        'bt_nodes.yaml'
    )
    bt_executor_node = Node(
        package='ros2_behavior_tree',
        executable='bt_executor',
        # name='bt_executor',   #   複数のBTを実行するので設定しない
        parameters = [bt_nodes],
        namespace=namespace,
        remappings=remappings,
        # parameters=[{'robot_name': 'mecanum1'}],
        output='screen',
        emulate_tty=True  #これがないと表示されない。foxy以降はemulate_tty=True
    )

    ld.add_action(declare_namespace_cmd)
    ld.add_action(bt_executor_node)
    return ld