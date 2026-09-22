"""pick / place / move_meca アクションサーバー群。

namespace 引数でロボット単位に起動できる (例: namespace:=amir1)。
各サーバは相対トピック/アクション名を使うため namespace 配下
(/amir1/pick, /amir1/gripper_controller/... 等) に解決される。
MoveGroupInterface もノードの namespace から /amir1/move_group を掴む。
namespace="" (default) なら従来どおり単体動作。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    pick_server_node = Node(
        package='amir_operation',
        executable='pick_server',
        namespace=namespace,
        output='screen',
    )

    move_meca_server_node = Node(
        package='amir_operation',
        executable='move_meca_server',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    place_server_node = Node(
        package='amir_operation',
        executable='place_server',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        pick_server_node,
        move_meca_server_node,
        place_server_node,
    ])
