import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    use_sim_time = (
        LaunchConfiguration("use_sim_time")
        .perform(context)
        .lower()
        in ("true", "1", "yes")
    )

    namespace = LaunchConfiguration("namespace").perform(context)

    package_dir = get_package_share_directory("mecanum_navigation2")

    slam_params = os.path.join(
        package_dir,
        "config",
        "mapper_params_online_async.yaml",
    )

    if namespace:
        scan_topic = f"/{namespace}/scan"
    else:
        scan_topic = "scan"

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "scan_topic": scan_topic,
            },
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
        ],
    )

    return [slam_toolbox_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
