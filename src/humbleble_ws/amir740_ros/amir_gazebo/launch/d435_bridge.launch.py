from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    D435 カメラの Ignition → ROS2 ブリッジ。
    gazebo_bringup.launch.py と並行して起動する:
      ros2 launch amir_gazebo d435_bridge.launch.py
    """

    # Ignition rgbd_camera は <topic>d435</topic> で以下を publish する:
    #   /d435/image          : カラー画像
    #   /d435/depth_image    : 深度画像
    #   /d435/points         : 点群 (PointCloud2)
    #   /d435/camera_info    : カメラ内部パラメータ
    d435_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="d435_bridge",
        arguments=[
            "/d435/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    return LaunchDescription([d435_bridge])
