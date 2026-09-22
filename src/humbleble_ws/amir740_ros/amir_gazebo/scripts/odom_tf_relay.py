#!/usr/bin/env python3
"""
Relay /mecanum_drive_controller/tf_odometry → /tf.

ros2_controllers (Humble) publishes the odom→base_footprint transform
to a namespaced topic instead of /tf directly.
This node bridges that gap so the standard TF tree is complete.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class OdomTfRelay(Node):
    def __init__(self):
        super().__init__('odom_tf_relay')

        tf_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.create_subscription(
            TFMessage,
            '/mecanum_drive_controller/tf_odometry',
            lambda msg: self._pub.publish(msg),
            tf_qos,
        )
        self.get_logger().info(
            'odom_tf_relay: /mecanum_drive_controller/tf_odometry → /tf'
        )


def main():
    rclpy.init()
    node = OdomTfRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
