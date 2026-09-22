#!/usr/bin/env python3
"""Relay /cmd_vel (Nav2 velocity_smoother output) → /rover_twist (mecanum relay input).

In nav2_bringup navigation_launch.py the velocity_smoother is remapped
(cmd_vel_smoothed → cmd_vel, cmd_vel → cmd_vel_nav), so /cmd_vel is ALREADY the
smoothed output and the raw controller output is on /cmd_vel_nav. Subscribe to /cmd_vel
here so the smoothed (ramped-acceleration) command reaches the wheels.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        # 相対名: namespace 配下で /<ns>/cmd_vel → /<ns>/rover_twist に解決
        # (namespace 無しなら従来どおり /cmd_vel → /rover_twist)
        self._pub = self.create_publisher(Twist, 'rover_twist', 10)
        self.create_subscription(Twist, 'cmd_vel', self._pub.publish, 10)
        namespace = self.get_namespace().rstrip('/')
        prefix = namespace if namespace else ''
        self.get_logger().info(
            f'cmd_vel_relay: {prefix}/cmd_vel -> {prefix}/rover_twist')


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
