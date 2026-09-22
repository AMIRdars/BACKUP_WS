#!/usr/bin/env python3
"""
/rover_twist (geometry_msgs/Twist) を
/mecanum_drive_controller/reference_unstamped へそのまま転送するリレーノード。

mecanum_drive_controller v2.x は ~/reference (TwistStamped) と
~/reference_unstamped (Twist) を購読する。~/cmd_vel は存在しない。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RoverTwistRelayIgn(Node):
    def __init__(self):
        super().__init__('rover_twist_relay_ign')
        # mecanum_drive_controller v2.x uses ~/reference_unstamped (Twist), not ~/cmd_vel
        # 相対トピック名にして launch の namespace で prefix されるようにする
        # (namespace 空なら従来どおり /rover_twist, /mecanum_drive_controller/... で解決)
        self._pub = self.create_publisher(Twist, 'mecanum_drive_controller/reference_unstamped', 10)
        self._sub = self.create_subscription(
            Twist, 'rover_twist',
            lambda msg: self._pub.publish(msg),
            10,
        )
        self.get_logger().info('rover_twist_relay_ign: /rover_twist → /mecanum_drive_controller/reference_unstamped')


def main():
    rclpy.init()
    rclpy.spin(RoverTwistRelayIgn())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
