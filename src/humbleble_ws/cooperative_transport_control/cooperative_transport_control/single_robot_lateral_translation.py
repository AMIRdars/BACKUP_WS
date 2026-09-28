"""Move one mecanum robot laterally without any grasp or payload logic."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SingleRobotLateralTranslation(Node):
    """Closed-loop one-robot right translation in the initial world frame."""

    def __init__(self):
        super().__init__('single_robot_lateral_translation')
        self.declare_parameter('robot_namespace', 'amir1')
        self.declare_parameter('distance_m', 0.5)
        self.declare_parameter('speed_m_s', 0.05)
        self.declare_parameter('acceleration_m_s2', 0.025)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('position_tolerance_m', 0.01)

        self._robot = str(self.get_parameter('robot_namespace').value).strip('/')
        self._distance = float(self.get_parameter('distance_m').value)
        self._speed = abs(float(self.get_parameter('speed_m_s').value))
        self._acceleration = abs(float(
            self.get_parameter('acceleration_m_s2').value))
        self._tolerance = abs(float(
            self.get_parameter('position_tolerance_m').value))
        if self._distance < 0.0:
            raise ValueError('distance_m must be non-negative')
        if self._speed <= 0.0 or self._acceleration <= 0.0:
            raise ValueError('speed_m_s and acceleration_m_s2 must be positive')

        self._pose = None
        self._start = None
        self._target = None
        self._speed_command = 0.0
        self._finished = False
        self._publisher = self.create_publisher(
            Twist, f'/{self._robot}/rover_twist', 10)
        self.create_subscription(
            Odometry, f'/{self._robot}/odom', self._on_odom, 10)
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value),
            self._tick)
        self.get_logger().info(
            f'Waiting for /{self._robot}/odom; right translation '
            f'target={self._distance:.3f} m')

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2
                         + pose.orientation.z ** 2),
        )
        self._pose = (pose.position.x, pose.position.y, yaw)
        if self._start is None:
            self._start = self._pose
            # Robot-local right is local -Y, expressed in the initial world
            # frame. Keeping this direction fixed gives a straight lateral
            # trajectory even if the robot has a small heading disturbance.
            right_x = math.sin(yaw)
            right_y = -math.cos(yaw)
            self._target = (
                pose.position.x + right_x * self._distance,
                pose.position.y + right_y * self._distance,
                yaw,
            )
            self.get_logger().info(
                f'Start pose captured; moving right {self._distance:.3f} m')

    def _publish_stop(self) -> None:
        self._publisher.publish(Twist())

    def _tick(self) -> None:
        if self._pose is None or self._start is None or self._target is None:
            self._publish_stop()
            return
        if self._finished:
            self._publish_stop()
            return

        x, y, yaw = self._pose
        target_x, target_y, target_yaw = self._target
        error_x = target_x - x
        error_y = target_y - y
        distance_error = math.hypot(error_x, error_y)
        if distance_error <= self._tolerance:
            self._finished = True
            self._publish_stop()
            self.get_logger().info('Right translation complete; robot stopped')
            return

        # Trapezoidal speed profile with braking distance, projected onto the
        # fixed initial right direction.
        right_x = math.sin(self._start[2])
        right_y = -math.cos(self._start[2])
        progress_error = max(0.0, error_x * right_x + error_y * right_y)
        braking_speed = math.sqrt(2.0 * self._acceleration * progress_error)
        desired_speed = min(self._speed, braking_speed)
        dt = 1.0 / float(self.get_parameter('control_rate').value)
        delta = self._acceleration * dt
        self._speed_command += max(-delta, min(delta, desired_speed - self._speed_command))

        world_vx = right_x * self._speed_command
        world_vy = right_y * self._speed_command
        c = math.cos(yaw)
        s = math.sin(yaw)
        command = Twist()
        command.linear.x = c * world_vx + s * world_vy
        command.linear.y = -s * world_vx + c * world_vy
        command.angular.z = max(-0.15, min(0.15, 1.0 * _wrap(target_yaw - yaw)))
        self._publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotLateralTranslation()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()
