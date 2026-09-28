"""Rotate a grasped payload around robot A while robot B follows an arc."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _world_to_body(vx: float, vy: float, yaw: float):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


class PivotRotationController(Node):
    """Closed-loop specified-point rotation with amir1 as the pivot."""

    def __init__(self):
        super().__init__('pivot_rotation_controller')
        self.declare_parameter('pivot_namespace', 'amir1')
        self.declare_parameter('follower_namespace', 'amir2')
        self.declare_parameter('target_angle_deg', 90.0)
        self.declare_parameter('angular_velocity_deg_s', 1.0)
        self.declare_parameter('angular_acceleration_deg_s2', 0.25)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('position_kp', 0.8)
        self.declare_parameter('maximum_pivot_correction_speed', 0.02)
        self.declare_parameter('angle_tolerance_deg', 1.0)
        self.declare_parameter('wait_for_rotation_complete', False)
        self.declare_parameter('take_over_coordinator', False)

        self._pivot = str(self.get_parameter('pivot_namespace').value).strip('/')
        self._follower = str(
            self.get_parameter('follower_namespace').value).strip('/')
        self._target = math.radians(float(
            self.get_parameter('target_angle_deg').value))
        self._max_omega = math.radians(float(
            self.get_parameter('angular_velocity_deg_s').value))
        self._alpha = math.radians(float(
            self.get_parameter('angular_acceleration_deg_s2').value))
        self._angle_tolerance = math.radians(float(
            self.get_parameter('angle_tolerance_deg').value))
        self._poses = {}
        self._grasped = {self._pivot: False, self._follower: False}
        self._support_removed = False
        self._rotation_complete = False
        self._wait_for_rotation_complete = bool(
            self.get_parameter('wait_for_rotation_complete').value)
        self._take_over_coordinator = bool(
            self.get_parameter('take_over_coordinator').value)
        self._handoff_ready_at = None
        self._emergency = False
        self._started = False
        self._finished = False
        self._omega = 0.0
        self._initial_pivot = None
        self._initial_vector_angle = None
        self._target_yaw = {}

        self._command_publishers = {
            name: self.create_publisher(Twist, f'/{name}/rover_twist', 10)
            for name in (self._pivot, self._follower)}
        self._active_publisher = self.create_publisher(
            Bool, '/cooperative_transport/active', 10)
        self._handoff_publisher = self.create_publisher(
            Bool, '/cooperative_transport/external_control', 10)
        if self._wait_for_rotation_complete:
            completion_qos = QoSProfile(depth=1)
            completion_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                Bool, '/cooperative_rotation/complete',
                lambda message: setattr(
                    self, '_rotation_complete', bool(message.data)),
                completion_qos)
        self.create_subscription(
            Bool, '/cooperative_transport/emergency_stop',
            lambda message: setattr(self, '_emergency', bool(message.data)), 10)
        support_qos = QoSProfile(depth=1)
        support_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_transport/support_removed',
            lambda message: setattr(self, '_support_removed', bool(message.data)),
            support_qos)
        for name in (self._pivot, self._follower):
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp(robot, message), 10)
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odom(robot, message), 10)
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value),
            self._tick)
        self.get_logger().info(
            f'Waiting for grasp; pivot={self._pivot}, follower={self._follower}, '
            f'target={math.degrees(self._target):.1f} deg')

    def _on_grasp(self, robot, message):
        self._grasped[robot] = bool(message.data)

    def _on_odom(self, robot, message):
        pose = message.pose.pose
        self._poses[robot] = (
            pose.position.x, pose.position.y, _yaw(pose.orientation))

    def _publish_stop(self):
        for publisher in self._command_publishers.values():
            publisher.publish(Twist())
        self._active_publisher.publish(Bool(data=False))

    def _start_if_ready(self):
        if self._started:
            return
        if (self._emergency or not all(self._grasped.values())
                or not self._support_removed
                or (self._wait_for_rotation_complete
                    and not self._rotation_complete)
                or any(name not in self._poses for name in self._command_publishers)):
            return
        pivot = self._poses[self._pivot]
        follower = self._poses[self._follower]
        vector_x = follower[0] - pivot[0]
        vector_y = follower[1] - pivot[1]
        if math.hypot(vector_x, vector_y) < 0.1:
            return
        self._initial_pivot = pivot
        self._initial_vector_angle = math.atan2(vector_y, vector_x)
        self._target_yaw = {
            self._pivot: pivot[2] + self._target,
            self._follower: follower[2] + self._target,
        }
        self._started = True
        if self._take_over_coordinator:
            self._handoff_publisher.publish(Bool(data=True))
            self._handoff_ready_at = time.monotonic() + 0.5
        self._active_publisher.publish(Bool(data=True))
        self.get_logger().info('Pivot rotation started around amir1 (A)')

    def _tick(self):
        self._start_if_ready()
        # Do not publish zero velocities while the preceding trajectory still
        # owns the bases; that would race with the coordinator's commands.
        if not self._started:
            return
        if (self._handoff_ready_at is not None
                and time.monotonic() < self._handoff_ready_at):
            self._handoff_publisher.publish(Bool(data=True))
            return
        if self._finished or self._emergency:
            self._publish_stop()
            return
        if not all(self._grasped.values()):
            self.get_logger().error('Grasp lost; stopping pivot rotation')
            self._finished = True
            self._publish_stop()
            return

        pivot = self._poses[self._pivot]
        follower = self._poses[self._follower]
        vector_angle = math.atan2(
            follower[1] - pivot[1], follower[0] - pivot[0])
        rotated = _wrap(vector_angle - self._initial_vector_angle)
        error = _wrap(self._target - rotated)
        if abs(error) <= self._angle_tolerance:
            self._finished = True
            self._omega = 0.0
            self._publish_stop()
            self.get_logger().info('Pivot rotation complete; both robots stopped')
            return

        sign = 1.0 if error >= 0.0 else -1.0
        braking_omega = math.sqrt(2.0 * self._alpha * abs(error))
        desired = sign * min(self._max_omega, braking_omega)
        dt = 1.0 / float(self.get_parameter('control_rate').value)
        max_delta = self._alpha * dt
        self._omega += max(-max_delta, min(max_delta, desired - self._omega))

        # Keep A at its captured position while B follows the rigid-body arc.
        correction_limit = float(self.get_parameter(
            'maximum_pivot_correction_speed').value)
        correction_x = max(-correction_limit, min(
            correction_limit,
            (self._initial_pivot[0] - pivot[0]) * float(
                self.get_parameter('position_kp').value)))
        correction_y = max(-correction_limit, min(
            correction_limit,
            (self._initial_pivot[1] - pivot[1]) * float(
                self.get_parameter('position_kp').value)))
        follower_vx = correction_x - self._omega * (
            follower[1] - pivot[1])
        follower_vy = correction_y + self._omega * (
            follower[0] - pivot[0])
        pivot_vx, pivot_vy = _world_to_body(correction_x, correction_y, pivot[2])
        follower_vx, follower_vy = _world_to_body(
            follower_vx, follower_vy, follower[2])

        pivot_command = Twist()
        pivot_command.linear.x = pivot_vx
        pivot_command.linear.y = pivot_vy
        pivot_command.angular.z = self._omega
        follower_command = Twist()
        follower_command.linear.x = follower_vx
        follower_command.linear.y = follower_vy
        follower_command.angular.z = self._omega
        self._command_publishers[self._pivot].publish(pivot_command)
        self._command_publishers[self._follower].publish(follower_command)
        self._active_publisher.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = PivotRotationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
