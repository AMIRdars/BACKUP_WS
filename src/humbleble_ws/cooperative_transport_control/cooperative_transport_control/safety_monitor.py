"""Fail-safe monitor that stops both robots together."""

import math
import time
from typing import Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


_JOINT_LIMITS = {
    'Joint_1': (-2.96706, 2.96706),
    'Joint_2': (0.0, 2.356194),
    'Joint_3': (-2.792527, 0.0),
    'Joint_4': (-2.094395, 1.308997),
    'Joint_5': (-2.75762, 2.75762),
}


class SafetyMonitor(Node):
    """Latch an emergency stop when a cooperative invariant is violated."""

    def __init__(self) -> None:
        super().__init__('safety_monitor')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('monitor_rate', 50.0)
        self.declare_parameter('odometry_timeout', 0.30)
        self.declare_parameter('minimum_robot_separation', 2.30)
        self.declare_parameter('maximum_robot_separation', 3.10)
        self.declare_parameter('maximum_base_speed', 0.20)
        self.declare_parameter('base_speed_fault_duration', 0.0)
        self.declare_parameter('joint_limit_margin', math.radians(5.0))
        self.declare_parameter('joint_hard_limit_tolerance', 1.0e-3)
        self.declare_parameter('require_attached', True)
        self.declare_parameter(
            'external_fault_topic', '/cooperative_transport/slip_fault')
        self.declare_parameter(
            'wrench_fault_topic', '/cooperative_transport/wrench_fault')

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._positions = {}
        self._speeds = {}
        self._speed_exceeded_since: Dict[str, float] = {}
        self._odom_times = {}
        self._joint_positions: Dict[str, Dict[str, float]] = {}
        self._attached: Dict[str, Optional[bool]] = {name: None for name in self._names}
        self._active = False
        self._external_fault = False
        self._wrench_fault = False
        self._latched = False
        self._reason = 'OK'

        for name in self._names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message), 10)
            self.create_subscription(
                JointState, f'/{name}/joint_states',
                lambda message, robot=name: self._on_joint_state(robot, message), 10)
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp_state(robot, message), 10)
        self.create_subscription(
            Bool, '/cooperative_transport/active', self._on_active, 10)
        self.create_subscription(
            Bool, self.get_parameter('external_fault_topic').value,
            self._on_external_fault, 10)
        self.create_subscription(
            Bool, self.get_parameter('wrench_fault_topic').value,
            lambda message: setattr(self, '_wrench_fault', message.data), 10)

        self._stop_publisher = self.create_publisher(
            Bool, '/cooperative_transport/emergency_stop', 10)
        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/safety_status', 10)
        self.create_service(
            Trigger, '/cooperative_transport/reset_safety', self._on_reset)
        rate = float(self.get_parameter('monitor_rate').value)
        self.create_timer(1.0 / rate, self._monitor_tick)
        self.get_logger().info('Safety monitor ready; emergency stop affects both robots.')

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        self._positions[robot] = (
            message.pose.pose.position.x, message.pose.pose.position.y)
        twist = message.twist.twist
        self._speeds[robot] = math.hypot(twist.linear.x, twist.linear.y)
        self._odom_times[robot] = time.monotonic()

    def _on_joint_state(self, robot: str, message: JointState) -> None:
        self._joint_positions[robot] = dict(zip(message.name, message.position))

    def _on_grasp_state(self, robot: str, message: Bool) -> None:
        self._attached[robot] = message.data

    def _on_active(self, message: Bool) -> None:
        self._active = message.data

    def _on_external_fault(self, message: Bool) -> None:
        self._external_fault = message.data

    def _violation(self) -> Optional[str]:
        if not self._active:
            self._speed_exceeded_since.clear()
            return None
        if self._external_fault:
            return 'external slip/contact monitor fault'
        if self._wrench_fault:
            return 'wrist force/torque threshold exceeded'
        if len(self._positions) != 2:
            return 'missing odometry'
        timeout = float(self.get_parameter('odometry_timeout').value)
        now = time.monotonic()
        for name in self._names:
            if now - self._odom_times.get(name, 0.0) > timeout:
                return f'{name} odometry timeout'

        p1 = self._positions[self._names[0]]
        p2 = self._positions[self._names[1]]
        separation = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        minimum = float(self.get_parameter('minimum_robot_separation').value)
        maximum = float(self.get_parameter('maximum_robot_separation').value)
        if not minimum <= separation <= maximum:
            return f'robot separation {separation:.3f} m outside [{minimum:.3f}, {maximum:.3f}]'

        speed_limit = float(self.get_parameter('maximum_base_speed').value)
        speed_duration = max(0.0, float(
            self.get_parameter('base_speed_fault_duration').value))
        for name, speed in self._speeds.items():
            if speed > speed_limit:
                exceeded_since = self._speed_exceeded_since.setdefault(name, now)
                if now - exceeded_since >= speed_duration:
                    return (
                        f'{name} base speed {speed:.3f} m/s exceeds '
                        f'{speed_limit:.3f} for {speed_duration:.2f} s')
            else:
                self._speed_exceeded_since.pop(name, None)

        if bool(self.get_parameter('require_attached').value):
            for name, attached in self._attached.items():
                if attached is not True:
                    return f'{name} grasp is not attached'

        margin = float(self.get_parameter('joint_limit_margin').value)
        hard_tolerance = float(
            self.get_parameter('joint_hard_limit_tolerance').value)
        for robot, positions in self._joint_positions.items():
            for joint, (lower, upper) in _JOINT_LIMITS.items():
                if joint not in positions:
                    continue
                position = positions[joint]
                if margin > 0.0:
                    if position - lower < margin or upper - position < margin:
                        return (
                            f'{robot}/{joint} is within '
                            f'{math.degrees(margin):.1f} deg of a limit')
                elif (position < lower - hard_tolerance
                      or position > upper + hard_tolerance):
                    return f'{robot}/{joint} is outside its hard limit'
        return None

    def _monitor_tick(self) -> None:
        violation = self._violation()
        if violation is not None and not self._latched:
            self._latched = True
            self._reason = violation
            self.get_logger().error(f'Safety stop latched: {violation}')
        stop = Bool()
        stop.data = self._latched
        self._stop_publisher.publish(stop)
        status = String()
        status.data = f'STOPPED: {self._reason}' if self._latched else 'OK'
        self._status_publisher.publish(status)

    def _on_reset(self, _request: Trigger.Request,
                  response: Trigger.Response) -> Trigger.Response:
        violation = self._violation()
        if violation is not None:
            response.success = False
            response.message = f'Cannot reset: {violation}'
            return response
        self._latched = False
        self._reason = 'OK'
        response.success = True
        response.message = 'Safety stop reset.'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
