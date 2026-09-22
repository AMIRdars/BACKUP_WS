"""Monitor both wrist force/torque sensors and request synchronized stop."""

import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .wrench_metrics import RunningWrenchMetrics, WrenchVector, stop_reason


class WrenchMonitor(Node):
    """Publish wrench statistics and a debounced force-threshold fault."""

    def __init__(self) -> None:
        super().__init__('wrench_monitor')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('monitor_rate', 50.0)
        self.declare_parameter('sensor_timeout', 0.25)
        self.declare_parameter('planar_force_warning', 30.0)
        self.declare_parameter('planar_force_stop', 50.0)
        self.declare_parameter('vertical_force_stop', 100.0)
        self.declare_parameter('torque_stop', 15.0)
        self.declare_parameter('fault_duration', 0.10)
        self.declare_parameter('enable_threshold_stop', False)

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._wrenches: Dict[str, WrenchVector] = {}
        self._receipt_times: Dict[str, float] = {}
        self._statistics = {
            name: RunningWrenchMetrics() for name in self._names
        }
        self._active = False
        self._exceeded_since: Optional[float] = None
        self._fault = False
        self._reason = 'inactive'

        for name in self._names:
            self.create_subscription(
                WrenchStamped, f'/{name}/ft_sensor',
                lambda message, robot=name: self._on_wrench(robot, message), 10)
        self.create_subscription(
            Bool, '/cooperative_transport/active', self._on_active, 10)
        self._fault_publisher = self.create_publisher(
            Bool, '/cooperative_transport/wrench_fault', 10)
        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/wrench_status', 10)
        rate = float(self.get_parameter('monitor_rate').value)
        self.create_timer(1.0 / rate, self._tick)
        threshold_mode = bool(self.get_parameter(
            'enable_threshold_stop').value)
        self.get_logger().info(
            'Dual wrist wrench monitor ready; mode='
            f'{"threshold stop" if threshold_mode else "measurement only"}.')

    def _on_wrench(self, robot: str, message: WrenchStamped) -> None:
        wrench = message.wrench
        sample = WrenchVector(
            wrench.force.x, wrench.force.y, wrench.force.z,
            wrench.torque.x, wrench.torque.y, wrench.torque.z)
        self._wrenches[robot] = sample
        self._receipt_times[robot] = time.monotonic()
        if self._active:
            self._statistics[robot].update(sample)

    def _on_active(self, message: Bool) -> None:
        if message.data and not self._active:
            for statistics in self._statistics.values():
                statistics.reset()
            self._exceeded_since = None
            self._fault = False
        self._active = message.data
        if not self._active:
            self._exceeded_since = None
            self._fault = False

    def _current_violation(self, now: float) -> Optional[str]:
        if (not self._active
                or not bool(self.get_parameter(
                    'enable_threshold_stop').value)):
            return None
        timeout = float(self.get_parameter('sensor_timeout').value)
        for name in self._names:
            age = now - self._receipt_times.get(name, 0.0)
            if age > timeout:
                return f'{name} F/T sensor timeout ({age:.3f} s)'
        planar = float(self.get_parameter('planar_force_stop').value)
        vertical = float(self.get_parameter('vertical_force_stop').value)
        torque = float(self.get_parameter('torque_stop').value)
        for name in self._names:
            reason = stop_reason(
                self._wrenches[name], planar, vertical, torque)
            if reason is not None:
                return f'{name} {reason}'
        return None

    def _tick(self) -> None:
        now = time.monotonic()
        violation = self._current_violation(now)
        if violation is None:
            self._exceeded_since = None
            self._fault = False
            self._reason = 'OK' if self._active else 'inactive'
        else:
            if self._exceeded_since is None:
                self._exceeded_since = now
            duration = float(self.get_parameter('fault_duration').value)
            self._fault = now - self._exceeded_since >= duration
            self._reason = violation
            if self._fault:
                self.get_logger().error(
                    f'Wrench safety threshold exceeded: {violation}',
                    throttle_duration_sec=1.0)

        fault = Bool()
        fault.data = self._fault
        self._fault_publisher.publish(fault)
        status = String()
        status.data = self._status_text()
        self._status_publisher.publish(status)

    def _status_text(self) -> str:
        fields = [
            ('FAULT' if self._fault else
             'MONITORING' if self._active and not bool(self.get_parameter(
                 'enable_threshold_stop').value) else
             'OK' if self._active else 'INACTIVE'),
            self._reason,
        ]
        warning = float(self.get_parameter('planar_force_warning').value)
        for name in self._names:
            wrench = self._wrenches.get(name)
            if wrench is None:
                fields.append(f'{name}=unavailable')
                continue
            statistics = self._statistics[name]
            warned = max(abs(wrench.fx), abs(wrench.fy)) > warning
            fields.append(
                f'{name}: F=({wrench.fx:.2f},{wrench.fy:.2f},'
                f'{wrench.fz:.2f})N M=({wrench.tx:.2f},{wrench.ty:.2f},'
                f'{wrench.tz:.2f})Nm rms={statistics.force_rms:.2f}N '
                f'peak={statistics.peak_force:.2f}N'
                f'{" WARNING" if warned else ""}')
        return '; '.join(fields)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WrenchMonitor()
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
