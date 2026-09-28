"""Wait for observed simulator readiness and return a launch process event."""
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class SimulationStartupGate(Node):
    """Use wall time only for deadlines; readiness comes from ROS state."""

    def __init__(self):
        super().__init__('simulation_startup_gate')
        for key, value in {
            'mode': 'controllers', 'stage': 'controllers_ready',
            'robot_namespaces': ['amir1', 'amir2'], 'timeout': 180.0,
            'required_controllers': ['joint_state_broadcaster', 'arm_controller',
                                     'mecanum_drive_controller', 'gripper_controller'],
            'joint_timeout': 1.0, 'stable_duration': 0.5,
            'open_position': -1.0, 'joint_tolerance': 0.02, 'joint_velocity_tolerance': 0.02,
            'arm_home_positions': [0.0, 1.4, -1.6, 0.2, 1.570796],
        }.items():
            self.declare_parameter(key, value)
        self._mode = self.get_parameter('mode').value
        if self._mode not in ('world', 'manager', 'controllers', 'open_home'):
            raise ValueError('Unknown readiness mode')
        self._names = self.get_parameter('robot_namespaces').value
        self._started = time.monotonic()
        self._done = False
        self.success = False
        self._stable_since = None
        self._clock_previous = None
        self._clock_advancing = False
        self._clock_seen = 0.0
        self._joints = {}
        self._joint_velocities = {}
        self._joint_seen = {}
        self._manager_status = ''
        self._active = {}
        self._response_seen = {}
        self._requests = {}
        self._last_request = {}
        self._controller_clients = {}
        self._actions = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(
            Bool, '/cooperative_transport/startup/' +
            self.get_parameter('stage').value, qos)
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        if self._mode in ('manager', 'controllers'):
            for name in self._names:
                prefix = '/' + name.strip('/') if name.strip('/') else ''
                self._controller_clients[name] = self.create_client(
                    ListControllers, prefix + '/controller_manager/list_controllers')
                self._actions[name] = ActionClient(
                    self, GripperCommand, prefix + '/gripper_controller/gripper_cmd')
        if self._mode == 'open_home':
            for name in self._names:
                self.create_subscription(
                    JointState, f'/{name}/joint_states',
                    lambda msg, robot=name: self._joint(robot, msg), 10)
            self.create_subscription(
                String, '/cooperative_transport/friction_grasp_status',
                lambda msg: setattr(self, '_manager_status', msg.data), 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f'Waiting for {self._mode}: {self._names}')

    def _on_clock(self, msg):
        current = msg.clock.sec + msg.clock.nanosec * 1e-9
        self._clock_advancing = (self._clock_previous is not None
                                 and current > self._clock_previous)
        self._clock_previous = current
        self._clock_seen = time.monotonic()

    def _joint(self, name, msg):
        self._joints[name] = dict(zip(msg.name, msg.position))
        self._joint_velocities[name] = dict(zip(msg.name, msg.velocity))
        self._joint_seen[name] = time.monotonic()

    def _controllers(self, now):
        required = set(self.get_parameter('required_controllers').value)
        for name, client in self._controller_clients.items():
            pending = self._requests.get(name)
            if pending:
                future, sent = pending
                if future.done():
                    try:
                        response = future.result()
                        self._active[name] = {c.name for c in response.controller
                                              if c.state == 'active'}
                        self._response_seen[name] = now
                    except Exception:
                        self._active.pop(name, None)
                    self._requests.pop(name, None)
                elif now - sent > 5.0:
                    client.remove_pending_request(future)
                    future.cancel()
                    self._requests.pop(name, None)
            if (name not in self._requests and client.service_is_ready()
                    and now - self._last_request.get(name, 0.0) >= 0.5):
                self._last_request[name] = now
                self._requests[name] = (client.call_async(ListControllers.Request()), now)
        return all(
            now - self._response_seen.get(name, 0.0) < 2.0
            and (self._mode == 'manager' or
                 (required.issubset(self._active.get(name, set()))
                  and self._actions[name].server_is_ready()))
            for name in self._names)

    def _open_home(self, now):
        names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Gripper']
        targets = list(self.get_parameter('arm_home_positions').value) + [
            self.get_parameter('open_position').value]
        tolerance = self.get_parameter('joint_tolerance').value
        return (self._manager_status.startswith('READY_TO_GRASP;') and all(
            now - self._joint_seen.get(robot, 0.0) < self.get_parameter('joint_timeout').value
            and all(name in self._joints.get(robot, {})
                    and abs(self._joints[robot][name] - target) <= tolerance
                    and name in self._joint_velocities.get(robot, {})
                    and abs(self._joint_velocities[robot][name]) <= self.get_parameter('joint_velocity_tolerance').value
                    for name, target in zip(names, targets))
            for robot in self._names))

    def _tick(self):
        now = time.monotonic()
        if self._mode == 'world':
            ready = self._clock_advancing and now - self._clock_seen < 1.0
        elif self._mode == 'open_home':
            ready = self._open_home(now)
        else:
            ready = self._controllers(now)
        if ready:
            if self._stable_since is None:
                self._stable_since = now
            if now - self._stable_since >= self.get_parameter('stable_duration').value:
                self.success = True
                self._done = True
                self._publisher.publish(Bool(data=True))
                self.get_logger().info('Ready: ' + self.get_parameter('stage').value)
        else:
            self._stable_since = None
        if not self._done and now - self._started > self.get_parameter('timeout').value:
            self._done = True
            self._publisher.publish(Bool(data=False))
            self.get_logger().error(
                f'Readiness timeout in {self._mode}; active={self._active}, '
                f'grasp_manager={self._manager_status}')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationStartupGate()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = 0 if node.success else 1
    except KeyboardInterrupt:
        result = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
