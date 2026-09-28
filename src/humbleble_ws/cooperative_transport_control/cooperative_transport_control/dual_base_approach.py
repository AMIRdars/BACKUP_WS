"""Move both AMIR bases toward the supported payload, then request grasp."""

import math
import time
from typing import Dict

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool


class DualBaseApproach(Node):
    """Approach the payload symmetrically using odometry feedback."""

    def __init__(self) -> None:
        super().__init__('dual_base_approach')
        self.declare_parameter('robot_namespaces', ['amir1', 'amir2'])
        self.declare_parameter('approach_distance', 0.10)
        self.declare_parameter('maximum_speed', 0.035)
        self.declare_parameter('position_kp', 1.0)
        self.declare_parameter('position_tolerance', 0.003)
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('state_timeout', 0.5)
        self.declare_parameter('startup_timeout', 15.0)
        self.declare_parameter('motion_timeout', 15.0)
        self.declare_parameter('require_payload_detach', True)
        self.declare_parameter('require_arm_home', False)
        self.declare_parameter(
            'arm_home_positions', [0.0, 1.4, -1.6, 0.2, 1.570796])
        self.declare_parameter('arm_home_tolerance', 0.02)
        self.declare_parameter('open_grippers_before_approach', True)
        # Keep clear of the physical lower stop (-1.047198 rad).  Commanding
        # that exact limit stalls the Fortress gripper joint, preventing the
        # following close command from moving the fingers.
        self.declare_parameter('gripper_open_position', -1.0)
        self.declare_parameter('gripper_open_tolerance', 0.02)
        self.declare_parameter('gripper_maximum_effort', 0.8)
        self.declare_parameter(
            'attach_service', '/cooperative_transport/attach_all')

        self._names = tuple(
            str(name).strip('/') for name in
            self.get_parameter('robot_namespaces').value)
        if not self._names:
            raise ValueError('robot_namespaces must contain at least one robot')

        self._positions: Dict[str, float] = {}
        self._yaws: Dict[str, float] = {}
        self._state_times: Dict[str, float] = {}
        self._targets: Dict[str, float] = {}
        self._grasp_states: Dict[str, bool] = {}
        self._arm_home_states: Dict[str, bool] = {}
        self._joint_state_times: Dict[str, float] = {}
        self._gripper_positions: Dict[str, float] = {}
        self._gripper_state_times: Dict[str, float] = {}
        self._base_command_publishers = {
            name: self.create_publisher(Twist, f'/{name}/rover_twist', 10)
            for name in self._names
        }
        self._detach_publishers = {
            name: self.create_publisher(Empty, f'/{name}/grasp/detach', 10)
            for name in self._names
        }
        self._gripper_open_clients = {
            name: ActionClient(
                self, GripperCommand,
                f'/{name}/gripper_controller/gripper_cmd')
            for name in self._names
        }
        for name in self._names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message),
                10)
            self.create_subscription(
                String, f'/{name}/grasp/state',
                lambda message, robot=name: self._on_grasp_state(
                    robot, message),
                10)
            self.create_subscription(
                JointState, f'/{name}/joint_states',
                lambda message, robot=name: self._on_joint_state(
                    robot, message),
                10)

        self._attach_client = self.create_client(
            SetBool, str(self.get_parameter('attach_service').value))
        self._phase = 'WAITING_FOR_DETACH'
        self._started_at = time.monotonic()
        self._motion_started_at = 0.0
        self._request_pending = False
        self._open_goals_sent = False
        self._accepted_open_goals = set()
        rate = float(self.get_parameter('control_rate').value)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        distance = float(self.get_parameter('approach_distance').value)
        preparation = (
            'detaching its initial grasp joints and '
            if bool(self.get_parameter('require_payload_detach').value)
            else '')
        self.get_logger().info(
            f'Waiting for the payload, {preparation}'
            'opening both grippers to their safe maximum, '
            f'then moving both AMIR bases {distance:.3f} m toward it')

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        pose = message.pose.pose
        orientation = pose.orientation
        self._positions[robot] = pose.position.x
        self._yaws[robot] = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z))
        self._state_times[robot] = time.monotonic()

    def _on_grasp_state(self, robot: str, message: String) -> None:
        state = message.data.strip().lower()
        if state in ('attached', 'detached'):
            self._grasp_states[robot] = state == 'attached'

    def _on_joint_state(self, robot: str, message: JointState) -> None:
        required_names = (
            'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5')
        positions = dict(zip(message.name, message.position))
        if 'Gripper' in positions:
            self._gripper_positions[robot] = positions['Gripper']
            self._gripper_state_times[robot] = time.monotonic()
        if not all(name in positions for name in required_names):
            return
        target = tuple(float(value) for value in self.get_parameter(
            'arm_home_positions').value)
        if len(target) != len(required_names):
            self._arm_home_states[robot] = False
        else:
            tolerance = float(
                self.get_parameter('arm_home_tolerance').value)
            self._arm_home_states[robot] = all(
                abs(positions[name] - desired) <= tolerance
                for name, desired in zip(required_names, target))
        self._joint_state_times[robot] = time.monotonic()

    def _publish_detach(self) -> None:
        message = Empty()
        for publisher in self._detach_publishers.values():
            publisher.publish(message)

    def _states_ready(self) -> bool:
        now = time.monotonic()
        timeout = float(self.get_parameter('state_timeout').value)
        return all(
            name in self._positions and name in self._yaws
            and now - self._state_times[name] <= timeout
            for name in self._names)

    def _arms_ready(self) -> bool:
        if not bool(self.get_parameter('require_arm_home').value):
            return True
        now = time.monotonic()
        timeout = float(self.get_parameter('state_timeout').value)
        return all(
            self._arm_home_states.get(name, False)
            and name in self._joint_state_times
            and now - self._joint_state_times[name] <= timeout
            for name in self._names)

    def _grippers_open(self) -> bool:
        now = time.monotonic()
        timeout = float(self.get_parameter('state_timeout').value)
        target = float(self.get_parameter('gripper_open_position').value)
        tolerance = float(self.get_parameter('gripper_open_tolerance').value)
        return all(
            name in self._accepted_open_goals
            and name in self._gripper_positions
            and now - self._gripper_state_times[name] <= timeout
            and abs(self._gripper_positions[name] - target) <= tolerance
            for name in self._names)

    def _publish_zero(self) -> None:
        zero = Twist()
        for publisher in self._base_command_publishers.values():
            publisher.publish(zero)

    def _ros_now(self) -> float:
        """Return ROS time so motion timeout follows Gazebo simulation time."""
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _finish(self, success: bool, message: str) -> None:
        self._publish_zero()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        self._timer.cancel()
        rclpy.shutdown()

    def _request_grasp(self) -> None:
        if self._request_pending:
            return
        if not self._attach_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = True
        self._request_pending = True
        future = self._attach_client.call_async(request)
        future.add_done_callback(self._on_grasp_response)

    def _on_grasp_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exception:
            self._finish(False, f'Failed to request grasp: {exception}')
            return
        if not response.success:
            self._finish(False, f'Grasp request rejected: {response.message}')
            return
        self._phase = 'HOLDING_ZERO'
        self.get_logger().info(
            'Both bases reached the payload; gripper-close sequence requested. '
            'Holding both bases stopped during grasp.')

    def _start_moving(self) -> None:
        """Capture approach targets only after both grippers are safely open."""
        distance = float(self.get_parameter('approach_distance').value)
        self._targets = {
            name: position + (distance if position < 0.0 else -distance)
            for name, position in self._positions.items()
        }
        self._motion_started_at = self._ros_now()
        self._phase = 'MOVING'
        positions = ', '.join(
            f'{name}={self._positions[name]:.6f}'
            for name in self._names)
        targets = ', '.join(
            f'{name}={target:.6f}'
            for name, target in self._targets.items())
        self.get_logger().info(
            f'Both grippers are at their safe-open position. '
            'Payload detached at base positions: '
            f'{positions}; approaching targets: {targets}')

    def _send_open_goals(self) -> None:
        if not all(
                client.server_is_ready()
                for client in self._gripper_open_clients.values()):
            return
        position = float(self.get_parameter('gripper_open_position').value)
        effort = float(self.get_parameter('gripper_maximum_effort').value)
        self._open_goals_sent = True
        for robot, client in self._gripper_open_clients.items():
            goal = GripperCommand.Goal()
            goal.command.position = position
            goal.command.max_effort = effort
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda response, name=robot:
                self._on_open_goal_response(name, response))
        self.get_logger().info(
            f'Opening both grippers to the safe maximum ({position:.6f} rad) before '
            'base approach')

    def _on_open_goal_response(self, robot: str, future) -> None:
        try:
            handle = future.result()
        except Exception as exception:
            self._finish(False, f'{robot} gripper open request failed: {exception}')
            return
        if handle is None or not handle.accepted:
            self._finish(False, f'{robot} gripper controller rejected open command')
            return
        self._accepted_open_goals.add(robot)
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result, name=robot: self._on_open_result(name, result))

    def _on_open_result(self, robot: str, future) -> None:
        try:
            result = future.result().result
        except Exception as exception:
            self._finish(False, f'{robot} gripper open action failed: {exception}')
            return
        # The grasp manager may send the same open goal and cancel ours.
        # Neither a canceled action nor a stalled flag proves the physical
        # opening: the timer gates motion on fresh measured joint positions.
        if self._phase == 'OPENING_GRIPPERS' and not result.reached_goal:
            self.get_logger().info(
                f'{robot} open action ended; waiting for measured safe opening')

    def _tick(self) -> None:
        now = time.monotonic()
        if self._phase == 'WAITING_FOR_DETACH':
            # Gazebo Fortress DetachableJoint starts attached. Publish before
            # payload creation as well, so the new plugin detaches immediately
            # and cannot pull the arms across the 0.10 m spawn offset.
            self._publish_zero()
            require_detach = bool(
                self.get_parameter('require_payload_detach').value)
            if require_detach:
                self._publish_detach()
            detached = (
                not require_detach
                or all(
                    self._grasp_states.get(name) is False
                    for name in self._names))
            if detached and self._states_ready() and self._arms_ready():
                if bool(self.get_parameter(
                        'open_grippers_before_approach').value):
                    self._phase = 'OPENING_GRIPPERS'
                else:
                    self._start_moving()
            elif now - self._started_at > float(
                    self.get_parameter('startup_timeout').value):
                self._finish(
                    False,
                    'Timed out waiting for payload detach, Q_HOME, and AMIR '
                    'odometry')
            return

        if self._phase == 'OPENING_GRIPPERS':
            self._publish_zero()
            if not self._open_goals_sent:
                self._send_open_goals()
            if self._grippers_open() and self._states_ready() and self._arms_ready():
                self._start_moving()
            elif now - self._started_at > float(
                    self.get_parameter('startup_timeout').value):
                self._finish(False, 'Timed out waiting for measured safe-open grippers')
            return

        if self._phase == 'REQUESTING_GRASP':
            self._publish_zero()
            self._request_grasp()
            return

        if self._phase == 'HOLDING_ZERO':
            self._publish_zero()
            return

        if not self._states_ready():
            self._finish(False, 'Lost fresh odometry while approaching payload')
            return

        maximum_speed = float(self.get_parameter('maximum_speed').value)
        position_kp = float(self.get_parameter('position_kp').value)
        tolerance = float(self.get_parameter('position_tolerance').value)
        all_reached = True
        for name in self._names:
            error = self._targets[name] - self._positions[name]
            if abs(error) > tolerance:
                all_reached = False
            world_vx = max(
                -maximum_speed,
                min(maximum_speed, position_kp * error))
            yaw = self._yaws[name]
            command = Twist()
            command.linear.x = math.cos(yaw) * world_vx
            command.linear.y = -math.sin(yaw) * world_vx
            self._base_command_publishers[name].publish(command)

        if all_reached:
            self._publish_zero()
            self._phase = 'REQUESTING_GRASP'
            distance = float(self.get_parameter('approach_distance').value)
            self.get_logger().info(
                f'Both AMIR bases completed the {distance:.3f} m '
                'payload approach')
            return
        if self._ros_now() - self._motion_started_at > float(
                self.get_parameter('motion_timeout').value):
            self._finish(False, 'Timed out while approaching the payload')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualBaseApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_zero()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
