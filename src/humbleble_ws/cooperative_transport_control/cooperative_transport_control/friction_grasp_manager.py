"""Operate both AMIR grippers for a contact-and-friction grasp."""

from collections import deque
import math
import time
from typing import Deque, Dict, Optional, Tuple

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Contact, Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .grip_force_control import holding_recovery_position, next_gripper_position


_FINGERS = ('left', 'right')


def contact_normal_force(
        contact: Contact, robot: str, finger: str,
        payload_token: str = 'cooperative_payload') -> float:
    """Return the summed normal force for one finger-payload contact."""
    first = contact.collision1.name
    second = contact.collision2.name
    finger_token = f'finger_{finger}_1'
    first_is_finger = robot in first and finger_token in first
    second_is_finger = robot in second and finger_token in second
    if first_is_finger and payload_token not in second:
        return 0.0
    if second_is_finger and payload_token not in first:
        return 0.0
    if not first_is_finger and not second_is_finger:
        return 0.0

    total = 0.0
    for index, wrench in enumerate(contact.wrenches):
        vector = (
            wrench.body_1_wrench.force if first_is_finger
            else wrench.body_2_wrench.force)
        if index < len(contact.normals):
            normal = contact.normals[index]
            total += abs(
                vector.x * normal.x
                + vector.y * normal.y
                + vector.z * normal.z)
        else:
            total += math.sqrt(
                vector.x * vector.x
                + vector.y * vector.y
                + vector.z * vector.z)
    return total


class FrictionGraspManager(Node):
    """Open, close, preload, and lift two grippers without fixed joints."""

    def __init__(self) -> None:
        super().__init__('friction_grasp_manager')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('auto_grasp', True)
        self.declare_parameter('auto_grasp_delay', 5.0)
        self.declare_parameter('controller_wait_timeout', 12.0)
        self.declare_parameter('open_position', -1.0)
        self.declare_parameter('close_position', 0.20)
        self.declare_parameter('maximum_effort', 0.8)
        self.declare_parameter('target_normal_force', 14.0)
        self.declare_parameter('force_tolerance', 3.0)
        self.declare_parameter('force_position_gain', 0.00025)
        self.declare_parameter('force_control_max_step', 0.004)
        self.declare_parameter('overforce_release_max_step', 0.006)
        self.declare_parameter('contact_search_step', 0.008)
        self.declare_parameter('contact_reacquisition_step', 0.0002)
        self.declare_parameter('force_control_interval', 0.10)
        self.declare_parameter('force_filter_alpha', 0.25)
        self.declare_parameter('closing_timeout', 30.0)
        self.declare_parameter('maximum_normal_force', 20.0)
        self.declare_parameter('holding_force_release_threshold', 18.0)
        self.declare_parameter('holding_max_release_offset', 0.002)
        self.declare_parameter('prioritize_grasp_retention', True)
        self.declare_parameter('bias_sample_duration', 0.50)
        self.declare_parameter('open_duration', 2.0)
        self.declare_parameter('close_duration', 3.0)
        self.declare_parameter('contact_verification_timeout', 2.0)
        self.declare_parameter('lift_duration', 3.0)
        self.declare_parameter('lift_joint_2', 0.10)
        self.declare_parameter('lift_joint_3', -0.10)
        self.declare_parameter('minimum_contact_position', -0.90)
        self.declare_parameter('maximum_contact_position', 0.15)
        self.declare_parameter('maximum_gripper_mismatch', 0.05)
        self.declare_parameter('require_contact', False)
        self.declare_parameter('payload_contact_token', 'cooperative_payload')
        self.declare_parameter('minimum_normal_force', 8.0)
        self.declare_parameter('contact_timeout', 0.25)
        self.declare_parameter('contact_stability_duration', 0.30)
        self.declare_parameter('contact_loss_timeout', 0.30)

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._gripper_clients = {
            name: ActionClient(
                self, GripperCommand,
                f'/{name}/gripper_controller/gripper_cmd')
            for name in self._names
        }
        self._arm_publishers = {
            name: self.create_publisher(
                JointTrajectory, f'/{name}/arm_controller/joint_trajectory', 10)
            for name in self._names
        }
        self._state_publishers = {
            name: self.create_publisher(
                Bool, f'/cooperative_transport/{name}/grasp_state', 10)
            for name in self._names
        }
        self._joint_positions: Dict[str, float] = {}
        self._contact_forces: Dict[Tuple[str, str], float] = {}
        self._filtered_contact_forces: Dict[Tuple[str, str], float] = {}
        self._contact_times: Dict[Tuple[str, str], float] = {}
        self._force_times: Dict[Tuple[str, str], float] = {}
        self._contact_present: Dict[Tuple[str, str], bool] = {}
        self._wrenches: Dict[Tuple[str, str], Tuple[float, ...]] = {}
        self._wrench_samples: Dict[
            Tuple[str, str], Deque[Tuple[float, Tuple[float, ...]]]] = {
                (robot, finger): deque()
                for robot in self._names for finger in _FINGERS
            }
        self._wrench_biases: Dict[Tuple[str, str], Tuple[float, ...]] = {}
        self._gripper_commands: Dict[str, float] = {}
        self._contact_seen: Dict[str, bool] = {
            name: False for name in self._names}
        # Once a stable grasp has been established, transient tangential loads
        # during transport must not be interpreted as permission to open past
        # the grasped position.  The floor is released only for a true
        # over-force condition or an explicit release request.
        self._holding_position_floors: Dict[str, float] = {}
        self._last_force_control = 0.0
        for name in self._names:
            self.create_subscription(
                JointState, f'/{name}/joint_states',
                lambda message, robot=name: self._on_joint_state(robot, message), 10)
            for finger in _FINGERS:
                self.create_subscription(
                    Contacts, f'/{name}/finger_{finger}_contact',
                    lambda message, robot=name, side=finger:
                    self._on_contact(robot, side, message), 20)
                self.create_subscription(
                    WrenchStamped, f'/{name}/finger_{finger}_wrench',
                    lambda message, robot=name, side=finger:
                    self._on_finger_wrench(robot, side, message), 20)

        self._calibrated_wrench_publishers = {
            (robot, finger): self.create_publisher(
                WrenchStamped,
                f'/{robot}/finger_{finger}_wrench_calibrated', 20)
            for robot in self._names for finger in _FINGERS
        }

        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/friction_grasp_status', 10)
        self._contact_status_publisher = self.create_publisher(
            String, '/cooperative_transport/contact_status', 10)
        self.create_service(
            SetBool, '/cooperative_transport/friction_grasp', self._on_command)
        self.create_service(
            Trigger, '/cooperative_transport/friction_grasp_status', self._on_status)

        self._state = 'WAITING_FOR_CONTROLLERS'
        self._held = False
        self._state_started = self._now()
        self._controller_wait_started = time.monotonic()
        self._controller_warning_sent = False
        self._auto_requested = False
        self._action_rejected = False
        self._contacts_valid_since: Optional[float] = None
        self._contact_lost_since: Optional[float] = None
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'Friction grasp manager ready; no detachable joint will be used.')

    def _on_joint_state(self, robot: str, message: JointState) -> None:
        try:
            index = message.name.index('Gripper')
        except ValueError:
            return
        if index < len(message.position):
            self._joint_positions[robot] = message.position[index]

    def _on_contact(
            self, robot: str, finger: str, message: Contacts) -> None:
        payload = str(self.get_parameter('payload_contact_token').value)
        key = (robot, finger)
        self._contact_present[key] = any(
            contact_normal_force(contact, robot, finger, payload) > 0.0
            or self._is_payload_contact(contact, robot, finger, payload)
            for contact in message.contacts)
        self._contact_times[key] = self._now()

    @staticmethod
    def _is_payload_contact(
            contact: Contact, robot: str, finger: str,
            payload_token: str) -> bool:
        names = (contact.collision1.name, contact.collision2.name)
        finger_token = f'finger_{finger}_1'
        return (
            any(robot in name and finger_token in name for name in names)
            and any(payload_token in name for name in names))

    def _on_finger_wrench(
            self, robot: str, finger: str,
            message: WrenchStamped) -> None:
        key = (robot, finger)
        # The contact face normal is the force-sensor X axis.  Taking its
        # absolute load is independent of which side of the payload is held.
        normal_force = abs(message.wrench.force.x)
        self._contact_forces[key] = normal_force
        alpha = float(self.get_parameter('force_filter_alpha').value)
        previous = self._filtered_contact_forces.get(key, normal_force)
        self._filtered_contact_forces[key] = (
            alpha * normal_force + (1.0 - alpha) * previous)
        now = self._now()
        self._force_times[key] = now

        wrench = (
            message.wrench.force.x, message.wrench.force.y,
            message.wrench.force.z, message.wrench.torque.x,
            message.wrench.torque.y, message.wrench.torque.z)
        self._wrenches[key] = wrench
        samples = self._wrench_samples[key]
        samples.append((now, wrench))
        keep_duration = max(
            1.0, float(self.get_parameter('bias_sample_duration').value) * 2.0)
        while samples and now - samples[0][0] > keep_duration:
            samples.popleft()

        if key in self._wrench_biases:
            self._publish_calibrated_wrench(key, message)

    def _publish_calibrated_wrench(
            self, key: Tuple[str, str], message: WrenchStamped) -> None:
        """Publish sensor load relative to the steady post-grasp preload."""
        bias = self._wrench_biases[key]
        calibrated = WrenchStamped()
        calibrated.header = message.header
        calibrated.wrench.force.x = message.wrench.force.x - bias[0]
        calibrated.wrench.force.y = message.wrench.force.y - bias[1]
        calibrated.wrench.force.z = message.wrench.force.z - bias[2]
        calibrated.wrench.torque.x = message.wrench.torque.x - bias[3]
        calibrated.wrench.torque.y = message.wrench.torque.y - bias[4]
        calibrated.wrench.torque.z = message.wrench.torque.z - bias[5]
        self._calibrated_wrench_publishers[key].publish(calibrated)

    def _capture_wrench_biases(self) -> None:
        """Average the final settled grasp samples and use them as zero."""
        now = self._now()
        duration = float(self.get_parameter('bias_sample_duration').value)
        biases = {}
        for key in self._wrench_samples:
            recent = [
                wrench for stamp, wrench in self._wrench_samples[key]
                if now - stamp <= duration]
            if not recent and key in self._wrenches:
                recent = [self._wrenches[key]]
            if recent:
                biases[key] = tuple(
                    sum(wrench[index] for wrench in recent) / len(recent)
                    for index in range(6))
        self._wrench_biases = biases
        summary = ', '.join(
            f'{robot}/{finger}={abs(bias[0]):.1f}N'
            for (robot, finger), bias in sorted(biases.items()))
        self.get_logger().info(f'Finger wrench bias captured: {summary}')

    def _set_state(self, state: str) -> None:
        self._state = state
        self._state_started = self._now()
        self.get_logger().info(f'Friction grasp state: {state}')

    def _now(self) -> float:
        """Return ROS time so action timing follows Gazebo simulation time."""
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _send_grippers(self, position: float) -> None:
        self._action_rejected = False
        for name in self._names:
            self._send_gripper(name, position)

    def _send_gripper(self, name: str, position: float) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = float(
            self.get_parameter('maximum_effort').value)
        self._gripper_commands[name] = position
        future = self._gripper_clients[name].send_goal_async(goal)
        future.add_done_callback(
            lambda result, robot=name: self._on_goal_response(robot, result))

    def _on_goal_response(self, robot: str, future) -> None:
        try:
            handle = future.result()
        except Exception as exception:  # noqa: B902 - ROS future exception
            self._action_rejected = True
            self.get_logger().error(f'{robot} gripper goal failed: {exception}')
            return
        if not handle.accepted:
            self._action_rejected = True
            self.get_logger().error(f'{robot} gripper goal was rejected.')

    def _send_arm_pose(self, lifted: bool) -> None:
        message = JointTrajectory()
        message.joint_names = [
            'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        point = JointTrajectoryPoint()
        if lifted:
            point.positions = [
                0.0,
                float(self.get_parameter('lift_joint_2').value),
                float(self.get_parameter('lift_joint_3').value),
                0.0,
                0.0,
            ]
        else:
            point.positions = [0.0] * 5
        seconds = float(self.get_parameter('lift_duration').value)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1.0) * 1.0e9)
        message.points = [point]
        for publisher in self._arm_publishers.values():
            publisher.publish(message)

    def _joint_contact_plausible(self) -> bool:
        if len(self._joint_positions) != len(self._names):
            return False
        minimum = float(self.get_parameter('minimum_contact_position').value)
        maximum = float(self.get_parameter('maximum_contact_position').value)
        mismatch = float(self.get_parameter('maximum_gripper_mismatch').value)
        positions = [self._joint_positions[name] for name in self._names]
        return (
            all(minimum <= position <= maximum for position in positions)
            and abs(positions[0] - positions[1]) <= mismatch)

    def _physical_contacts_valid(self) -> bool:
        now = self._now()
        timeout = float(self.get_parameter('contact_timeout').value)
        minimum_force = float(
            self.get_parameter('minimum_normal_force').value)
        return all(
            key in self._contact_times
            and now - self._contact_times[key] <= timeout
            and self._contact_present.get(key, False)
            and key in self._force_times
            and now - self._force_times[key] <= timeout
            and self._contact_forces.get(key, 0.0) >= minimum_force
            for key in (
                (robot, finger)
                for robot in self._names for finger in _FINGERS))

    def _bilateral_contact(self, robot: str) -> bool:
        now = self._now()
        timeout = float(self.get_parameter('contact_timeout').value)
        return all(
            key in self._contact_times
            and now - self._contact_times[key] <= timeout
            and self._contact_present.get(key, False)
            for key in ((robot, finger) for finger in _FINGERS))

    def _force_target_reached(self) -> bool:
        if not all(self._bilateral_contact(robot) for robot in self._names):
            return False
        target = float(self.get_parameter('target_normal_force').value)
        tolerance = float(self.get_parameter('force_tolerance').value)
        return all(
            abs(self._filtered_contact_forces.get(key, 0.0) - target)
            <= tolerance
            for key in (
                (robot, finger)
                for robot in self._names for finger in _FINGERS))

    def _update_force_control(self) -> None:
        """Search for contact, then maintain the requested normal force."""
        now = self._now()
        interval = float(self.get_parameter('force_control_interval').value)
        if now - self._last_force_control < interval:
            return
        self._last_force_control = now

        target = float(self.get_parameter('target_normal_force').value)
        tolerance = float(self.get_parameter('force_tolerance').value)
        gain = float(self.get_parameter('force_position_gain').value)
        maximum_step = float(
            self.get_parameter('force_control_max_step').value)
        overforce_step = float(self.get_parameter(
            'overforce_release_max_step').value)
        search_step = float(self.get_parameter('contact_search_step').value)
        reacquisition_step = float(self.get_parameter(
            'contact_reacquisition_step').value)
        minimum_position = float(self.get_parameter('open_position').value)
        maximum_position = float(self.get_parameter('close_position').value)
        maximum_force = float(
            self.get_parameter('maximum_normal_force').value)

        for robot in self._names:
            if robot not in self._joint_positions:
                continue
            filtered_forces = tuple(
                self._filtered_contact_forces.get((robot, finger), 0.0)
                for finger in _FINGERS)
            # Filtering is useful for target tracking, but must never delay a
            # protective opening command.  Use the larger of filtered and raw
            # normal load for every upper-limit decision.
            raw_forces = tuple(
                self._contact_forces.get((robot, finger), 0.0)
                for finger in _FINGERS)
            if max(raw_forces) > 0.1 or self._bilateral_contact(robot):
                self._contact_seen[robot] = True
            forces = tuple(max(filtered, raw) for filtered, raw in zip(
                filtered_forces, raw_forces))
            over_force = max(forces) > maximum_force
            retain_grasp = bool(
                self.get_parameter('prioritize_grasp_retention').value)
            if over_force:
                self.get_logger().warning(
                    f'{robot} finger force above {maximum_force:.1f} N; '
                    + ('retaining verified grasp position.' if retain_grasp
                       else 'backing off.'),
                    throttle_duration_sec=1.0)
            if self._state == 'HOLDING':
                # Once the support has been removed, opening on a short force
                # peak can irreversibly drop the payload. In retention mode,
                # keep the verified position and only close to recover a weak
                # finger contact. Force peaks remain reported for analysis.
                release_step = overforce_step if over_force else maximum_step
                control_position = self._gripper_commands.get(
                    robot, self._joint_positions[robot])
                position = holding_recovery_position(
                    control_position,
                    self._gripper_commands.get(
                        robot, self._joint_positions[robot]),
                    forces, target,
                    float(self.get_parameter('minimum_normal_force').value),
                    float(self.get_parameter(
                        'holding_force_release_threshold').value),
                    gain, release_step, minimum_position, maximum_position,
                    allow_release=not retain_grasp)
                # Do not release far enough to lose the payload during
                # the support-removal transient. The bound is relative to the
                # verified grasp position and still permits force relief.
                if robot in self._holding_position_floors:
                    release_limit = (
                        self._holding_position_floors[robot]
                        - float(self.get_parameter(
                            'holding_max_release_offset').value))
                    position = max(position, release_limit)
            else:
                # Before first contact, base search on the measured joint so
                # the command cannot run far ahead of the actuator.  After
                # contact, accumulate sub-milliradian force corrections on
                # the previous command; repeatedly using the measured joint
                # would reissue the same goal while the action was moving.
                control_position = self._joint_positions[robot]
                if self._contact_seen[robot]:
                    control_position = self._gripper_commands.get(
                        robot, control_position)
                position = next_gripper_position(
                    control_position, forces,
                    self._bilateral_contact(robot), target, tolerance, gain,
                    overforce_step if over_force else maximum_step,
                    (reacquisition_step if self._contact_seen[robot]
                     else search_step),
                    minimum_position, maximum_position)
            previous = self._gripper_commands.get(robot)
            if previous is None or abs(position - previous) >= 1.0e-4:
                self._send_gripper(robot, position)

    def _contact_plausible(self) -> bool:
        if not self._joint_contact_plausible():
            return False
        return (
            not bool(self.get_parameter('require_contact').value)
            or self._physical_contacts_valid())

    def _request_grasp(self) -> bool:
        if self._state not in ('READY_TO_GRASP', 'RELEASED'):
            return False
        self._action_rejected = False
        self._wrench_biases.clear()
        self._holding_position_floors.clear()
        self._gripper_commands = dict(self._joint_positions)
        self._contact_seen = {name: False for name in self._names}
        self._last_force_control = 0.0
        self._contacts_valid_since = None
        self._contact_lost_since = None
        self._set_state('CLOSING_GRIPPERS')
        return True

    def _request_release(self) -> bool:
        if self._state != 'HOLDING':
            return False
        self._held = False
        self._holding_position_floors.clear()
        self._send_arm_pose(False)
        self._set_state('LOWERING')
        return True

    def _on_command(self, request: SetBool.Request,
                    response: SetBool.Response) -> SetBool.Response:
        accepted = self._request_grasp() if request.data else self._request_release()
        response.success = accepted
        response.message = (
            'Sequence accepted.' if accepted
            else f'Cannot change grasp while state is {self._state}.')
        return response

    def _on_status(self, _request: Trigger.Request,
                   response: Trigger.Response) -> Trigger.Response:
        response.success = self._held
        response.message = self._status_text()
        return response

    def _tick(self) -> None:
        now = self._now()
        elapsed = now - self._state_started
        if self._state == 'WAITING_FOR_CONTROLLERS':
            controllers_ready = all(
                client.server_is_ready()
                for client in self._gripper_clients.values())
            wait_timeout = float(
                self.get_parameter('controller_wait_timeout').value)
            if controllers_ready:
                self._send_grippers(float(self.get_parameter('open_position').value))
                self._set_state('OPENING_GRIPPERS')
            elif (not self._controller_warning_sent
                  and time.monotonic() - self._controller_wait_started
                  >= wait_timeout):
                self._controller_warning_sent = True
                self.get_logger().warning(
                    'Still waiting for both gripper action servers; no partial '
                    'grasp command will be sent.')
        elif self._state == 'OPENING_GRIPPERS':
            if elapsed >= float(self.get_parameter('open_duration').value):
                self._set_state('READY_TO_GRASP')
        elif self._state == 'READY_TO_GRASP':
            auto_delay = float(self.get_parameter('auto_grasp_delay').value)
            if (bool(self.get_parameter('auto_grasp').value)
                    and not self._auto_requested
                    and elapsed >= auto_delay):
                self._auto_requested = True
                self._request_grasp()
        elif self._state == 'CLOSING_GRIPPERS':
            self._update_force_control()
            if self._action_rejected:
                self._set_state('ERROR_GRIPPER_ACTION')
            elif elapsed >= float(self.get_parameter('close_duration').value):
                if self._contact_plausible() and self._force_target_reached():
                    if self._contacts_valid_since is None:
                        self._contacts_valid_since = now
                    stable = float(self.get_parameter(
                        'contact_stability_duration').value)
                    if now - self._contacts_valid_since >= stable:
                        self._send_arm_pose(True)
                        self._set_state('LIFTING')
                else:
                    self._contacts_valid_since = None
                if (self._state == 'CLOSING_GRIPPERS' and elapsed >= float(
                        self.get_parameter('closing_timeout').value)):
                    error = (
                        'ERROR_NO_PHYSICAL_CONTACT'
                        if bool(self.get_parameter('require_contact').value)
                        else 'ERROR_NO_GRIPPER_MOTION')
                    self._set_state(error)
        elif self._state == 'LIFTING':
            self._update_force_control()
            if elapsed >= float(self.get_parameter('lift_duration').value) + 0.5:
                if self._contact_plausible():
                    self._held = True
                    self._contact_lost_since = None
                    self._holding_position_floors = {
                        robot: self._joint_positions[robot]
                        for robot in self._names
                        if robot in self._joint_positions
                    }
                    self._capture_wrench_biases()
                    self._set_state('HOLDING')
                elif elapsed >= (
                        float(self.get_parameter('lift_duration').value) + 0.5
                        + float(self.get_parameter(
                            'contact_verification_timeout').value)):
                    self._set_state('ERROR_CONTACT_LOST_DURING_LIFT')
        elif self._state == 'HOLDING':
            self._update_force_control()
            if (bool(self.get_parameter('require_contact').value)
                    and not self._physical_contacts_valid()):
                if self._contact_lost_since is None:
                    self._contact_lost_since = now
                elif now - self._contact_lost_since >= float(
                        self.get_parameter('contact_loss_timeout').value):
                    self._held = False
                    self._set_state('ERROR_CONTACT_LOST')
            else:
                self._contact_lost_since = None
        elif self._state == 'LOWERING':
            if elapsed >= float(self.get_parameter('lift_duration').value) + 0.5:
                self._send_grippers(float(self.get_parameter('open_position').value))
                self._set_state('OPENING_FOR_RELEASE')
        elif self._state == 'OPENING_FOR_RELEASE':
            if elapsed >= float(self.get_parameter('open_duration').value):
                self._set_state('RELEASED')

        self._publish_state()

    def _status_text(self) -> str:
        positions = ', '.join(
            f'{name}={self._joint_positions.get(name, float("nan")):.3f}'
            for name in self._names)
        commands = ', '.join(
            f'{name}={self._gripper_commands.get(name, float("nan")):.3f}'
            for name in self._names)
        floors = ', '.join(
            f'{name}={self._holding_position_floors.get(name, float("nan")):.3f}'
            for name in self._names)
        forces = ', '.join(
            f'{robot}/{finger}='
            f'{self._contact_forces.get((robot, finger), 0.0):.1f}N'
            for robot in self._names for finger in _FINGERS)
        contact_state = (
            'verified' if self._physical_contacts_valid() else 'not_verified')
        return (
            f'{self._state}; held={self._held}; contacts={contact_state}; '
            f'grippers: {positions}; commands: {commands}; hold_floors: '
            f'{floors}; target_force='
            f'{float(self.get_parameter("target_normal_force").value):.1f}N; '
            f'normal_forces: {forces}')

    def _publish_state(self) -> None:
        state = Bool()
        state.data = self._held
        for publisher in self._state_publishers.values():
            publisher.publish(state)
        status = String()
        status.data = self._status_text()
        self._status_publisher.publish(status)
        contact = String()
        contact.data = (
            'CONTACT_OK' if self._physical_contacts_valid()
            else 'CONTACT_NOT_VERIFIED')
        self._contact_status_publisher.publish(contact)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrictionGraspManager()
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
