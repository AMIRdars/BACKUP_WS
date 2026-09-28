"""Centralized leader-follower coordinator for two AMIR mobile bases."""

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from .admittance import OneAxisAdmittance
from .kinematics import clamp_vector, estimate_payload_pose, Pose2, Twist2
from .kinematics import rate_limit, rigid_body_commands
from .kinematics import yaw_from_quaternion


class CooperativeCoordinator(Node):
    """Generate both base commands from one payload-frame goal."""

    def __init__(self) -> None:
        super().__init__('cooperative_coordinator')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('position_kp', 0.5)
        self.declare_parameter('yaw_kp', 0.8)
        self.declare_parameter('heading_kp', 0.3)
        self.declare_parameter('max_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.10)
        self.declare_parameter('max_linear_acceleration', 0.10)
        self.declare_parameter('max_angular_acceleration', 0.20)
        self.declare_parameter('odometry_timeout', 0.30)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', math.radians(2.0))
        self.declare_parameter('require_attached', True)
        self.declare_parameter('use_measured_payload_pose', False)
        self.declare_parameter('use_measured_payload_position', False)
        self.declare_parameter('measured_payload_position_weight', 1.0)
        self.declare_parameter('formation_alignment_kp', 0.0)
        self.declare_parameter('formation_alignment_max_fraction', 1.0)
        self.declare_parameter('formation_alignment_tolerance', 0.008)
        self.declare_parameter('formation_recovery_start', 0.0)
        self.declare_parameter('formation_recovery_stop', 0.0)
        self.declare_parameter('formation_recovery_speed', 0.0)
        self.declare_parameter('enable_formation_distance_control', False)
        self.declare_parameter('formation_distance_kp', 0.8)
        self.declare_parameter(
            'formation_distance_max_correction_speed', 0.012)
        self.declare_parameter('formation_distance_tolerance', 0.005)
        self.declare_parameter('payload_pose_timeout', 0.30)
        self.declare_parameter('use_world_model_poses', False)
        self.declare_parameter('world_name', 'cooperative_transport_friction')
        # Calibration multipliers keep the object-level kinematics in REP-103
        # even if a particular base/controller needs an axis sign correction.
        self.declare_parameter('lateral_command_sign', 1.0)
        self.declare_parameter('angular_command_sign', 1.0)
        # Stage-7 compliance: only the follower base's local Y axis is
        # compliant. It is opt-in so previous validated trajectories retain
        # exactly the same controller behavior.
        self.declare_parameter('enable_lateral_admittance', False)
        self.declare_parameter('admittance_mass', 3.0)
        self.declare_parameter('admittance_damping', 26.8)
        self.declare_parameter('admittance_stiffness', 60.0)
        self.declare_parameter('admittance_force_deadband', 3.0)
        self.declare_parameter('admittance_filter_time_constant', 0.18)
        self.declare_parameter('admittance_max_displacement', 0.012)
        self.declare_parameter('admittance_max_velocity', 0.004)
        self.declare_parameter('admittance_force_sign', 1.0)
        self.declare_parameter('admittance_sensor_timeout', 0.25)
        self.declare_parameter('allow_external_command_handoff', False)

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._poses: Dict[str, Pose2] = {}
        self._odom_times: Dict[str, float] = {}
        self._attached: Dict[str, Optional[bool]] = {name: None for name in self._names}
        self._last_commands = {name: Twist2(0.0, 0.0, 0.0) for name in self._names}
        self._enabled = False
        self._emergency_stop = False
        self._external_control = False
        self._goal: Optional[Pose2] = None
        self._measured_payload: Optional[Pose2] = None
        self._measured_payload_time = 0.0
        self._payload_offset_local: Optional[Tuple[float, float]] = None
        self._formation_distance_target: Optional[float] = None
        self._alignment_recovering = False
        self._last_tick = time.monotonic()
        self._last_goal_log_time = 0.0
        self._wrench_y: Dict[str, float] = {}
        self._wrench_times: Dict[str, float] = {}
        self._wrench_bias: Dict[str, float] = {}
        self._admittance = OneAxisAdmittance(
            float(self.get_parameter('admittance_mass').value),
            float(self.get_parameter('admittance_damping').value),
            float(self.get_parameter('admittance_stiffness').value),
            float(self.get_parameter('admittance_force_deadband').value),
            float(self.get_parameter(
                'admittance_filter_time_constant').value),
            float(self.get_parameter('admittance_max_displacement').value),
            float(self.get_parameter('admittance_max_velocity').value),
        )

        self._command_publishers = {}
        for name in self._names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message), 10)
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp_state(robot, message), 10)
            self.create_subscription(
                WrenchStamped, f'/{name}/ft_sensor',
                lambda message, robot=name: self._on_wrench(robot, message), 10)
            self._command_publishers[name] = self.create_publisher(
                Twist, f'/{name}/rover_twist', 10)

        self.create_subscription(
            PoseStamped, '/cooperative_transport/goal', self._on_goal, 10)
        self.create_subscription(
            PoseStamped, '/cooperative_transport/measured_payload_pose',
            self._on_measured_payload, 10)
        self.create_subscription(
            Bool, '/cooperative_transport/enable', self._on_enable_topic, 10)
        self.create_subscription(
            Bool, '/cooperative_transport/emergency_stop', self._on_emergency_stop, 10)
        if bool(self.get_parameter('allow_external_command_handoff').value):
            self.create_subscription(
                Bool, '/cooperative_transport/external_control',
                self._on_external_control, 10)
        self.declare_parameter('world_pose_topic', '')
        world = self.get_parameter('world_name').value
        pose_topic = str(self.get_parameter('world_pose_topic').value) or f'/world/{world}/pose/info'
        self.create_subscription(
            TFMessage, pose_topic,
            self._on_world_poses, 10)
        self.create_service(
            SetBool, '/cooperative_transport/set_enabled', self._on_set_enabled)

        self._state_publisher = self.create_publisher(
            String, '/cooperative_transport/state', 10)
        self._active_publisher = self.create_publisher(
            Bool, '/cooperative_transport/active', 10)
        self._payload_pose_publisher = self.create_publisher(
            PoseStamped, '/cooperative_transport/payload_pose', 10)
        self._admittance_data_publisher = self.create_publisher(
            Float64MultiArray,
            '/cooperative_transport/admittance_correction', 10)
        self._admittance_status_publisher = self.create_publisher(
            String, '/cooperative_transport/admittance_status', 10)

        rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info(
            f'Coordinator ready: /{self._names[0]}/rover_twist and '
            f'/{self._names[1]}/rover_twist')

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        if bool(self.get_parameter('use_world_model_poses').value):
            return
        pose = message.pose.pose
        self._poses[robot] = Pose2(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w),
        )
        self._odom_times[robot] = time.monotonic()

    def _on_world_poses(self, message: TFMessage) -> None:
        if not bool(self.get_parameter('use_world_model_poses').value):
            return
        now = time.monotonic()
        for transform in message.transforms:
            name = transform.child_frame_id
            if name not in self._names:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._poses[name] = Pose2(
                translation.x,
                translation.y,
                yaw_from_quaternion(
                    rotation.x, rotation.y, rotation.z, rotation.w),
            )
            self._odom_times[name] = now

    def _on_grasp_state(self, robot: str, message: Bool) -> None:
        self._attached[robot] = message.data

    def _on_wrench(self, robot: str, message: WrenchStamped) -> None:
        self._wrench_y[robot] = message.wrench.force.y
        self._wrench_times[robot] = time.monotonic()

    def _on_goal(self, message: PoseStamped) -> None:
        orientation = message.pose.orientation
        self._goal = Pose2(
            message.pose.position.x,
            message.pose.position.y,
            yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w),
        )
        now = time.monotonic()
        if now - self._last_goal_log_time >= 1.0:
            self.get_logger().info(
                f'Payload goal: x={self._goal.x:.3f}, '
                f'y={self._goal.y:.3f}, yaw={self._goal.yaw:.3f}')
            self._last_goal_log_time = now

    def _on_enable_topic(self, message: Bool) -> None:
        self._set_enabled(message.data)

    def _on_external_control(self, message: Bool) -> None:
        if message.data and not self._external_control:
            self._set_enabled(False)
            self._external_control = True
            self.get_logger().info('Base command control handed to pivot rotation')

    def _on_measured_payload(self, message: PoseStamped) -> None:
        pose = message.pose
        orientation = pose.orientation
        self._measured_payload = Pose2(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                orientation.x, orientation.y,
                orientation.z, orientation.w),
        )
        self._measured_payload_time = time.monotonic()

    def _feedback_pose(self) -> Optional[Pose2]:
        use_full_pose = bool(
            self.get_parameter('use_measured_payload_pose').value)
        use_position = bool(
            self.get_parameter('use_measured_payload_position').value)
        if not use_full_pose and not use_position:
            return None
        timeout = float(self.get_parameter('payload_pose_timeout').value)
        if (self._measured_payload is None
                or time.monotonic() - self._measured_payload_time > timeout):
            return None
        if use_full_pose:
            return self._measured_payload
        formation = estimate_payload_pose(
            self._poses[self._names[0]], self._poses[self._names[1]])
        weight = max(0.0, min(1.0, float(self.get_parameter(
            'measured_payload_position_weight').value)))
        return Pose2(
            formation.x + weight * (self._measured_payload.x - formation.x),
            formation.y + weight * (self._measured_payload.y - formation.y),
            formation.yaw,
        )

    def _on_set_enabled(self, request: SetBool.Request,
                        response: SetBool.Response) -> SetBool.Response:
        success, text = self._set_enabled(request.data)
        response.success = success
        response.message = text
        return response

    def _set_enabled(self, enabled: bool):
        if enabled and self._external_control:
            return False, 'External pivot controller owns base commands.'
        if enabled and self._emergency_stop:
            return False, 'Safety stop is latched; reset the safety monitor first.'
        if enabled and self._goal is None:
            return False, 'Publish /cooperative_transport/goal before enabling.'
        was_enabled = self._enabled
        self._enabled = enabled
        if enabled and not was_enabled:
            self._capture_payload_offset()
            self._capture_formation_distance_target()
            if bool(self.get_parameter('enable_lateral_admittance').value):
                self._capture_wrench_bias()
        if not enabled:
            self._alignment_recovering = False
            self._formation_distance_target = None
            self._reset_admittance()
            self._publish_stop()
        return True, 'Cooperative transport enabled.' if enabled else 'Stopped.'

    def _capture_payload_offset(self) -> None:
        if self._measured_payload is None or len(self._poses) != 2:
            self._payload_offset_local = None
            return
        formation = estimate_payload_pose(
            self._poses[self._names[0]], self._poses[self._names[1]])
        dx = self._measured_payload.x - formation.x
        dy = self._measured_payload.y - formation.y
        cosine = math.cos(formation.yaw)
        sine = math.sin(formation.yaw)
        self._payload_offset_local = (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
        )

    def _capture_formation_distance_target(self) -> None:
        """Fix the bilateral base separation for the current transport run."""
        if not bool(self.get_parameter(
                'enable_formation_distance_control').value):
            self._formation_distance_target = None
            return
        if len(self._poses) != 2:
            self._formation_distance_target = None
            return
        first = self._poses[self._names[0]]
        second = self._poses[self._names[1]]
        self._formation_distance_target = math.hypot(
            second.x - first.x, second.y - first.y)
        self.get_logger().info(
            'Formation-distance reference captured: '
            f'{self._formation_distance_target:.4f} m')

    def _formation_distance_error(self) -> Optional[float]:
        if self._formation_distance_target is None or len(self._poses) != 2:
            return None
        first = self._poses[self._names[0]]
        second = self._poses[self._names[1]]
        return math.hypot(second.x - first.x, second.y - first.y) - (
            self._formation_distance_target)

    def _wrenches_fresh(self) -> bool:
        timeout = float(
            self.get_parameter('admittance_sensor_timeout').value)
        now = time.monotonic()
        return all(
            name in self._wrench_y
            and now - self._wrench_times.get(name, 0.0) <= timeout
            for name in self._names)

    def _capture_wrench_bias(self) -> bool:
        """Use the steady grasp preload as the zero internal-force reference."""
        self._admittance.reset()
        self._wrench_bias = {}
        if not self._wrenches_fresh():
            return False
        self._wrench_bias = {
            name: self._wrench_y[name] for name in self._names
        }
        self.get_logger().info(
            'Lateral admittance wrench bias captured: '
            f'{self._names[0]}={self._wrench_bias[self._names[0]]:.2f} N, '
            f'{self._names[1]}={self._wrench_bias[self._names[1]]:.2f} N')
        return True

    def _reset_admittance(self) -> None:
        self._admittance.reset()
        self._wrench_bias = {}
        self._publish_admittance('INACTIVE', 0.0, 0.0, 0.0, 0.0)

    def _lateral_admittance_correction(self, dt: float) -> float:
        if not bool(self.get_parameter('enable_lateral_admittance').value):
            self._publish_admittance('DISABLED', 0.0, 0.0, 0.0, 0.0)
            return 0.0
        if not self._wrenches_fresh():
            self._admittance.reset()
            self._wrench_bias = {}
            self._publish_admittance('SENSOR_TIMEOUT', 0.0, 0.0, 0.0, 0.0)
            return 0.0
        if len(self._wrench_bias) != 2:
            self._capture_wrench_bias()
            self._publish_admittance('CALIBRATING', 0.0, 0.0, 0.0, 0.0)
            return 0.0
        raw_force_error = 0.5 * sum(
            self._wrench_y[name] - self._wrench_bias[name]
            for name in self._names)
        control_force = (
            float(self.get_parameter('admittance_force_sign').value)
            * raw_force_error)
        state = self._admittance.update(control_force, dt)
        self._publish_admittance(
            'ACTIVE', raw_force_error, state.filtered_force,
            state.displacement, state.velocity)
        return state.velocity

    def _publish_admittance(
            self, state: str, raw_force: float, filtered_force: float,
            displacement: float, velocity: float) -> None:
        data = Float64MultiArray()
        data.data = [raw_force, filtered_force, displacement, velocity]
        self._admittance_data_publisher.publish(data)
        status = String()
        status.data = (
            f'{state}; axis={self._names[1]}/local_y; '
            f'force_error={raw_force:.2f}N; '
            f'filtered_force={filtered_force:.2f}N; '
            f'displacement={displacement:.4f}m; '
            f'correction={velocity:.4f}m/s')
        self._admittance_status_publisher.publish(status)

    def _formation_alignment_error(self) -> Optional[Tuple[float, float]]:
        if self._measured_payload is None or self._payload_offset_local is None:
            return None
        formation = estimate_payload_pose(
            self._poses[self._names[0]], self._poses[self._names[1]])
        cosine = math.cos(formation.yaw)
        sine = math.sin(formation.yaw)
        desired_dx = (
            cosine * self._payload_offset_local[0]
            - sine * self._payload_offset_local[1])
        desired_dy = (
            sine * self._payload_offset_local[0]
            + cosine * self._payload_offset_local[1])
        return (
            self._measured_payload.x - formation.x - desired_dx,
            self._measured_payload.y - formation.y - desired_dy,
        )

    def _recovery_velocity(
            self, alignment_error: Optional[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        start = float(self.get_parameter('formation_recovery_start').value)
        stop = float(self.get_parameter('formation_recovery_stop').value)
        speed = float(self.get_parameter('formation_recovery_speed').value)
        if alignment_error is None or start <= 0.0 or speed <= 0.0:
            self._alignment_recovering = False
            return None
        magnitude = math.hypot(*alignment_error)
        if self._alignment_recovering:
            if magnitude <= stop:
                self._alignment_recovering = False
                return None
        elif magnitude >= start:
            self._alignment_recovering = True
        if not self._alignment_recovering or magnitude <= 1.0e-9:
            return None
        return (
            speed * alignment_error[0] / magnitude,
            speed * alignment_error[1] / magnitude,
        )

    def _on_emergency_stop(self, message: Bool) -> None:
        newly_stopped = message.data and not self._emergency_stop
        self._emergency_stop = message.data
        if message.data:
            self._enabled = False
            self._reset_admittance()
            self._publish_stop()
            if newly_stopped:
                self.get_logger().error('Emergency stop received; both bases stopped.')

    def _ready_state(self) -> Optional[str]:
        if self._emergency_stop:
            return 'EMERGENCY_STOP'
        if len(self._poses) != 2:
            return 'WAITING_FOR_ODOMETRY'
        timeout = float(self.get_parameter('odometry_timeout').value)
        now = time.monotonic()
        if any(now - self._odom_times.get(name, 0.0) > timeout for name in self._names):
            return 'ODOMETRY_TIMEOUT'
        if bool(self.get_parameter('require_attached').value):
            if not all(self._attached[name] is True for name in self._names):
                return 'WAITING_FOR_GRASP'
        if self._goal is None:
            return 'WAITING_FOR_GOAL'
        if not self._enabled:
            return 'IDLE'
        return None

    def _control_tick(self) -> None:
        if self._external_control:
            return
        now = time.monotonic()
        dt = max(1.0e-3, min(0.1, now - self._last_tick))
        self._last_tick = now
        blocked_state = self._ready_state()
        if blocked_state is not None:
            self._publish_stop()
            self._publish_state(blocked_state, False)
            return

        feedback = self._feedback_pose()
        alignment_error = self._formation_alignment_error()
        recovery_velocity = self._recovery_velocity(alignment_error)
        formation_distance_error = self._formation_distance_error()
        command1, command2, payload = rigid_body_commands(
            self._poses[self._names[0]], self._poses[self._names[1]], self._goal,
            float(self.get_parameter('position_kp').value),
            float(self.get_parameter('yaw_kp').value),
            float(self.get_parameter('heading_kp').value),
            float(self.get_parameter('max_linear_speed').value),
            float(self.get_parameter('max_angular_speed').value),
            feedback,
            alignment_error,
            float(self.get_parameter('formation_alignment_kp').value),
            float(self.get_parameter(
                'formation_alignment_max_fraction').value),
            recovery_velocity,
            self._formation_distance_target,
            float(self.get_parameter('formation_distance_kp').value),
            float(self.get_parameter(
                'formation_distance_max_correction_speed').value),
        )
        correction = self._lateral_admittance_correction(dt)
        corrected_x, corrected_y = clamp_vector(
            command2.x, command2.y + correction,
            float(self.get_parameter('max_linear_speed').value))
        command2 = Twist2(corrected_x, corrected_y, command2.yaw)
        self._publish_payload_pose(payload)

        position_error = math.hypot(self._goal.x - payload.x, self._goal.y - payload.y)
        yaw_error = abs(math.atan2(
            math.sin(self._goal.yaw - payload.yaw),
            math.cos(self._goal.yaw - payload.yaw)))
        if feedback is not None:
            position_error = max(position_error, math.hypot(
                self._goal.x - feedback.x, self._goal.y - feedback.y))
            yaw_error = max(yaw_error, abs(math.atan2(
                math.sin(self._goal.yaw - feedback.yaw),
                math.cos(self._goal.yaw - feedback.yaw))))
        alignment_ok = True
        if (alignment_error is not None
                and float(self.get_parameter('formation_alignment_kp').value) > 0.0):
            alignment_ok = math.hypot(*alignment_error) <= float(
                self.get_parameter('formation_alignment_tolerance').value)
        distance_ok = True
        if (formation_distance_error is not None
                and bool(self.get_parameter(
                    'enable_formation_distance_control').value)):
            distance_ok = abs(formation_distance_error) <= float(
                self.get_parameter('formation_distance_tolerance').value)
        if (position_error <= float(self.get_parameter('position_tolerance').value)
                and yaw_error <= float(self.get_parameter('yaw_tolerance').value)
                and alignment_ok and distance_ok):
            self._publish_stop()
            self._publish_state('HOLDING', True)
            return

        for name, target in zip(self._names, (command1, command2)):
            target = Twist2(
                target.x,
                target.y * float(
                    self.get_parameter('lateral_command_sign').value),
                target.yaw * float(
                    self.get_parameter('angular_command_sign').value),
            )
            command = rate_limit(
                self._last_commands[name], target, dt,
                float(self.get_parameter('max_linear_acceleration').value),
                float(self.get_parameter('max_angular_acceleration').value),
            )
            self._publish_command(name, command)
        self._publish_state(
            ('ALIGNMENT_RECOVERY' if self._alignment_recovering else
             'FORMATION_DISTANCE_RECOVERY' if not distance_ok else
             'TRANSPORT'),
            True)

    def _publish_command(self, robot: str, command: Twist2) -> None:
        message = Twist()
        message.linear.x = command.x
        message.linear.y = command.y
        message.angular.z = command.yaw
        self._command_publishers[robot].publish(message)
        self._last_commands[robot] = command

    def _publish_stop(self) -> None:
        zero = Twist2(0.0, 0.0, 0.0)
        for name in self._names:
            self._publish_command(name, zero)

    def _publish_state(self, state: str, active: bool) -> None:
        state_message = String()
        state_message.data = state
        self._state_publisher.publish(state_message)
        active_message = Bool()
        active_message.data = active
        self._active_publisher.publish(active_message)

    def _publish_payload_pose(self, payload: Pose2) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        message.pose.position.x = payload.x
        message.pose.position.y = payload.y
        message.pose.orientation.z = math.sin(0.5 * payload.yaw)
        message.pose.orientation.w = math.cos(0.5 * payload.yaw)
        self._payload_pose_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CooperativeCoordinator()
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


if __name__ == '__main__':
    main()
