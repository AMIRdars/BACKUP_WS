"""Expose cooperative payload motions as a cancellable ROS 2 Action.

The server deliberately does not command either base directly while moving.
It publishes object-frame goals to the existing ``cooperative_coordinator``,
which owns rigid-body velocity conversion and base command limits.
"""

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from coop_transport_interfaces.action import CooperativeMotion

from .motion_math import (
    Pose2, Twist2, arm_grasp_position, clamp_vector, normalize_angle,
    pivot_velocity, rate_limit, resolve_target_yaw,
    rotate_point_about_pivot, grasp_world_position, world_to_body_velocity)


class CooperativeMotionServer(Node):
    """Translate BT motion goals into safe object goals for the coordinator."""

    def __init__(self) -> None:
        super().__init__('cooperative_motion_server')
        self.declare_parameter('action_name', '/cooperative_motion')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('pose_timeout', 0.50)
        self.declare_parameter('action_timeout', 180.0)
        self.declare_parameter('require_grasp', True)
        self.declare_parameter('position_kp', 1.0)
        self.declare_parameter('yaw_kp', 1.0)
        self.declare_parameter('default_linear_velocity', 0.03)
        self.declare_parameter('default_angular_velocity', 0.05)
        self.declare_parameter('max_linear_velocity', 0.03)
        self.declare_parameter('max_angular_velocity', 0.05)
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('reposition_base_kp', 1.5)
        self.declare_parameter('reposition_heading_kp', 1.0)
        self.declare_parameter('reposition_linear_acceleration', 0.04)
        self.declare_parameter('reposition_angular_acceleration', 0.10)
        self.declare_parameter('reposition_vertical_warning', 0.03)

        self._names = tuple(
            self.get_parameter(name).value.strip('/') for name in
            ('robot1_namespace', 'robot2_namespace'))
        self._payload_pose: Optional[Pose2] = None
        self._payload_time = 0.0
        self._robot_poses: Dict[str, Pose2] = {}
        self._robot_times: Dict[str, float] = {}
        self._joint_positions: Dict[str, Tuple[float, ...]] = {}
        self._joint_times: Dict[str, float] = {}
        self._grasped: Dict[str, Optional[bool]] = {
            name: None for name in self._names}
        # The execute callback waits for fresh pose subscriptions, therefore
        # Action and subscription callbacks must be allowed to run concurrently.
        self._callback_group = ReentrantCallbackGroup()

        self._goal_publisher = self.create_publisher(
            PoseStamped, '/cooperative_transport/goal', 10,
            callback_group=self._callback_group)
        self._enable_publisher = self.create_publisher(
            Bool, '/cooperative_transport/enable', 10,
            callback_group=self._callback_group)
        self._stop_publishers = {
            name: self.create_publisher(
                Twist, f'/{name}/rover_twist', 10,
                callback_group=self._callback_group)
            for name in self._names}
        self._arm_publishers = {
            name: self.create_publisher(
                JointTrajectory, f'/{name}/arm_controller/joint_trajectory', 10,
                callback_group=self._callback_group)
            for name in self._names}
        self.create_subscription(
            PoseStamped, '/cooperative_transport/payload_pose',
            self._on_payload_pose, 10, callback_group=self._callback_group)
        for name in self._names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message), 10,
                callback_group=self._callback_group)
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp(robot, message), 10,
                callback_group=self._callback_group)
            self.create_subscription(
                JointState, f'/{name}/joint_states',
                lambda message, robot=name: self._on_joint_state(robot, message),
                10, callback_group=self._callback_group)

        self._server = ActionServer(
            self, CooperativeMotion,
            self.get_parameter('action_name').value,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group)
        self.get_logger().info(
            'Ready: /cooperative_motion controls payload motion and grasp-preserving arm repositioning.')

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_payload_pose(self, message: PoseStamped) -> None:
        self._payload_pose = Pose2(
            message.pose.position.x, message.pose.position.y,
            self._yaw_from_quaternion(message.pose.orientation))
        self._payload_time = time.monotonic()

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        pose = message.pose.pose
        self._robot_poses[robot] = Pose2(
            pose.position.x, pose.position.y,
            self._yaw_from_quaternion(pose.orientation))
        self._robot_times[robot] = time.monotonic()

    def _on_grasp(self, robot: str, message: Bool) -> None:
        self._grasped[robot] = message.data

    def _on_joint_state(self, robot: str, message: JointState) -> None:
        required = ('Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5')
        positions = dict(zip(message.name, message.position))
        if not all(joint in positions for joint in required):
            return
        self._joint_positions[robot] = tuple(positions[joint] for joint in required)
        self._joint_times[robot] = time.monotonic()

    def _formation_pose(self) -> Optional[Pose2]:
        if len(self._robot_poses) != 2:
            return None
        now = time.monotonic()
        timeout = float(self.get_parameter('pose_timeout').value)
        if any(now - self._robot_times.get(name, 0.0) > timeout
               for name in self._names):
            return None
        first, second = (self._robot_poses[name] for name in self._names)
        return Pose2(
            0.5 * (first.x + second.x), 0.5 * (first.y + second.y),
            math.atan2(second.y - first.y, second.x - first.x))

    def _current_pose(self) -> Optional[Pose2]:
        # ``payload_pose`` is the best available estimate when the coordinator
        # is active. Before it is enabled, use the already validated formation
        # estimate so that an Action can establish its own initial object pose.
        timeout = float(self.get_parameter('pose_timeout').value)
        if (self._payload_pose is not None
                and time.monotonic() - self._payload_time <= timeout):
            return self._payload_pose
        return self._formation_pose()

    def _goal_callback(self, goal: CooperativeMotion.Goal) -> GoalResponse:
        valid_motions = {
            CooperativeMotion.Goal.MOVE_STRAIGHT,
            CooperativeMotion.Goal.MOVE_BACKWARD,
            CooperativeMotion.Goal.ROTATE_IN_PLACE,
            CooperativeMotion.Goal.ROTATE_AROUND_PIVOT,
            CooperativeMotion.Goal.REPOSITION_ARMS,
        }
        values = (goal.target_x, goal.target_y, goal.target_yaw,
                  goal.pivot_x, goal.pivot_y, goal.rotation_angle,
                  goal.linear_velocity, goal.angular_velocity,
                  goal.position_tolerance, goal.angle_tolerance,
                  goal.joint_1_target, goal.joint_2_target,
                  goal.joint_3_target, goal.joint_4_target,
                  goal.joint_5_target, goal.joint_tolerance,
                  goal.arm_motion_duration)
        if goal.motion_type not in valid_motions or goal.angle_mode not in (
                CooperativeMotion.Goal.ANGLE_ABSOLUTE,
                CooperativeMotion.Goal.ANGLE_RELATIVE):
            self.get_logger().warn('Rejected CooperativeMotion: unsupported motion or angle mode.')
            return GoalResponse.REJECT
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('Rejected CooperativeMotion: all goal values must be finite.')
            return GoalResponse.REJECT
        if (goal.position_tolerance <= 0.0 or goal.angle_tolerance <= 0.0):
            self.get_logger().warn('Rejected CooperativeMotion: tolerances must be positive.')
            return GoalResponse.REJECT
        if (goal.motion_type == CooperativeMotion.Goal.REPOSITION_ARMS
                and (goal.joint_tolerance <= 0.0
                     or goal.arm_motion_duration <= 0.0)):
            self.get_logger().warn(
                'Rejected arm reposition: joint tolerance and duration must be positive.')
            return GoalResponse.REJECT
        joint_limits = ((-2.96706, 2.96706), (0.0, 2.356194),
                        (-2.792527, 0.0), (-2.094395, 1.308997),
                        (-2.75762, 2.75762))
        targets = (goal.joint_1_target, goal.joint_2_target,
                   goal.joint_3_target, goal.joint_4_target,
                   goal.joint_5_target)
        if (goal.motion_type == CooperativeMotion.Goal.REPOSITION_ARMS
                and any(target < lower or target > upper
                        for target, (lower, upper) in zip(targets, joint_limits))):
            self.get_logger().warn('Rejected CooperativeMotion: arm target is outside AMIR joint limits.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_object_goal(self, pose: Pose2) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('world_frame').value
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = math.sin(0.5 * pose.yaw)
        message.pose.orientation.w = math.cos(0.5 * pose.yaw)
        self._goal_publisher.publish(message)

    def _set_enabled(self, enabled: bool) -> None:
        message = Bool()
        message.data = enabled
        self._enable_publisher.publish(message)

    def _stop_robots(self) -> None:
        """Disable the coordinator and issue an immediate zero-command backup."""
        # Shutdown callbacks can run after the ROS context is invalidated.
        # Best effort is sufficient then; while the context is live every
        # normal success/failure/cancel path publishes all three commands.
        try:
            self._set_enabled(False)
            stop = Twist()
            for publisher in self._stop_publishers.values():
                publisher.publish(stop)
        except Exception as exception:  # rclpy raises RCLError after shutdown
            self.get_logger().debug(f'Unable to publish final stop: {exception}')

    def _grasp_is_valid(self) -> bool:
        return (not bool(self.get_parameter('require_grasp').value)
                or all(self._grasped[name] is True for name in self._names))

    def _fresh_joint_positions(self, robot: str) -> Optional[Tuple[float, ...]]:
        positions = self._joint_positions.get(robot)
        if positions is None:
            return None
        if (time.monotonic() - self._joint_times.get(robot, 0.0)
                > float(self.get_parameter('pose_timeout').value)):
            return None
        return positions

    @staticmethod
    def _target_joints(goal) -> Tuple[float, ...]:
        return (
            goal.joint_1_target, goal.joint_2_target, goal.joint_3_target,
            goal.joint_4_target, goal.joint_5_target)

    @staticmethod
    def _trajectory_duration(seconds: float) -> Tuple[int, int]:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1.0e9))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return whole_seconds, nanoseconds

    def _publish_arm_trajectory(
            self, robot: str, positions: Tuple[float, ...], duration: float) -> None:
        message = JointTrajectory()
        message.joint_names = [
            'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec, point.time_from_start.nanosec = (
            self._trajectory_duration(duration))
        message.points = [point]
        self._arm_publishers[robot].publish(message)

    def _publish_arm_hold(self) -> None:
        """Replace an interrupted arm trajectory with a short hold point."""
        for robot in self._names:
            positions = self._fresh_joint_positions(robot)
            if positions is not None:
                self._publish_arm_trajectory(robot, positions, 0.20)

    def _publish_reposition_command(self, robot: str, command: Twist2) -> None:
        message = Twist()
        message.linear.x = command.x
        message.linear.y = command.y
        message.angular.z = command.yaw
        self._stop_publishers[robot].publish(message)

    def _execute_arm_reposition(self, goal_handle, initial: Pose2):
        """Move both arms and compensate the two bases for grasp-point displacement."""
        current_joints = {
            robot: self._fresh_joint_positions(robot) for robot in self._names}
        if any(positions is None for positions in current_joints.values()):
            return self._finish(
                goal_handle, False,
                'joint state unavailable at arm reposition start', initial,
                stop_arm=False)

        # Freeze each grasp point in world coordinates before changing either
        # arm.  This also handles the opposite headings of AMIR1 and AMIR2.
        anchors = {
            robot: grasp_world_position(
                self._robot_poses[robot], current_joints[robot])
            for robot in self._names}
        initial_base_yaws = {
            robot: self._robot_poses[robot].yaw for robot in self._names}
        target_joints = self._target_joints(goal_handle.request)
        target_tcp = arm_grasp_position(target_joints)
        current_tcp_z = arm_grasp_position(current_joints[self._names[0]])[2]
        vertical_change = abs(target_tcp[2] - current_tcp_z)
        if vertical_change > float(self.get_parameter('reposition_vertical_warning').value):
            self.get_logger().warn(
                'Requested arm posture changes TCP height by '
                f'{vertical_change:.3f} m; the planar bases can only compensate x/y.')

        self._set_enabled(False)
        duration = float(goal_handle.request.arm_motion_duration)
        for robot in self._names:
            self._publish_arm_trajectory(robot, target_joints, duration)

        max_linear = min(
            float(self.get_parameter('max_linear_velocity').value),
            goal_handle.request.linear_velocity
            if goal_handle.request.linear_velocity > 0.0
            else float(self.get_parameter('default_linear_velocity').value))
        max_angular = min(
            float(self.get_parameter('max_angular_velocity').value),
            goal_handle.request.angular_velocity
            if goal_handle.request.angular_velocity > 0.0
            else float(self.get_parameter('default_angular_velocity').value))
        if max_linear <= 0.0 or max_angular <= 0.0:
            return self._finish(
                goal_handle, False, 'configured velocity limit is not positive',
                initial, stop_arm=True)

        last_commands = {
            robot: Twist2(0.0, 0.0, 0.0) for robot in self._names}
        last_tick = time.monotonic()
        started = last_tick
        rate = self.create_rate(float(self.get_parameter('control_rate').value))
        try:
            while rclpy.ok():
                if not self._grasp_is_valid():
                    return self._finish(
                        goal_handle, False, 'grasp state lost during arm reposition',
                        self._formation_pose(), stop_arm=True)
                if goal_handle.is_cancel_requested:
                    return self._finish(
                        goal_handle, False, 'goal cancelled', self._formation_pose(),
                        stop_arm=True)
                if time.monotonic() - started > float(
                        self.get_parameter('action_timeout').value):
                    return self._finish(
                        goal_handle, False, 'arm reposition timeout',
                        self._formation_pose(), stop_arm=True)

                now = time.monotonic()
                dt = min(0.1, max(0.0, now - last_tick))
                last_tick = now
                max_tcp_error = 0.0
                max_heading_error = 0.0
                max_joint_error = 0.0
                for robot in self._names:
                    pose = self._robot_poses.get(robot)
                    joint_positions = self._fresh_joint_positions(robot)
                    if (pose is None or
                            now - self._robot_times.get(robot, 0.0) >
                            float(self.get_parameter('pose_timeout').value) or
                            joint_positions is None):
                        return self._finish(
                            goal_handle, False,
                            f'pose or joint state timeout for {robot}',
                            self._formation_pose(), stop_arm=True)

                    tcp_x, tcp_y, _ = grasp_world_position(pose, joint_positions)
                    error_x, error_y = anchors[robot][0] - tcp_x, anchors[robot][1] - tcp_y
                    world_vx, world_vy = clamp_vector(
                        float(self.get_parameter('reposition_base_kp').value) * error_x,
                        float(self.get_parameter('reposition_base_kp').value) * error_y,
                        max_linear)
                    local_vx, local_vy = world_to_body_velocity(
                        world_vx, world_vy, pose.yaw)
                    heading_error = normalize_angle(
                        initial_base_yaws[robot] - pose.yaw)
                    target_command = Twist2(
                        local_vx, local_vy,
                        max(-max_angular, min(max_angular,
                            float(self.get_parameter('reposition_heading_kp').value)
                            * heading_error)))
                    command = rate_limit(
                        last_commands[robot], target_command, dt,
                        float(self.get_parameter(
                            'reposition_linear_acceleration').value),
                        float(self.get_parameter(
                            'reposition_angular_acceleration').value))
                    self._publish_reposition_command(robot, command)
                    last_commands[robot] = command
                    max_tcp_error = max(max_tcp_error, math.hypot(error_x, error_y))
                    max_heading_error = max(max_heading_error, abs(heading_error))
                    max_joint_error = max(
                        max_joint_error,
                        max(abs(target - current) for target, current in
                            zip(target_joints, joint_positions)))

                current_pose = self._formation_pose()
                feedback = CooperativeMotion.Feedback()
                if current_pose is not None:
                    feedback.current_x, feedback.current_y, feedback.current_yaw = current_pose
                feedback.position_error = max_tcp_error
                feedback.angle_error = max_heading_error
                feedback.command_vx = max_tcp_error
                feedback.command_vy = max_joint_error
                feedback.command_wz = max_heading_error
                goal_handle.publish_feedback(feedback)

                if (max_joint_error <= goal_handle.request.joint_tolerance
                        and max_tcp_error <= goal_handle.request.position_tolerance
                        and max_heading_error <= goal_handle.request.angle_tolerance):
                    return self._finish(
                        goal_handle, True,
                        'arm targets reached while grasp points were held',
                        current_pose, stop_arm=False)
                rate.sleep()
        except Exception as exception:
            self.get_logger().error(f'Arm reposition failed: {exception}')
            return self._finish(
                goal_handle, False, f'controller error: {exception}',
                self._formation_pose(), stop_arm=True)
        finally:
            rate.destroy()
        return self._finish(
            goal_handle, False, 'ROS shutdown', self._formation_pose(), stop_arm=True)

    def _finish(self, goal_handle, success: bool, message: str,
                pose: Optional[Pose2], stop_arm: bool = False) -> CooperativeMotion.Result:
        if stop_arm:
            self._publish_arm_hold()
        self._stop_robots()
        result = CooperativeMotion.Result()
        result.success = success
        result.message = message
        if pose is not None:
            result.final_x, result.final_y, result.final_yaw = pose
        if success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute(self, goal_handle) -> CooperativeMotion.Result:
        goal = goal_handle.request
        initial = self._current_pose()
        if initial is None:
            return self._finish(goal_handle, False, 'object pose unavailable at action start', None)
        if not self._grasp_is_valid():
            return self._finish(goal_handle, False, 'both robots must be in grasp state', initial)

        if goal.motion_type == CooperativeMotion.Goal.REPOSITION_ARMS:
            return self._execute_arm_reposition(goal_handle, initial)

        target_yaw = resolve_target_yaw(
            initial.yaw, goal.angle_mode, goal.target_yaw, goal.rotation_angle,
            CooperativeMotion.Goal.ANGLE_ABSOLUTE)
        is_pivot_motion = goal.motion_type in (
            CooperativeMotion.Goal.ROTATE_IN_PLACE,
            CooperativeMotion.Goal.ROTATE_AROUND_PIVOT)
        pivot_x, pivot_y = goal.pivot_x, goal.pivot_y
        if goal.motion_type == CooperativeMotion.Goal.ROTATE_IN_PLACE:
            pivot_x, pivot_y = initial.x, initial.y

        max_linear = min(
            float(self.get_parameter('max_linear_velocity').value),
            goal.linear_velocity if goal.linear_velocity > 0.0 else
            float(self.get_parameter('default_linear_velocity').value))
        max_angular = min(
            float(self.get_parameter('max_angular_velocity').value),
            goal.angular_velocity if goal.angular_velocity > 0.0 else
            float(self.get_parameter('default_angular_velocity').value))
        if max_linear <= 0.0 or max_angular <= 0.0:
            return self._finish(goal_handle, False, 'configured velocity limit is not positive', initial)

        # Keep target steps bounded. This lets the existing coordinator do
        # low-level tracking while preserving the circular pivot trajectory.
        reference_angle = 0.0
        target_delta = normalize_angle(target_yaw - initial.yaw)
        last_tick = time.monotonic()
        started = last_tick
        rate = self.create_rate(float(self.get_parameter('control_rate').value))
        try:
            while rclpy.ok():
                current = self._current_pose()
                if current is None:
                    return self._finish(goal_handle, False, 'object pose timeout', None)
                if not self._grasp_is_valid():
                    return self._finish(goal_handle, False, 'grasp state lost', current)
                if goal_handle.is_cancel_requested:
                    return self._finish(goal_handle, False, 'goal cancelled', current)
                if time.monotonic() - started > float(
                        self.get_parameter('action_timeout').value):
                    return self._finish(goal_handle, False, 'action timeout', current)

                now = time.monotonic()
                dt = min(0.1, max(0.0, now - last_tick))
                last_tick = now
                if is_pivot_motion:
                    reference_angle += math.copysign(
                        min(abs(target_delta - reference_angle), max_angular * dt),
                        target_delta - reference_angle)
                    desired_x, desired_y = rotate_point_about_pivot(
                        initial, pivot_x, pivot_y, reference_angle)
                    desired_yaw = normalize_angle(initial.yaw + reference_angle)
                    command = pivot_velocity(
                        current, pivot_x, pivot_y, desired_x, desired_y,
                        desired_yaw,
                        float(self.get_parameter('position_kp').value),
                        float(self.get_parameter('yaw_kp').value), max_angular)
                else:
                    desired_x, desired_y, desired_yaw = (
                        goal.target_x, goal.target_y, target_yaw)
                    dx, dy = desired_x - current.x, desired_y - current.y
                    distance = math.hypot(dx, dy)
                    if distance > 1.0e-9:
                        scale = min(max_linear, distance) / distance
                        command = (dx * scale, dy * scale, 0.0)
                    else:
                        command = (0.0, 0.0, 0.0)

                desired = Pose2(desired_x, desired_y, desired_yaw)
                self._publish_object_goal(desired)
                self._set_enabled(True)
                position_error = math.hypot(desired_x - current.x, desired_y - current.y)
                angle_error = abs(normalize_angle(target_yaw - current.yaw))
                feedback = CooperativeMotion.Feedback()
                feedback.current_x, feedback.current_y, feedback.current_yaw = current
                feedback.position_error = position_error
                feedback.angle_error = angle_error
                feedback.command_vx, feedback.command_vy, feedback.command_wz = command
                goal_handle.publish_feedback(feedback)

                # For pivot turns compare against the final circular endpoint,
                # not merely the latest incremental reference.
                if is_pivot_motion:
                    final_x, final_y = rotate_point_about_pivot(
                        initial, pivot_x, pivot_y, target_delta)
                    final_position_error = math.hypot(final_x - current.x,
                                                      final_y - current.y)
                    reached_reference = abs(normalize_angle(
                        target_delta - reference_angle)) <= 1.0e-6
                else:
                    final_position_error = position_error
                    reached_reference = True
                if (reached_reference
                        and final_position_error <= goal.position_tolerance
                        and angle_error <= goal.angle_tolerance):
                    return self._finish(goal_handle, True, 'goal reached', current)
                rate.sleep()
        except Exception as exception:
            self.get_logger().error(f'CooperativeMotion failed: {exception}')
            return self._finish(goal_handle, False, f'controller error: {exception}',
                                self._current_pose())
        finally:
            rate.destroy()
        return self._finish(goal_handle, False, 'ROS shutdown', self._current_pose())

    def destroy_node(self):
        self._stop_robots()
        self._server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CooperativeMotionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robots()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
