"""Move both AMIR arms to the common home posture after robot spawn."""

import math
import time
from typing import Dict, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .motion_math import (
    Pose2, Twist2, clamp_vector, grasp_world_position,
    normalize_angle, rate_limit, world_to_body_velocity)


ARM_JOINT_NAMES = [
    'Joint_1',
    'Joint_2',
    'Joint_3',
    'Joint_4',
    'Joint_5',
]

# Joint_2 + Joint_3 + Joint_4 == 0 keeps the gripper's longitudinal face
# parallel to the floor.  Joint_5 = pi / 2 rolls the upper and lower grasp
# faces parallel to the horizontal payload. Joint_3 and Joint_4 share the
# pitch correction.
Q_HOME = [0.0, 1.4, -1.6, 0.2, 1.570796]


class ArmHomePositioner(Node):
    """Synchronously set every spawned AMIR arm to the common home posture."""

    def __init__(self) -> None:
        super().__init__('arm_home_positioner')
        self.declare_parameter('robot_namespaces', ['amir1', 'amir2'])
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('motion_duration', 8.0)
        self.declare_parameter('joint_tolerance', 0.03)
        self.declare_parameter('position_tolerance', 0.025)
        self.declare_parameter('linear_velocity', 0.03)
        self.declare_parameter('angular_velocity', 0.05)
        self.declare_parameter('base_kp', 1.5)
        self.declare_parameter('heading_kp', 1.0)
        self.declare_parameter('linear_acceleration', 0.04)
        self.declare_parameter('angular_acceleration', 0.10)
        self.declare_parameter('startup_timeout', 30.0)
        # Initial positioning is performed before the payload is spawned, so the
        # default must leave both mobile bases still.  This can be enabled later
        # for a grasp-preserving repositioning operation.
        self.declare_parameter('compensate_base', False)

        self._names = tuple(
            str(name).strip('/') for name in
            self.get_parameter('robot_namespaces').value)
        if not self._names:
            raise ValueError('robot_namespaces must contain at least one robot')
        self._joints: Dict[str, Tuple[float, ...]] = {}
        self._joint_times: Dict[str, float] = {}
        self._poses: Dict[str, Pose2] = {}
        self._pose_times: Dict[str, float] = {}
        self._trajectory_publishers = {
            name: self.create_publisher(
                JointTrajectory, f'/{name}/arm_controller/joint_trajectory', 10)
            for name in self._names}
        self._arm_clients = {
            name: ActionClient(
                self, FollowJointTrajectory,
                f'/{name}/arm_controller/follow_joint_trajectory')
            for name in self._names}
        self._base_publishers = {
            name: self.create_publisher(Twist, f'/{name}/rover_twist', 10)
            for name in self._names}
        for name in self._names:
            self.create_subscription(
                JointState, f'/{name}/joint_states',
                lambda message, robot=name: self._on_joint_state(robot, message), 10)
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message), 10)

        self._phase = 'WAITING_FOR_STATE'
        self._started_at = time.monotonic()
        self._motion_started_at = 0.0
        self._anchors: Dict[str, Tuple[float, float, float]] = {}
        self._initial_yaws: Dict[str, float] = {}
        self._last_commands = {
            name: Twist2(0.0, 0.0, 0.0) for name in self._names}
        self._goal_handles = {}
        self._action_succeeded = set()
        self._action_error = ''
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value), self._tick)
        self.get_logger().info(
            f'Waiting to move all {self._names} to Q_HOME={Q_HOME} rad')

    def _on_joint_state(self, robot: str, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in ARM_JOINT_NAMES):
            self._joints[robot] = tuple(positions[name] for name in ARM_JOINT_NAMES)
            self._joint_times[robot] = time.monotonic()

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        pose = message.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        self._poses[robot] = Pose2(pose.position.x, pose.position.y, yaw)
        self._pose_times[robot] = time.monotonic()

    def _state_ready(self) -> bool:
        timeout = 0.5
        now = time.monotonic()
        return all(
            name in self._joints and name in self._poses
            and now - self._joint_times[name] <= timeout
            and now - self._pose_times[name] <= timeout
            for name in self._names)

    def _arm_servers_ready(self) -> bool:
        """A goal is sent only after every AMIR controller accepts actions."""
        return all(client.server_is_ready() for client in self._arm_clients.values())

    @staticmethod
    def _duration(seconds: float) -> Tuple[int, int]:
        whole = int(seconds)
        nanoseconds = int(round((seconds - whole) * 1.0e9))
        if nanoseconds >= 1_000_000_000:
            whole += 1
            nanoseconds -= 1_000_000_000
        return whole, nanoseconds

    def _home_trajectory(self) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(Q_HOME)
        point.time_from_start.sec, point.time_from_start.nanosec = self._duration(
            float(self.get_parameter('motion_duration').value))
        trajectory.points = [point]
        return trajectory

    def _send_home_goals(self) -> None:
        """Send one accepted FollowJointTrajectory goal to every AMIR."""
        trajectory = self._home_trajectory()
        for robot, client in self._arm_clients.items():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda result, name=robot: self._on_goal_response(name, result))

    def _on_goal_response(self, robot: str, future) -> None:
        try:
            handle = future.result()
        except Exception as exception:
            self._action_error = f'{robot} arm goal request failed: {exception}'
            return
        if handle is None or not handle.accepted:
            self._action_error = f'{robot} arm controller rejected Q_HOME'
            return
        self._goal_handles[robot] = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result, name=robot: self._on_action_result(name, result))

    def _on_action_result(self, robot: str, future) -> None:
        try:
            result = future.result().result
        except Exception as exception:
            self._action_error = f'{robot} arm trajectory failed: {exception}'
            return
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._action_error = (
                f'{robot} arm trajectory failed with error code {result.error_code}')
            return
        self._action_succeeded.add(robot)

    def _cancel_home_goals(self) -> None:
        for robot, handle in self._goal_handles.items():
            self._arm_clients[robot].async_cancel_goal(handle)

    def _publish_zero(self) -> None:
        zero = Twist()
        for publisher in self._base_publishers.values():
            publisher.publish(zero)

    def _hold_interrupted_arm(self) -> None:
        for robot, positions in self._joints.items():
            trajectory = JointTrajectory()
            trajectory.joint_names = list(ARM_JOINT_NAMES)
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start.nanosec = 200_000_000
            trajectory.points = [point]
            self._trajectory_publishers[robot].publish(trajectory)

    def _finish(self, success: bool, message: str) -> None:
        self._publish_zero()
        if not success:
            self._cancel_home_goals()
            self._hold_interrupted_arm()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        self._timer.cancel()
        rclpy.shutdown()

    def _tick(self) -> None:
        if self._phase == 'WAITING_FOR_STATE':
            if self._state_ready() and self._arm_servers_ready():
                self._anchors = {
                    robot: grasp_world_position(self._poses[robot], self._joints[robot])
                    for robot in self._names}
                self._initial_yaws = {
                    robot: self._poses[robot].yaw for robot in self._names}
                self._send_home_goals()
                self._phase = 'WAITING_FOR_GOAL_ACCEPTANCE'
                self.get_logger().info(
                    f'Sent Q_HOME={Q_HOME} rad goals to all arm controllers')
            elif time.monotonic() - self._started_at > float(
                    self.get_parameter('startup_timeout').value):
                self._finish(
                    False,
                    'Timed out waiting for all AMIR states and arm action servers')
            return

        if self._phase == 'WAITING_FOR_GOAL_ACCEPTANCE':
            if self._action_error:
                self._finish(False, self._action_error)
                return
            if len(self._goal_handles) != len(self._names):
                return
            self._phase = 'MOVING'
            self._motion_started_at = time.monotonic()
            self.get_logger().info('All AMIR arm controllers accepted Q_HOME')
            return

        now = time.monotonic()
        if self._action_error:
            self._finish(False, self._action_error)
            return

        # For initial Q_HOME positioning there is no payload to preserve.  The
        # FollowJointTrajectory result is the controller's authoritative proof
        # that each arm reached the requested radian targets.  Do not wait for
        # an unrelated mobile-base correction to converge.
        if not bool(self.get_parameter('compensate_base').value):
            self._publish_zero()
            if len(self._action_succeeded) == len(self._names):
                self._finish(True, 'All AMIR arms reached Q_HOME')
                return
            if now - self._motion_started_at > float(
                    self.get_parameter('motion_duration').value) + 30.0:
                self._finish(False, 'Timed out while moving all AMIR arms to Q_HOME')
            return

        max_position_error = 0.0
        max_joint_error = 0.0
        dt = 1.0 / float(self.get_parameter('control_rate').value)
        for robot in self._names:
            if robot not in self._joints or robot not in self._poses:
                self._finish(False, f'State lost for {robot}')
                return
            pose = self._poses[robot]
            grasp_x, grasp_y, _ = grasp_world_position(pose, self._joints[robot])
            error_x = self._anchors[robot][0] - grasp_x
            error_y = self._anchors[robot][1] - grasp_y
            world_vx, world_vy = clamp_vector(
                float(self.get_parameter('base_kp').value) * error_x,
                float(self.get_parameter('base_kp').value) * error_y,
                float(self.get_parameter('linear_velocity').value))
            local_vx, local_vy = world_to_body_velocity(
                world_vx, world_vy, pose.yaw)
            heading_error = normalize_angle(self._initial_yaws[robot] - pose.yaw)
            angular_velocity = float(self.get_parameter('angular_velocity').value)
            desired = Twist2(
                local_vx, local_vy,
                max(-angular_velocity, min(
                    angular_velocity,
                    float(self.get_parameter('heading_kp').value) * heading_error)))
            command = rate_limit(
                self._last_commands[robot], desired, dt,
                float(self.get_parameter('linear_acceleration').value),
                float(self.get_parameter('angular_acceleration').value))
            message = Twist()
            message.linear.x = command.x
            message.linear.y = command.y
            message.angular.z = command.yaw
            self._base_publishers[robot].publish(message)
            self._last_commands[robot] = command
            max_position_error = max(max_position_error, math.hypot(error_x, error_y))
            max_joint_error = max(
                max_joint_error,
                max(abs(target - current) for target, current in
                    zip(Q_HOME, self._joints[robot])))

        if (len(self._action_succeeded) == len(self._names)
                and max_position_error <= float(self.get_parameter('position_tolerance').value)
                and max_joint_error <= float(self.get_parameter('joint_tolerance').value)):
            self._finish(
                True,
                f'All AMIR arms reached Q_HOME; grasp-point error='
                f'{max_position_error:.4f} m')
            return
        if now - self._motion_started_at > float(
                self.get_parameter('motion_duration').value) + 30.0:
            self._finish(False, 'Timed out while moving all AMIR arms to Q_HOME')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmHomePositioner()
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
