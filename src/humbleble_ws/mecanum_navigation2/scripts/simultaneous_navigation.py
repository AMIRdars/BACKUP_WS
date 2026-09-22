#!/usr/bin/env python3
"""Send different Nav2 goals to amir1 and amir2 and verify concurrent motion."""

import math
import time
from functools import partial

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class SimultaneousNavigation(Node):
    """One-shot verifier for simultaneous, namespace-separated navigation."""

    def __init__(self):
        super().__init__('simultaneous_navigation')

        self.declare_parameter('amir1_x', 2.0)
        self.declare_parameter('amir1_y', -2.0)
        self.declare_parameter('amir1_yaw', 0.0)
        self.declare_parameter('amir2_x', 2.0)
        self.declare_parameter('amir2_y', 2.0)
        self.declare_parameter('amir2_yaw', 0.0)
        self.declare_parameter('server_timeout', 30.0)
        self.declare_parameter('result_timeout', 180.0)
        self.declare_parameter('motion_window', 1.0)

        self._robots = ('amir1', 'amir2')
        self._action_clients = {
            robot: ActionClient(
                self, NavigateToPose, f'/{robot}/navigate_to_pose')
            for robot in self._robots
        }
        self._goals = {
            robot: (
                float(self.get_parameter(f'{robot}_x').value),
                float(self.get_parameter(f'{robot}_y').value),
                float(self.get_parameter(f'{robot}_yaw').value),
            )
            for robot in self._robots
        }
        self._subscriptions = [
            self.create_subscription(
                Twist,
                f'/{robot}/cmd_vel',
                partial(self._motion_callback, robot),
                10,
            )
            for robot in self._robots
        ]
        self._last_motion = {}
        self._motion_overlap_confirmed = False
        self._goal_handles = {}
        self._results = {}

    def wait_for_servers(self):
        timeout = float(self.get_parameter('server_timeout').value)
        deadline = time.monotonic() + timeout
        waiting = set(self._robots)

        while rclpy.ok() and waiting and time.monotonic() < deadline:
            for robot in tuple(waiting):
                if self._action_clients[robot].wait_for_server(timeout_sec=0.2):
                    waiting.remove(robot)
                    self.get_logger().info(
                        f'/{robot}/navigate_to_pose is ready')

        if waiting:
            self.get_logger().error(
                'Action server unavailable: ' + ', '.join(sorted(waiting)))
            return False
        return True

    def send_goals(self):
        send_times = {}

        for robot in self._robots:
            x, y, yaw = self._goals[robot]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_times[robot] = time.monotonic()
            future = self._action_clients[robot].send_goal_async(goal)
            future.add_done_callback(partial(self._goal_response, robot))
            self.get_logger().info(
                f'{robot}: goal sent: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        skew_ms = abs(send_times['amir2'] - send_times['amir1']) * 1000.0
        self.get_logger().info(f'Goal send skew: {skew_ms:.1f} ms')

    def _goal_response(self, robot, future):
        try:
            handle = future.result()
        except Exception as error:  # rclpy reports transport failures here
            self.get_logger().error(f'{robot}: goal request failed: {error}')
            self._results[robot] = None
            return

        if not handle.accepted:
            self.get_logger().error(f'{robot}: goal rejected')
            self._results[robot] = GoalStatus.STATUS_ABORTED
            return

        self._goal_handles[robot] = handle
        self.get_logger().info(f'{robot}: goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(partial(self._goal_result, robot))

    def _goal_result(self, robot, future):
        try:
            status = future.result().status
        except Exception as error:  # rclpy reports transport failures here
            self.get_logger().error(f'{robot}: result failed: {error}')
            status = None
        self._results[robot] = status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{robot}: goal succeeded')
        else:
            self.get_logger().error(f'{robot}: goal ended with status {status}')

    def _motion_callback(self, robot, message):
        moving = (
            abs(message.linear.x) > 0.01
            or abs(message.linear.y) > 0.01
            or abs(message.angular.z) > 0.01
        )
        if not moving:
            return

        self._last_motion[robot] = time.monotonic()
        if self._motion_overlap_confirmed:
            return
        if not all(name in self._last_motion for name in self._robots):
            return

        window = float(self.get_parameter('motion_window').value)
        times = [self._last_motion[name] for name in self._robots]
        if max(times) - min(times) <= window:
            self._motion_overlap_confirmed = True
            self.get_logger().info(
                'SIMULTANEOUS MOTION CONFIRMED: '
                '/amir1/cmd_vel and /amir2/cmd_vel are both active')

    def wait_for_results(self):
        timeout = float(self.get_parameter('result_timeout').value)
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and len(self._results) < len(self._robots)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if len(self._results) < len(self._robots):
            unfinished = [
                robot for robot in self._robots if robot not in self._results]
            self.get_logger().error(
                'Timed out waiting for: ' + ', '.join(unfinished))
            for robot in unfinished:
                handle = self._goal_handles.get(robot)
                if handle is not None:
                    handle.cancel_goal_async()
            return False

        succeeded = all(
            self._results.get(robot) == GoalStatus.STATUS_SUCCEEDED
            for robot in self._robots
        )
        if succeeded and self._motion_overlap_confirmed:
            self.get_logger().info(
                'SIMULTANEOUS NAVIGATION PASSED: '
                'both robots moved concurrently and reached different goals')
            return True

        if succeeded:
            self.get_logger().error(
                'Both goals succeeded, but concurrent cmd_vel was not observed')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = SimultaneousNavigation()
    exit_code = 1
    try:
        if node.wait_for_servers():
            node.send_goals()
            exit_code = 0 if node.wait_for_results() else 1
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted')
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
