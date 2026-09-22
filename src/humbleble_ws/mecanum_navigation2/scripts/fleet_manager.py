#!/usr/bin/env python3
"""Small namespace-aware dispatcher for multiple Nav2 NavigateToPose servers."""

from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.declare_parameter('robots', ['amir1', 'amir2'])
        self._robots = list(self.get_parameter('robots').value)
        # Node already owns an internal ``_clients`` container.  Keep action
        # clients under a distinct name so rclpy's executor state is not
        # overwritten.
        self._action_clients = {}
        self._goal_handles = {}
        self._status_publishers = {}

        for robot in self._robots:
            action_name = f'/{robot}/navigate_to_pose'
            goal_topic = f'/fleet/{robot}/goal_pose'
            status_topic = f'/fleet/{robot}/status'
            self._action_clients[robot] = ActionClient(
                self, NavigateToPose, action_name)
            self._status_publishers[robot] = self.create_publisher(String, status_topic, 10)
            self.create_subscription(
                PoseStamped, goal_topic, partial(self._goal_callback, robot), 10)

        self.create_service(Trigger, '/fleet/cancel_all', self._cancel_all)
        self.get_logger().info(
            'Fleet manager ready for: ' + ', '.join(self._robots))

    def _publish_status(self, robot, status):
        msg = String()
        msg.data = status
        self._status_publishers[robot].publish(msg)
        self.get_logger().info(f'{robot}: {status}')

    def _goal_callback(self, robot, pose):
        client = self._action_clients[robot]
        if not client.wait_for_server(timeout_sec=1.0):
            self._publish_status(robot, 'navigate_to_pose unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = client.send_goal_async(goal)
        future.add_done_callback(partial(self._goal_response, robot))
        self._publish_status(robot, 'goal sent')

    def _goal_response(self, robot, future):
        handle = future.result()
        if not handle.accepted:
            self._publish_status(robot, 'goal rejected')
            return
        self._goal_handles[robot] = handle
        self._publish_status(robot, 'goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(partial(self._goal_result, robot))

    def _goal_result(self, robot, future):
        status = future.result().status
        self._goal_handles.pop(robot, None)
        self._publish_status(robot, f'goal finished with status {status}')

    def _cancel_all(self, request, response):
        del request
        handles = list(self._goal_handles.items())
        for _, handle in handles:
            handle.cancel_goal_async()
        response.success = True
        response.message = f'cancel requested for {len(handles)} active goal(s)'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch and rclpy may observe the same SIGINT at slightly different
        # times.  Keep a late KeyboardInterrupt from turning a normal stop into
        # an error, and use the idempotent shutdown helper.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
