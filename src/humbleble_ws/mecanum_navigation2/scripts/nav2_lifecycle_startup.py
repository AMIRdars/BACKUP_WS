#!/usr/bin/env python3
"""Start Nav2 lifecycle managers after DDS service discovery has settled."""

import sys

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


class Nav2LifecycleStartup(Node):
    """One-shot client that starts one or more Nav2 lifecycle managers."""

    def __init__(self):
        super().__init__('nav2_lifecycle_startup')
        self.declare_parameter(
            'manager_names', ['/lifecycle_manager_navigation'])
        self.declare_parameter('service_timeout', 30.0)
        self.declare_parameter('response_timeout', 120.0)

    def start_managers(self):
        manager_names = list(self.get_parameter('manager_names').value)
        service_timeout = float(self.get_parameter('service_timeout').value)
        response_timeout = float(self.get_parameter('response_timeout').value)

        for manager_name in manager_names:
            service_name = (
                f"{str(manager_name).rstrip('/')}/manage_nodes")
            client = self.create_client(
                ManageLifecycleNodes, service_name)

            self.get_logger().info(f'Waiting for {service_name}')
            if not client.wait_for_service(timeout_sec=service_timeout):
                self.get_logger().error(
                    f'Lifecycle service unavailable: {service_name}')
                return False

            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=response_timeout)

            if not future.done():
                self.get_logger().error(
                    f'Lifecycle startup timed out: {manager_name}')
                return False
            try:
                response = future.result()
            except Exception as error:
                self.get_logger().error(
                    f'Lifecycle startup failed for {manager_name}: {error}')
                return False
            if response is None or not response.success:
                self.get_logger().error(
                    f'Lifecycle manager rejected startup: {manager_name}')
                return False

            self.get_logger().info(
                f'Lifecycle manager is active: {manager_name}')

        return True


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LifecycleStartup()
    try:
        success = node.start_managers()
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
