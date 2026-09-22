"""Remove the temporary payload support after a friction grasp is established."""

from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from std_msgs.msg import Bool, String


class GraspSupportRemover(Node):
    """Delete only the temporary support once both robots report HOLDING."""

    def __init__(self) -> None:
        super().__init__('grasp_support_remover')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('world_name', 'cooperative_transport_friction')
        self.declare_parameter('support_model_name', 'grasp_support')
        self.declare_parameter('removal_delay', 1.0)
        self.declare_parameter('retry_period', 1.0)
        self.declare_parameter('require_physical_contact', True)

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._grasped: Dict[str, bool] = {
            name: False for name in self._names
        }
        self._holding_since: Optional[float] = None
        self._friction_grasp_holding = False
        self._physical_contact_verified = False
        self._last_attempt = float('-inf')
        self._request_pending = False
        self._removed = False

        world = str(self.get_parameter('world_name').value)
        self._delete_client = self.create_client(
            DeleteEntity, f'/world/{world}/remove')
        for name in self._names:
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp(robot, message), 10)
        self.create_subscription(
            String, '/cooperative_transport/friction_grasp_status',
            self._on_friction_grasp_status, 10)
        self.create_subscription(
            String, '/cooperative_transport/contact_status',
            self._on_contact_status, 10)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_publisher = self.create_publisher(
            Bool, '/cooperative_transport/support_removed', status_qos)
        self._publish_status()
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'Temporary grasp support will be removed only after both robots '
            'are HOLDING and physical finger contact is verified.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_grasp(self, robot: str, message: Bool) -> None:
        self._grasped[robot] = message.data

    def _on_friction_grasp_status(self, message: String) -> None:
        """Accept only the manager's final, verified HOLDING state."""
        self._friction_grasp_holding = (
            message.data.split(';', 1)[0].strip() == 'HOLDING')

    def _on_contact_status(self, message: String) -> None:
        self._physical_contact_verified = message.data == 'CONTACT_OK'

    def _publish_status(self) -> None:
        message = Bool()
        message.data = self._removed
        self._status_publisher.publish(message)

    def _tick(self) -> None:
        if self._removed:
            return
        now = self._now()
        physical_contact_ready = (
            not bool(self.get_parameter('require_physical_contact').value)
            or self._physical_contact_verified)
        ready_to_remove = (
            all(self._grasped.values())
            and self._friction_grasp_holding
            and physical_contact_ready)
        if not ready_to_remove:
            self._holding_since = None
            return
        if self._holding_since is None:
            self._holding_since = now
            return
        if now - self._holding_since < float(
                self.get_parameter('removal_delay').value):
            return
        if self._request_pending:
            return
        retry_period = float(self.get_parameter('retry_period').value)
        if now - self._last_attempt < retry_period:
            return
        self._last_attempt = now
        if not self._delete_client.service_is_ready():
            self.get_logger().warning(
                'Gazebo remove service is not ready; support removal will retry.')
            return

        request = DeleteEntity.Request()
        request.entity.name = str(
            self.get_parameter('support_model_name').value)
        request.entity.type = Entity.MODEL
        self._request_pending = True
        future = self._delete_client.call_async(request)
        future.add_done_callback(self._on_removed)

    def _on_removed(self, future) -> None:
        self._request_pending = False
        try:
            response = future.result()
        except Exception as exception:  # noqa: B902 - ROS future exception
            self.get_logger().error(
                f'Gazebo support-removal request failed: {exception}')
            return
        if not response.success:
            self.get_logger().warning(
                'Gazebo did not remove the support; request will retry.')
            return
        self._removed = True
        self._publish_status()
        name = self.get_parameter('support_model_name').value
        self.get_logger().info(
            f'Removed temporary support model "{name}"; transport may start.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspSupportRemover()
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
