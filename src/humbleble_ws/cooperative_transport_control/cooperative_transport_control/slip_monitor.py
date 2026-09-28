"""Monitor payload slip relative to the two-robot formation."""

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage

from .kinematics import yaw_from_quaternion


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


class SlipMonitor(Node):
    """Latch a fault when payload-to-formation pose changes excessively."""

    def __init__(self) -> None:
        super().__init__('slip_monitor')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('world_name', 'cooperative_transport_friction')
        self.declare_parameter('payload_name', 'cooperative_payload')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter(
            'transport_frame', 'cooperative_transport_frame')
        self.declare_parameter('horizontal_slip_limit', 0.030)
        self.declare_parameter('vertical_slip_limit', 0.025)
        self.declare_parameter('yaw_slip_limit', math.radians(4.0))
        self.declare_parameter('activation_grace_period', 0.5)
        self.declare_parameter('use_world_model_poses', False)

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._base_positions: Dict[str, Tuple[float, float]] = {}
        self._payload: Optional[Tuple[float, float, float, float]] = None
        self._grasped = {name: False for name in self._names}
        self._active = False
        self._active_since = 0.0
        self._baseline: Optional[Tuple[float, float, float, float]] = None
        self._current_error = (0.0, 0.0, 0.0, 0.0)
        self._fault = False
        self._reason = 'OK'

        for name in self._names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message), 10)
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp(robot, message), 10)
        self.declare_parameter('world_pose_topic', '')
        world = self.get_parameter('world_name').value
        pose_topic = str(self.get_parameter('world_pose_topic').value) or f'/world/{world}/pose/info'
        self.create_subscription(
            TFMessage, pose_topic, self._on_poses, 10)
        self.create_subscription(
            Bool, '/cooperative_transport/active', self._on_active, 10)

        self._fault_publisher = self.create_publisher(
            Bool, '/cooperative_transport/slip_fault', 10)
        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/slip_status', 10)
        self._measured_pose_publisher = self.create_publisher(
            PoseStamped, '/cooperative_transport/measured_payload_pose', 10)
        self._error_publisher = self.create_publisher(
            Float64MultiArray, '/cooperative_transport/slip_error', 10)
        # Publish the measured payload pose as a standard TF frame as well as
        # PoseStamped, so all transport calculations can share one object-fixed
        # coordinate system.
        self._transport_tf_broadcaster = TransformBroadcaster(self)
        self.create_service(
            Trigger, '/cooperative_transport/reset_slip', self._on_reset)
        self.create_timer(0.02, self._tick)
        self.get_logger().info('Payload slip monitor ready.')

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        if bool(self.get_parameter('use_world_model_poses').value):
            return
        point = message.pose.pose.position
        self._base_positions[robot] = (point.x, point.y)

    def _on_grasp(self, robot: str, message: Bool) -> None:
        self._grasped[robot] = message.data
        if not all(self._grasped.values()) and not self._active:
            self._baseline = None

    def _on_active(self, message: Bool) -> None:
        if message.data and not self._active:
            self._active_since = time.monotonic()
        self._active = message.data

    def _on_poses(self, message: TFMessage) -> None:
        payload_name = self.get_parameter('payload_name').value
        for transform in message.transforms:
            if (bool(self.get_parameter('use_world_model_poses').value)
                    and transform.child_frame_id in self._names):
                translation = transform.transform.translation
                self._base_positions[transform.child_frame_id] = (
                    translation.x, translation.y)
            if transform.child_frame_id != payload_name:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._payload = (
                translation.x, translation.y, translation.z,
                yaw_from_quaternion(
                    rotation.x, rotation.y, rotation.z, rotation.w),
            )
            measured = PoseStamped()
            measured.header.stamp = self.get_clock().now().to_msg()
            measured.header.frame_id = str(
                self.get_parameter('world_frame').value)
            measured.pose.position.x = translation.x
            measured.pose.position.y = translation.y
            measured.pose.position.z = translation.z
            measured.pose.orientation = rotation
            self._measured_pose_publisher.publish(measured)
            self._publish_transport_transform(translation, rotation)
            return

    def _publish_transport_transform(self, translation, rotation) -> None:
        """Broadcast world -> object-fixed transport coordinate frame.

        The Gazebo payload model origin is the payload centre of mass in the
        current SDF.  Its axes are therefore used directly as the transport
        frame axes and rotate with the physical payload.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter('world_frame').value)
        transform.child_frame_id = str(
            self.get_parameter('transport_frame').value)
        transform.transform.translation.x = translation.x
        transform.transform.translation.y = translation.y
        transform.transform.translation.z = translation.z
        transform.transform.rotation = rotation
        self._transport_tf_broadcaster.sendTransform(transform)

    def _relative_pose(self) -> Optional[Tuple[float, float, float, float]]:
        if self._payload is None or len(self._base_positions) != 2:
            return None
        first = self._base_positions[self._names[0]]
        second = self._base_positions[self._names[1]]
        center_x = 0.5 * (first[0] + second[0])
        center_y = 0.5 * (first[1] + second[1])
        formation_yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        dx = self._payload[0] - center_x
        dy = self._payload[1] - center_y
        cosine = math.cos(formation_yaw)
        sine = math.sin(formation_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        relative_yaw = _angle_difference(self._payload[3], formation_yaw)
        return local_x, local_y, self._payload[2], relative_yaw

    def _tick(self) -> None:
        relative = self._relative_pose()
        if (self._baseline is None and relative is not None
                and all(self._grasped.values()) and not self._active):
            self._baseline = relative
            self.get_logger().info(
                'Captured friction-grasp payload baseline: '
                f'x={relative[0]:.3f}, y={relative[1]:.3f}, z={relative[2]:.3f}')

        if (not self._fault and self._active and relative is not None
                and self._baseline is not None
                and time.monotonic() - self._active_since
                >= float(self.get_parameter('activation_grace_period').value)):
            dx = relative[0] - self._baseline[0]
            dy = relative[1] - self._baseline[1]
            horizontal = math.hypot(dx, dy)
            vertical = abs(relative[2] - self._baseline[2])
            yaw = abs(_angle_difference(relative[3], self._baseline[3]))
            self._current_error = (dx, dy, relative[2] - self._baseline[2], yaw)
            if horizontal > float(self.get_parameter('horizontal_slip_limit').value):
                self._latch(f'horizontal slip {horizontal:.3f} m')
            elif vertical > float(self.get_parameter('vertical_slip_limit').value):
                self._latch(f'vertical slip {vertical:.3f} m')
            elif yaw > float(self.get_parameter('yaw_slip_limit').value):
                self._latch(f'yaw slip {math.degrees(yaw):.2f} deg')

        fault = Bool()
        fault.data = self._fault
        self._fault_publisher.publish(fault)
        status = String()
        status.data = f'SLIP: {self._reason}' if self._fault else 'OK'
        self._status_publisher.publish(status)
        error = Float64MultiArray()
        error.data = list(self._current_error)
        self._error_publisher.publish(error)

    def _latch(self, reason: str) -> None:
        self._fault = True
        self._reason = reason
        self.get_logger().error(f'Payload slip fault latched: {reason}')

    def _on_reset(self, _request: Trigger.Request,
                  response: Trigger.Response) -> Trigger.Response:
        if self._active:
            response.success = False
            response.message = 'Disable cooperative transport before resetting slip.'
            return response
        self._fault = False
        self._reason = 'OK'
        self._baseline = self._relative_pose() if all(self._grasped.values()) else None
        response.success = True
        response.message = 'Slip monitor reset and baseline recaptured.'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlipMonitor()
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
