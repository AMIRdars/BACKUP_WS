"""Set one right-translation goal for the existing cooperative coordinator."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z
               + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y
                     + orientation.z * orientation.z),
    )


class CooperativeLateralGoal(Node):
    """Start a fixed-pose, rightward transport after the grasp is ready."""

    def __init__(self):
        super().__init__('cooperative_lateral_goal')
        self.declare_parameter('right_distance_m', 0.5)
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('wait_for_rotation', False)

        self._names = (
            str(self.get_parameter('robot1_namespace').value).strip('/'),
            str(self.get_parameter('robot2_namespace').value).strip('/'),
        )
        self._distance = float(self.get_parameter('right_distance_m').value)
        self._poses = {}
        self._grasped = {name: False for name in self._names}
        self._support_removed = False
        self._rotation_complete = False
        self._wait_for_rotation = bool(
            self.get_parameter('wait_for_rotation').value)
        self._state = ''
        self._goal = None
        self._enabled = False
        self._goal_publisher = self.create_publisher(
            PoseStamped, '/cooperative_transport/goal', 10)
        self._enable_publisher = self.create_publisher(
            Bool, '/cooperative_transport/enable', 10)
        support_qos = QoSProfile(depth=1)
        support_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_transport/support_removed',
            self._on_support_removed, support_qos)
        self.create_subscription(
            String, '/cooperative_transport/state', self._on_state, 10)
        rotation_qos = QoSProfile(depth=1)
        rotation_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_rotation/complete',
            self._on_rotation_complete, rotation_qos)
        for name in self._names:
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp(robot, message), 10)
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odom(robot, message), 10)
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value),
            self._tick)
        self.get_logger().info(
            f'Waiting for both grasp states; right target={self._distance:.3f} m')

    def _on_odom(self, robot: str, message: Odometry) -> None:
        pose = message.pose.pose
        self._poses[robot] = (
            pose.position.x, pose.position.y, _yaw(pose.orientation))

    def _on_grasp(self, robot: str, message: Bool) -> None:
        self._grasped[robot] = bool(message.data)

    def _on_support_removed(self, message: Bool) -> None:
        self._support_removed = bool(message.data)

    def _on_state(self, message: String) -> None:
        self._state = message.data

    def _on_rotation_complete(self, message: Bool) -> None:
        self._rotation_complete = bool(message.data)

    def _make_goal(self) -> PoseStamped:
        first = self._poses[self._names[0]]
        second = self._poses[self._names[1]]
        center_x = 0.5 * (first[0] + second[0])
        center_y = 0.5 * (first[1] + second[1])
        payload_yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        # Payload-local right is -Y, i.e. (sin(yaw), -cos(yaw)) in world.
        target = PoseStamped()
        target.header.frame_id = 'world'
        target.pose.position.x = center_x + math.sin(payload_yaw) * self._distance
        target.pose.position.y = center_y - math.cos(payload_yaw) * self._distance
        target.pose.orientation.z = math.sin(0.5 * payload_yaw)
        target.pose.orientation.w = math.cos(0.5 * payload_yaw)
        return target

    def _tick(self) -> None:
        ready = (
            all(self._grasped.values())
            and self._support_removed
            and (not self._wait_for_rotation or self._rotation_complete)
            and all(name in self._poses for name in self._names))
        if self._goal is None and ready:
            self._goal = self._make_goal()
            self.get_logger().info(
                f'Publishing cooperative right-translation goal '
                f'{self._distance:.3f} m')
        if self._goal is None:
            return

        self._goal.header.stamp = self.get_clock().now().to_msg()
        self._goal_publisher.publish(self._goal)
        if not self._enabled and self._state not in ('HOLDING', 'TRANSPORT'):
            self._enable_publisher.publish(Bool(data=True))
            self._enabled = True
        if self._state == 'HOLDING' and self._enabled:
            self._enable_publisher.publish(Bool(data=False))
            self._enabled = False
            self.get_logger().info('Cooperative right translation complete')


def main(args=None):
    rclpy.init(args=args)
    node = CooperativeLateralGoal()
    try:
        rclpy.spin(node)
    finally:
        node._enable_publisher.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()
