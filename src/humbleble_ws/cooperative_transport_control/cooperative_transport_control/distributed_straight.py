"""Independent straight-transport controllers using object-mediated feedback.

The leader alone receives a payload goal. The follower observes the moving
payload and its own finger loads, without subscribing to a leader command.
The Gazebo payload-pose topic stands in for a local object tracker. This is an
engineering adaptation, not the prescribed-performance law in the papers.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


def limit(value, bound):
    return max(-bound, min(bound, value))


def slew(previous, target, acceleration, dt):
    return previous + limit(target - previous, acceleration * dt)


def world_to_body(vx, vy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class _Base(Node):
    def __init__(self, name, robot):
        super().__init__(name)
        self.declare_parameter('robot', robot)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_speed', 0.04)
        self.declare_parameter('max_acceleration', 0.025)
        self.declare_parameter('odom_timeout', 0.5)
        self.robot = str(self.get_parameter('robot').value).strip('/')
        self.odom = None
        self.odom_at = 0.0
        self.grasped = False
        self.support_removed = False
        self.emergency_stop = False
        self.last_command = (0.0, 0.0)
        self.motion_owned = False
        self.last_tick = self._sim_now()
        self.command_pub = self.create_publisher(
            Twist, f'/{self.robot}/rover_twist', 10)
        self.create_subscription(
            Odometry, f'/{self.robot}/odom', self._on_odom, 10)
        self.create_subscription(
            Bool, f'/cooperative_transport/{self.robot}/grasp_state',
            lambda msg: setattr(self, 'grasped', msg.data), 10)
        self.create_subscription(
            Bool, '/cooperative_transport/support_removed',
            lambda msg: setattr(self, 'support_removed', msg.data), 10)
        self.create_subscription(
            Bool, '/cooperative_transport/emergency_stop',
            lambda msg: setattr(self, 'emergency_stop', msg.data), 10)

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_at = time.monotonic()

    def _sim_now(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _ready(self):
        return (self.grasped and self.support_removed
                and not self.emergency_stop and self.odom is not None
                and time.monotonic() - self.odom_at
                <= float(self.get_parameter('odom_timeout').value))

    def _publish(self, vx=0.0, vy=0.0, dt=0.02, immediate=False):
        if immediate:
            vx, vy = 0.0, 0.0
        else:
            acceleration = float(self.get_parameter('max_acceleration').value)
            vx = slew(self.last_command[0], vx, acceleration, dt)
            vy = slew(self.last_command[1], vy, acceleration, dt)
            speed = math.hypot(vx, vy)
            maximum = float(self.get_parameter('max_speed').value)
            if speed > maximum:
                vx, vy = vx * maximum / speed, vy * maximum / speed
        msg = Twist()
        msg.linear.x, msg.linear.y = vx, vy
        self.command_pub.publish(msg)
        self.last_command = (vx, vy)


class DistributedLeader(_Base):
    """Track a payload goal without commanding the follower's base."""

    def __init__(self):
        super().__init__('distributed_transport_leader', 'amir1')
        self.declare_parameter('goal_x', 0.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('cruise_speed', 0.025)
        self.declare_parameter('position_gain', 0.8)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('settling_time', 1.0)
        self.declare_parameter('tracking_pause_error', 0.05)
        self.declare_parameter('tracking_resume_error', 0.025)
        self.declare_parameter('payload_timeout', 0.5)
        self.declare_parameter('start_delay', 2.0)
        self.payload = None
        self.payload_at = 0.0
        self.reference = None
        self.progress = 0.0
        self.speed = 0.0
        self.paused = False
        self.ready_since = None
        self.within_since = None
        self.done = False
        self.active_pub = self.create_publisher(
            Bool, '/cooperative_transport/active', 10)
        self.state_pub = self.create_publisher(
            String, '/distributed_transport/leader/state', 10)
        self.create_subscription(
            PoseStamped, '/cooperative_transport/measured_payload_pose',
            self._on_payload, 10)
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value), self._tick)

    def _on_payload(self, msg):
        self.payload = (msg.pose.position.x, msg.pose.position.y)
        self.payload_at = time.monotonic()

    def _status(self, state, active=False):
        self.active_pub.publish(Bool(data=active))
        self.state_pub.publish(String(data=state))

    def _tick(self):
        now = time.monotonic()
        sim_now = self._sim_now()
        dt = max(0.0, min(0.1, sim_now - self.last_tick))
        self.last_tick = sim_now
        if not self._ready() or self.payload is None or (
                now - self.payload_at > float(
                    self.get_parameter('payload_timeout').value)):
            if self.motion_owned:
                self._publish(immediate=True)
            self._status('WAITING_OR_STOPPED')
            self.ready_since = None
            return
        self.motion_owned = True
        if self.done:
            self._publish(immediate=True)
            self._status('FINISHED', True)
            return
        if self.ready_since is None:
            self.ready_since = sim_now
        if sim_now - self.ready_since < float(self.get_parameter('start_delay').value):
            self._publish(immediate=True)
            self._status('READY')
            return
        if self.reference is None:
            self.reference = self.payload
            self.get_logger().info(
                f'Leader started at payload {self.reference}; '
                f'follower receives no target topic.')
        goal = (float(self.get_parameter('goal_x').value),
                float(self.get_parameter('goal_y').value))
        dx, dy = goal[0] - self.reference[0], goal[1] - self.reference[1]
        distance = math.hypot(dx, dy)
        if distance < 1.0e-6:
            self.done = True
            self._publish(immediate=True)
            self._status('FINISHED')
            return
        ux, uy = dx / distance, dy / distance
        ref_x = self.reference[0] + ux * self.progress
        ref_y = self.reference[1] + uy * self.progress
        error = math.hypot(ref_x - self.payload[0], ref_y - self.payload[1])
        if self.paused:
            self.paused = error > float(
                self.get_parameter('tracking_resume_error').value)
        else:
            self.paused = error >= float(
                self.get_parameter('tracking_pause_error').value)
        if not self.paused and self.progress < distance:
            accel = float(self.get_parameter('max_acceleration').value)
            cruise = float(self.get_parameter('cruise_speed').value)
            remaining = distance - self.progress
            self.speed = min(cruise, self.speed + accel * dt,
                             math.sqrt(max(0.0, 2.0 * accel * remaining)))
            self.progress = min(distance, self.progress + self.speed * dt)
        else:
            self.speed = 0.0 if self.progress >= distance else self.speed
        ref_x = self.reference[0] + ux * self.progress
        ref_y = self.reference[1] + uy * self.progress
        gain = float(self.get_parameter('position_gain').value)
        world_vx = (0.0 if self.paused else self.speed * ux) + gain * (ref_x - self.payload[0])
        world_vy = (0.0 if self.paused else self.speed * uy) + gain * (ref_y - self.payload[1])
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        body_vx, body_vy = world_to_body(world_vx, world_vy, yaw)
        self._publish(body_vx, body_vy, dt)
        if self.progress >= distance and math.hypot(
                goal[0] - self.payload[0], goal[1] - self.payload[1]) <= float(
                self.get_parameter('position_tolerance').value):
            if self.within_since is None:
                self.within_since = sim_now
            if sim_now - self.within_since >= float(
                    self.get_parameter('settling_time').value):
                self.done = True
        else:
            self.within_since = None
        self._status('FINISHED' if self.done else
                     'TRACKING_PAUSED' if self.paused else 'MOVING',
                     True)


class DistributedFollower(_Base):
    """Follow observed object motion, with bounded local force admittance."""

    def __init__(self):
        super().__init__('distributed_transport_follower', 'amir2')
        self.declare_parameter('force_axis', 'y')
        self.declare_parameter('force_sign', 1.0)
        self.declare_parameter('force_gain', 0.0001)
        self.declare_parameter('force_deadband', 1.5)
        self.declare_parameter('force_filter_alpha', 0.08)
        self.declare_parameter('force_timeout', 0.3)
        self.declare_parameter('payload_timeout', 0.3)
        self.declare_parameter('object_position_gain', 1.2)
        self.declare_parameter('object_velocity_alpha', 0.15)
        self.declare_parameter('max_force_correction', 0.005)
        self.declare_parameter('baseline_duration', 1.0)
        self.force = {}
        self.force_at = {}
        self.filtered = None
        self.baseline_sum = 0.0
        self.baseline_count = 0
        self.baseline_since = None
        self.baseline = None
        self.payload = None
        self.payload_at = 0.0
        self.payload_stamp = None
        self.payload_velocity = (0.0, 0.0)
        self.offset = None
        self.state_pub = self.create_publisher(
            String, '/distributed_transport/follower/state', 10)
        self.fault_pub = self.create_publisher(
            Bool, '/distributed_transport/follower/fault', 10)
        for side in ('left', 'right'):
            self.create_subscription(
                WrenchStamped, f'/{self.robot}/finger_{side}_wrench',
                lambda msg, key=side: self._on_force(key, msg), 10)
        self.create_subscription(
            PoseStamped, '/cooperative_transport/measured_payload_pose',
            self._on_payload, 10)
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value), self._tick)

    def _on_force(self, side, msg):
        axis = str(self.get_parameter('force_axis').value)
        self.force[side] = getattr(msg.wrench.force, axis)
        self.force_at[side] = time.monotonic()

    def _on_payload(self, msg):
        position = (msg.pose.position.x, msg.pose.position.y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        if self.payload is not None and self.payload_stamp is not None:
            dt = stamp - self.payload_stamp
            if 1.0e-4 < dt <= 0.2:
                raw_v = ((position[0] - self.payload[0]) / dt,
                         (position[1] - self.payload[1]) / dt)
                alpha = float(self.get_parameter('object_velocity_alpha').value)
                self.payload_velocity = (
                    alpha * raw_v[0] + (1.0 - alpha) * self.payload_velocity[0],
                    alpha * raw_v[1] + (1.0 - alpha) * self.payload_velocity[1])
        self.payload = position
        self.payload_stamp = stamp
        self.payload_at = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        sim_now = self._sim_now()
        dt = max(0.0, min(0.1, sim_now - self.last_tick))
        self.last_tick = sim_now
        fresh = all(side in self.force and now - self.force_at[side] <= float(
            self.get_parameter('force_timeout').value)
            for side in ('left', 'right'))
        pose_fresh = (self.payload is not None and
                      now - self.payload_at <= float(
                          self.get_parameter('payload_timeout').value))
        if not self._ready() or not fresh or not pose_fresh:
            if self.motion_owned:
                self._publish(immediate=True)
            self.fault_pub.publish(Bool(data=self.motion_owned and
                                        not self.emergency_stop))
            self.state_pub.publish(String(data='WAITING_OR_STOPPED'))
            self.baseline_since = None
            return
        self.motion_owned = True
        self.fault_pub.publish(Bool(data=False))
        measured = 0.5 * (self.force['left'] + self.force['right'])
        alpha = float(self.get_parameter('force_filter_alpha').value)
        self.filtered = measured if self.filtered is None else (
            alpha * measured + (1.0 - alpha) * self.filtered)
        if self.baseline is None:
            if self.baseline_since is None:
                self.baseline_since = sim_now
            self.baseline_sum += self.filtered
            self.baseline_count += 1
            self._publish(immediate=True)
            if sim_now - self.baseline_since >= float(
                    self.get_parameter('baseline_duration').value):
                self.baseline = self.baseline_sum / self.baseline_count
                base = self.odom.pose.pose.position
                self.offset = (base.x - self.payload[0],
                               base.y - self.payload[1])
                self.get_logger().info(
                    f'Follower force baseline {self.baseline:.3f} N; '
                    f'object offset {self.offset}; no leader goal subscription.')
            self.state_pub.publish(String(data='CALIBRATING'))
            return
        delta = self.filtered - self.baseline
        deadband = float(self.get_parameter('force_deadband').value)
        effective = math.copysign(max(0.0, abs(delta) - deadband), delta)
        velocity = (float(self.get_parameter('force_sign').value)
                    * float(self.get_parameter('force_gain').value) * effective)
        velocity = limit(velocity, float(
            self.get_parameter('max_force_correction').value))
        base = self.odom.pose.pose.position
        gain = float(self.get_parameter('object_position_gain').value)
        world_vx = (self.payload_velocity[0]
                    + gain * (self.payload[0] + self.offset[0] - base.x))
        world_vy = (self.payload_velocity[1]
                    + gain * (self.payload[1] + self.offset[1] - base.y))
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        body_vx, body_vy = world_to_body(world_vx, world_vy, yaw)
        self._publish(body_vx + velocity, body_vy, dt)
        self.state_pub.publish(String(data='FOLLOWING'))


def leader_main(args=None):
    rclpy.init(args=args)
    node = DistributedLeader()
    try:
        rclpy.spin(node)
    finally:
        node._publish(immediate=True)
        node.destroy_node()
        rclpy.shutdown()


def follower_main(args=None):
    rclpy.init(args=args)
    node = DistributedFollower()
    try:
        rclpy.spin(node)
    finally:
        node._publish(immediate=True)
        node.destroy_node()
        rclpy.shutdown()
