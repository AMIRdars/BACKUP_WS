"""Leader-only small relative SE(2) goals, PoseStamped in own start axes.

Position is metres, quaternion is rotation. No goal or reference is shared
with the follower. A rotation centre is converted to this displacement by
the leader's caller; the follower does not receive that centre.
"""
import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from .transport import TransportEndpoint, straight_reference
from .wrench import finite


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class PlanarLeader(TransportEndpoint):
    def __init__(self, node_name='leader_planar_trajectory', **kwargs):
        super().__init__(leader=True, node_name=node_name,
            goal_message_type=PoseStamped, goal_topic='transport_pose_goal', **kwargs)
        self.declare_parameter('max_yaw_rad', .001)
        self.declare_parameter('yaw_metric_m', .1)
        self.declare_parameter('terminal_timeout_s', 3.)
        self.declare_parameter('reference_acceleration_m_s2', 0.)
        for key in ('max_yaw_rad', 'yaw_metric_m', 'terminal_timeout_s'):
            if not math.isfinite(self.p(key)) or self.p(key) <= 0: raise ValueError('positive planar policy required')
        reference_acceleration = self.p('reference_acceleration_m_s2')
        if (not math.isfinite(reference_acceleration) or reference_acceleration < 0
                or reference_acceleration > self.p('acceleration_m_s2')):
            raise ValueError('reference acceleration must be zero (legacy) or within tracking acceleration')
        self.path_goal, self.planar_velocity = None, np.zeros(3)

    def goal(self, msg):
        try:
            p, q = msg.pose.position, msg.pose.orientation
            value = finite([p.x, p.y, p.z, q.x, q.y, q.z, q.w], (7,))
            yaw = yaw_of(msg.pose)
            if (abs(np.linalg.norm(value[3:])-1.) > 1e-6 or abs(p.z) > 1e-12
                    or abs(q.x)+abs(q.y) > 1e-12 or math.hypot(p.x, p.y) > self.p('max_distance_m')
                    or abs(yaw) > self.p('max_yaw_rad')):
                raise ValueError('goal must be a normalized planar relative pose in configured envelope')
            path = np.array([p.x, p.y, yaw])
            length = float(np.linalg.norm(path*np.array([1., 1., self.p('yaw_metric_m')])))
            if length <= 0 or length > self.p('max_distance_m'): raise ValueError('zero or excessive metric path length')
            proxy = Vector3Stamped(header=msg.header)
            proxy.vector.x = length
            before = self.accepted
            TransportEndpoint.goal(self, proxy)
            if self.accepted == before: return
            odom = self.encoder.pose.pose
            self.path_goal = (self.sim+.05, path, length, np.array([odom.position.x, odom.position.y, yaw_of(odom)]))
            self.goal_data = None  # parent retains home/health path; this class owns planar tracking
            self.planar_velocity[:] = 0.
        except ValueError as exc:
            self.rejected += 1
            self.last_rejection = str(exc)

    def base_request(self, command):
        if self.state != 'TRANSPORT' or self.path_goal is None: return np.zeros(3)
        start, path, length, origin = self.path_goal
        pose = self.encoder.pose.pose
        c, s = math.cos(origin[2]), math.sin(origin[2])
        actual = np.array([c*(pose.position.x-origin[0])+s*(pose.position.y-origin[1]),
            -s*(pose.position.x-origin[0])+c*(pose.position.y-origin[1]),
            math.atan2(math.sin(yaw_of(pose)-origin[2]), math.cos(yaw_of(pose)-origin[2]))])
        reference_acceleration = self.p('reference_acceleration_m_s2') or self.p('acceleration_m_s2')
        ref, speed, duration = straight_reference(self.sim-start, length, self.p('speed_m_s'), reference_acceleration)
        metric = np.array([1., 1., self.p('yaw_metric_m')])
        done = self.sim-start >= duration and np.linalg.norm((path-actual)*metric) <= self.p('goal_tolerance_m')
        desired = np.zeros(3) if done else path/length*speed+self.p('position_gain_s_inv')*(path/length*ref-actual)
        norm = np.linalg.norm(desired*metric)
        if norm > self.p('speed_m_s'): desired *= self.p('speed_m_s')/norm
        dt = max(0., self.sim-self.last_tick_sim) if self.last_tick_sim is not None else 0.
        delta = desired-self.planar_velocity
        norm = np.linalg.norm(delta*metric)
        if norm > self.p('acceleration_m_s2')*dt:
            delta *= self.p('acceleration_m_s2')*dt/max(norm, 1e-15)
        self.planar_velocity += delta
        if done and np.linalg.norm(self.planar_velocity) < 1e-12:
            self.completed += 1
            self.state, self.path_goal = 'READY', None
        elif self.sim-start > duration+self.p('terminal_timeout_s'):
            self.trip('leader local planar tracking timeout')
            return np.zeros(3)
        # Reference/feedback are expressed in start axes; actuator twist in
        # current body axes. Yaw comes from this leader's own encoder only.
        angle = actual[2]
        c, s = math.cos(angle), math.sin(angle)
        vx, vy, wz = self.planar_velocity
        return np.array([c*vx+s*vy, -s*vx+c*vy, wz])

    def trip(self, reason):
        super().trip(reason)
        self.path_goal = None
        if hasattr(self, 'planar_velocity'): self.planar_velocity[:] = 0.

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.path_goal = None
            self.planar_velocity[:] = 0.
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanarLeader()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
