"""CP-5 local ARM_ONLY mode. Base requests are always zero."""
import json
import time
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .admittance import PlanarAdmittance, solve_xyz
from .transport import TransportEndpoint
from .safety import stamp
from .wrench import finite
from .sensor_coherence import local_sensor_qos


class ArmAdmittance(TransportEndpoint):
    def __init__(self, *, node_name='follower_arm_admittance', mechanics_defaults=None, **kwargs):
        super().__init__(leader=False, node_name=node_name, **kwargs)
        self.role = 'arm_only_admittance'
        settings = {'virtual_mass': [1., 1.], 'virtual_damping': [40., 40.],
                'virtual_stiffness': [400., 400.], 'displacement_limit_m': [.0005, .0005],
                'velocity_limit_m_s': [.001, .001], 'acceleration_limit_m_s2': [.02, .02],
                'joint_request_speed_rad_s': .03}
        settings.update(mechanics_defaults or {})
        for name, value in settings.items():
            self.declare_parameter(name, value)
        if not np.isfinite(self.p('joint_request_speed_rad_s')) or self.p('joint_request_speed_rad_s') <= 0:
            raise ValueError('positive joint request speed required')
        self.make_core()  # validate policies while still disarmed
        self.core, self.reference, self.q_request = None, None, None
        self.force, self.force_stamp, self.force_wall = None, None, 0.
        self.filter_status, self.filter_wall, self.last_step_stamp = '', 0., None
        self.create_subscription(WrenchStamped, 'wrench_filtered', self.filtered, local_sensor_qos(self.source_watchdog))
        self.create_subscription(String, 'wrench_status', self.filter_state, 10)
        self.debug_pub = self.create_publisher(String, 'admittance/debug', 10)
        self.create_service(Trigger, 'admittance/enable', self.enable)
        expected = {'/clock', '/transport/safety/follower', '/transport/safety_permit', '/transport/emergency_stop',
            *['/follower/'+s for s in ('joint_states', 'mecanum_drive_controller/odometry', 'grasp/state',
                                     'wrench_filtered', 'wrench_status')]}
        if self.p('actuator_ack_required'):expected.add('/transport/interlock/follower')
        if ({s.topic_name for s in self.subscriptions} != expected
                or self.debug_pub.topic_name != '/follower/admittance/debug'):
            self.destroy_node()
            raise ValueError('admittance local boundary violated by remapping')

    def make_core(self):
        return PlanarAdmittance(*(self.p(k) for k in ('virtual_mass', 'virtual_damping', 'virtual_stiffness',
            'displacement_limit_m', 'velocity_limit_m_s', 'acceleration_limit_m_s2')))

    def prepare(self, request, response):
        response = super().prepare(request, response)
        for topic in ('wrench_filtered', 'wrench_status'):
            sources = self.get_publishers_info_by_topic('/follower/'+topic)
            if (len(sources) != 1 or sources[0].node_name != 'wrench_preprocessor'
                    or sources[0].node_namespace != '/follower'):
                self.prepared = response.success = False
                response.message = 'unverified preprocessor publisher for '+topic
        return response

    def filter_state(self, msg):
        self.filter_status, self.filter_wall = msg.data, time.monotonic()
        if self.core is not None and msg.data != 'READY':
            self.trip('wrench preprocessing is not READY')

    def filtered(self, msg):
        try:
            s = stamp(msg)
            if s <= 0 or msg.header.frame_id != 'follower/grasp_aligned' or (self.force_stamp is not None and s <= self.force_stamp):
                raise ValueError('invalid filtered wrench frame/timestamp')
            w = msg.wrench
            value = finite([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], (6,))
            self.force, self.force_stamp, self.force_wall = value[:2], s, time.monotonic()
        except ValueError as exc:
            self.force, self.force_wall = None, 0.
            if self.core is not None:
                self.trip(str(exc))

    def wrench_ready(self):
        return (self.force is not None and self.filter_status == 'READY' and
                (self.source_watchdog or time.monotonic()-min(self.force_wall, self.filter_wall) <= self.p('timeout_s')) and
                -.02 <= self.sim-self.force_stamp <= .05)

    def enable(self, request, response):
        response.success = (self.state == 'READY' and not self.latched and self.permit and not self.emergency
                            and self.local_health()[0] and self.wrench_ready() and self.core is None)
        if response.success:
            mapping = dict(zip(self.joints.name, self.joints.position))
            self.q_request = finite([mapping[n] for n in self.model.names], (5,))
            self.reference = self.model.frames(self.q_request)[0]
            try:
                solve_xyz(self.model, self.q_request, self.reference)
                self.core = self.make_core()
                self.last_step_stamp = self.force_stamp
                self.state = 'TRANSPORT'
            except ValueError as exc:
                response.success, response.message = False, str(exc)
        response.message = response.message or ('ARM_ONLY enabled at measured local TCP; no base motion' if response.success
                                                 else 'fresh tared wrench and locally READY state required')
        return response

    def trip(self, reason):
        super().trip(reason)
        self.core = None

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.core = self.reference = self.q_request = self.last_step_stamp = None
        return response

    def control_force(self):
        return self.force

    def solve_request(self, target):
        return solve_xyz(self.model, self.q_request, target)

    def tick(self):
        super().tick()
        if not hasattr(self, 'core'):
            return
        if self.core is not None:
            if not self.wrench_ready():
                self.trip('filtered wrench/clock watchdog')
            else:
                dt = self.force_stamp-self.last_step_stamp
                if dt > 0:
                    try:
                        displacement = self.core.step(self.control_force(), dt)
                        target = self.reference+np.r_[displacement, 0.]
                        requested = self.solve_request(target)
                        rate = min(self.p('joint_request_speed_rad_s'), float(np.min(self.model.velocity)))*dt
                        self.q_request += np.clip(requested-self.q_request, -rate, rate)
                        msg = JointTrajectory(joint_names=list(self.model.names))
                        msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'follower/base_footprint'
                        point = JointTrajectoryPoint(positions=self.q_request.tolist())
                        point.time_from_start.nanosec = 40000000
                        msg.points = [point]
                        self.publish_command(self.arm_pub, msg)
                        self.last_step_stamp = self.force_stamp
                    except ValueError as exc:
                        self.trip(str(exc))
        self.debug_pub.publish(String(data=json.dumps({'state': self.state, 'reason': self.reason,
            'enabled': self.core is not None, 'sim_time': self.sim,
            'force_xy_N': None if self.force is None else self.force.tolist(),
            'reference_xyz_m': None if self.reference is None else self.reference.tolist(),
            'displacement_xy_m': None if self.core is None else self.core.x.tolist(),
            'velocity_xy_m_s': None if self.core is None else self.core.v.tolist(),
            'saturated_xy': None if self.core is None else self.core.saturated.tolist()})))


def main(args=None):
    rclpy.init(args=args)
    node = ArmAdmittance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
