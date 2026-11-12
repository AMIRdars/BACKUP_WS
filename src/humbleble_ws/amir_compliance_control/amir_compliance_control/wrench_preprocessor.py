"""Read-only local F/T pipeline. No motion publisher or leader/GT input."""
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from .wrench import LocalModel, at_grasp, finite, interpolate
from .sensor_coherence import local_sensor_qos


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


class WrenchPreprocessor(Node):
    def __init__(self, **kwargs):
        super().__init__('wrench_preprocessor', **kwargs)
        for name, value in {'urdf_file': '', 'calibration_file': '', 'cutoff_hz': 8.,
                'max_age_s': .05, 'max_skew_s': .02, 'wall_timeout_s': .5,
                'bias_samples': 100, 'stationary_velocity': .001}.items():
            self.declare_parameter(name, value)
        self.p = lambda k: self.get_parameter(k).value
        self.declare_parameter('process_on_measurement',False)
        self.process_on_measurement=bool(self.p('process_on_measurement'))
        self.declare_parameter('simulation_source_watchdog',False)
        self.source_watchdog=self.p('simulation_source_watchdog')
        self.watch_clock=None;self.watch_clock_wall=time.monotonic()
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('use_sim_time=true required')
        self.robot = self.get_namespace().strip('/')
        if not self.robot or '/' in self.robot:
            raise ValueError('explicit single robot namespace required')
        xml = Path(self.p('urdf_file')).read_text()
        self.model = LocalModel(xml)
        cert = json.loads(Path(self.p('calibration_file')).read_text())
        if (not cert.get('sensor_calibration_passed') or cert.get('robot') != self.robot
                or cert.get('urdf_sha256') != hashlib.sha256(xml.encode()).hexdigest()
                or set(cert.get('verified_axes', [])) != {'fx', 'fy', 'fz', 'tx', 'ty', 'tz'}
                or cert.get('sign') not in (-1., 1.) or not cert.get('header_frame')):
            raise ValueError('matching measured six-axis sensor certificate required')
        self.sign, self.header = cert['sign'], cert['header_frame']
        if (not self.header.startswith(self.robot+'/') or self.p('bias_samples') < 2
                or any(not np.isfinite(self.p(k)) or self.p(k) <= 0 for k in
                       ('cutoff_hz', 'max_age_s', 'max_skew_s', 'wall_timeout_s', 'stationary_velocity'))):
            raise ValueError('invalid local sensor/filter parameters')
        self.history, self.pending = deque(maxlen=256), deque(maxlen=50)
        self.tare_window = deque(maxlen=self.p('bias_samples'))
        self.last_stamp, self.joint_stamp, self.odom_stamp = None, None, None
        self.last_seen_wrench = None
        self.last_wall, self.last_joint_wall, self.last_odom_wall = 0., 0., 0.
        self.bias, self.filtered = None, None
        self.stationary, self.joint_stationary = False, False
        self.state = 'WAITING'
        self.raw_pub = self.create_publisher(WrenchStamped, 'wrench_at_grasp_raw', 10)
        self.filtered_pub = self.create_publisher(WrenchStamped, 'wrench_filtered', 10)
        self.status_pub = self.create_publisher(String, 'wrench_status', 10)
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 10)
        input_qos=local_sensor_qos(self.source_watchdog)
        self.create_subscription(JointState, 'joint_states', self.joint, input_qos)
        self.create_subscription(Odometry, 'mecanum_drive_controller/odometry', self.odom, input_qos)
        self.create_subscription(WrenchStamped, 'ft_sensor', self.wrench, input_qos)
        allowed = {'/clock', *[f'/{self.robot}/{s}' for s in
            ('joint_states', 'mecanum_drive_controller/odometry', 'ft_sensor')]}
        if {s.topic_name for s in self.subscriptions} != allowed:
            raise ValueError('local subscription boundary violated by remapping')
        self.create_service(Trigger, 'tare_wrench', self.tare)
        if self.process_on_measurement:
            self.create_timer(.01,self.tick)
            self.create_timer(.01,self.wall_watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))
        else:self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def wall_watchdog(self):
        if self.last_stamp is None or self.wall_input_expired(time.monotonic()):
            self.fail('local sensor/clock watchdog')
            self.status_pub.publish(String(data=self.state))

    def wall_input_expired(self,wall):
        if not self.source_watchdog:
            return wall-min(self.last_wall,self.last_joint_wall,self.last_odom_wall)>self.p('wall_timeout_s')
        now=self.get_clock().now().nanoseconds*1e-9
        if self.watch_clock is None or now>self.watch_clock:self.watch_clock_wall=wall
        backwards=self.watch_clock is not None and now<self.watch_clock
        self.watch_clock=now
        return backwards or wall-self.watch_clock_wall>self.p('wall_timeout_s')

    def process_ready_measurements(self):
        if self.process_on_measurement and self.pending:self.tick()

    def fail(self, reason):
        self.state = 'INVALID: '+reason
        self.bias, self.filtered = None, None
        self.last_stamp = None
        self.tare_window.clear()

    def joint(self, msg):
        try:
            s = stamp(msg)
            if s <= 0 or (self.joint_stamp is not None and s <= self.joint_stamp):
                raise ValueError('joint timestamp did not advance')
            values, velocities = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
            q = finite([values[n] for n in self.model.names], (5,))
            v = finite([velocities[n] for n in self.model.names], (5,))
            self.model.frames(q)
            self.history.append((s, q))
            self.joint_stamp, self.last_joint_wall = s, time.monotonic()
            self.joint_stationary = np.max(np.abs(v)) <= self.p('stationary_velocity')
            self.process_ready_measurements()
        except (ValueError, KeyError) as exc:
            self.fail(str(exc))

    def odom(self, msg):
        try:
            if (msg.header.frame_id != self.robot+'/odom' or
                    msg.child_frame_id != self.robot+'/base_footprint'):
                raise ValueError('unexpected encoder frame')
            s = stamp(msg)
            if s <= 0 or (self.odom_stamp is not None and s <= self.odom_stamp):
                raise ValueError('encoder timestamp did not advance')
            t = msg.twist.twist
            v = finite([t.linear.x, t.linear.y, t.angular.z], (3,))
            self.stationary = np.max(np.abs(v)) <= self.p('stationary_velocity')
            self.odom_stamp, self.last_odom_wall = s, time.monotonic()
            self.process_ready_measurements()
        except ValueError as exc:
            self.fail(str(exc))

    def wrench(self, msg):
        s = stamp(msg)
        if s <= 0 or (self.last_seen_wrench is not None and s <= self.last_seen_wrench):
            self.fail('F/T timestamp did not advance')
            return
        self.last_seen_wrench = s
        if len(self.pending) == self.pending.maxlen:
            self.fail('wrench queue overflow')
            self.pending.clear()
        self.pending.append(msg)
        self.process_ready_measurements()

    def tare(self, request, response):
        response.success = (len(self.tare_window) == self.tare_window.maxlen
                            and self.state in ('UNTARED', 'READY'))
        if response.success:
            self.bias = np.mean(self.tare_window, axis=0)
            self.filtered = self.bias.copy()
            self.state = 'READY'
            response.message = 'stationary bias captured; raw wrench remains available'
        else:
            response.message = 'fresh stationary local samples required'
        return response

    def tick(self):
        now, wall = self.get_clock().now().nanoseconds*1e-9, time.monotonic()
        while (self.pending and self.history and stamp(self.pending[0]) <= self.history[-1][0]
               and stamp(self.pending[0]) <= now and self.joint_stamp <= now
               and self.odom_stamp is not None and self.odom_stamp <= now):
            msg = self.pending.popleft()
            try:
                s = stamp(msg)
                if msg.header.frame_id != self.header:
                    raise ValueError('unexpected sensor frame')
                if s <= 0 or (self.last_stamp is not None and s <= self.last_stamp):
                    raise ValueError('F/T timestamp did not advance')
                stamps = finite([s, self.joint_stamp, self.odom_stamp], (3,))
                invalid_time=(np.any(now-stamps < -1e-6) or np.max(now-stamps) > self.p('max_age_s')
                        or np.ptp(stamps) > self.p('max_skew_s'))
                if self.process_on_measurement:
                    ages_ns=np.rint((now-stamps)*1e9)
                    invalid_time=bool(np.min(ages_ns)<-1000 or np.max(ages_ns)>round(self.p('max_age_s')*1e9)
                        or round(float(np.ptp(stamps))*1e9)>round(self.p('max_skew_s')*1e9))
                if invalid_time:
                    raise ValueError('stale, future or skewed sensor state')
                q = interpolate(self.history, s, self.p('max_skew_s'))
                grasp, sensor = self.model.frames(q)
                w = msg.wrench
                raw = at_grasp([w.force.x, w.force.y, w.force.z,
                                w.torque.x, w.torque.y, w.torque.z], sensor, grasp, self.sign)
                dt = s-self.last_stamp if self.last_stamp is not None else .01
                if not 0 < dt <= self.p('max_age_s'):
                    raise ValueError('F/T interval discontinuity')
                self.last_stamp, self.last_wall = s, wall
                frame = self.robot+'/grasp_aligned'
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id = msg.header.stamp, self.robot+'/base_footprint'
                tf.child_frame_id = frame
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, grasp)
                tf.transform.rotation.w = 1.
                self.tf_pub.publish(TFMessage(transforms=[tf]))
                def output(values):
                    result = WrenchStamped()
                    result.header.stamp, result.header.frame_id = msg.header.stamp, frame
                    (result.wrench.force.x, result.wrench.force.y, result.wrench.force.z,
                     result.wrench.torque.x, result.wrench.torque.y,
                     result.wrench.torque.z) = map(float, values)
                    return result
                self.raw_pub.publish(output(raw))
                if self.stationary and self.joint_stationary:
                    self.tare_window.append(raw.copy())
                else:
                    self.tare_window.clear()
                if self.bias is not None:
                    alpha = 1-np.exp(-2*np.pi*self.p('cutoff_hz')*dt)
                    self.filtered += alpha*(raw-self.filtered)
                    self.filtered_pub.publish(output(self.filtered-self.bias))
                    self.state = 'READY'
                else:
                    self.state = 'UNTARED'
            except (ValueError, TypeError) as exc:
                self.fail(str(exc))
        if (self.last_stamp is None or now-self.last_stamp > self.p('max_age_s')
                or self.wall_input_expired(wall)):
            self.fail('local sensor/clock watchdog')
        self.status_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = WrenchPreprocessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
