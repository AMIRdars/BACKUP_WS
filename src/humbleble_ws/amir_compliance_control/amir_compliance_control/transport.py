"""CP-4 leader-only goals and a local stationary follower endpoint.

The follower holds; force admittance and mobile following belong to CP-5/6.
Neither endpoint imports experiment code or consumes ground truth / TF.
"""
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .safety import LATCHED, stamp
from .wrench import LocalModel, finite
from .sensor_coherence import local_sensor_qos


def straight_reference(t, distance, speed, acceleration):
    """Signed, bounded triangular/trapezoidal relative x reference."""
    if not all(math.isfinite(v) for v in (t, distance, speed, acceleration)) or speed <= 0 or acceleration <= 0:
        raise ValueError('finite trajectory and positive speed/acceleration required')
    if distance == 0:
        return 0., 0., 0.
    sign, length = math.copysign(1., distance), abs(distance)
    peak = min(speed, math.sqrt(length*acceleration))
    ramp = peak/acceleration
    cruise = max(0., length/peak-ramp)
    duration = 2*ramp+cruise
    if t <= 0:
        x, v = 0., 0.
    elif t < ramp:
        x, v = .5*acceleration*t*t, acceleration*t
    elif t < ramp+cruise:
        x, v = .5*peak*ramp+peak*(t-ramp), peak
    elif t < duration:
        remaining = duration-t
        x, v = length-.5*acceleration*remaining**2, acceleration*remaining
    else:
        x, v = length, 0.
    return sign*x, sign*v, duration


class TransportEndpoint(Node):
    def __init__(self, *, leader, node_name=None, goal_message_type=Vector3Stamped, goal_topic='transport_goal', base_trajectory=True, **kwargs):
        super().__init__(node_name or ('leader_trajectory_node' if leader else 'follower_hold'), **kwargs)
        self.leader = leader
        self.base_trajectory = base_trajectory
        self.role = 'leader' if leader else 'local_stationary_hold'
        self.robot = self.get_namespace().strip('/')
        if self.robot != ('leader' if leader else 'follower') or not self.get_parameter('use_sim_time').value:
            raise ValueError('explicit role namespace and use_sim_time=true required')
        for key, value in {'urdf_file': '', 'max_distance_m': .001, 'speed_m_s': .00025,
                'acceleration_m_s2': .001, 'position_gain_s_inv': 1.,
                'goal_tolerance_m': .000002, 'timeout_s': .15}.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        self.declare_parameter('simulation_source_watchdog',False)
        self.source_watchdog=self.p('simulation_source_watchdog')
        self.declare_parameter('command_delay_s', 0.)
        self.declare_parameter('straight_reference_acceleration_m_s2', 0.)
        reference_acceleration = self.get_parameter('straight_reference_acceleration_m_s2').value
        if (not math.isfinite(reference_acceleration) or reference_acceleration < 0
                or reference_acceleration > self.p('acceleration_m_s2')):
            raise ValueError('straight reference acceleration must be zero (legacy) or within tracking acceleration')
        if not math.isfinite(self.p('command_delay_s')) or not 0 <= self.p('command_delay_s') <= 1.:
            raise ValueError('command delay must be within simulation test envelope [0,1] s')
        self.pending_commands = deque()
        self.delay_active = False
        self.declare_parameter('actuator_ack_required', False)
        self.actuator,self.actuator_wall=None,0.
        for key in ('max_distance_m', 'speed_m_s', 'acceleration_m_s2', 'position_gain_s_inv', 'goal_tolerance_m', 'timeout_s'):
            if not math.isfinite(self.p(key)) or self.p(key) <= 0:
                raise ValueError('positive finite transport policy required')
        xml = Path(self.p('urdf_file')).read_text()
        self.model = LocalModel(xml)
        root = ET.fromstring(xml)
        self.home = finite([float(root.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text)
                            for n in self.model.names], (5,))
        if np.any(self.home <= self.model.lower) or np.any(self.home >= self.model.upper):
            raise ValueError('model home is outside joint limits')
        self.sim, self.clock_wall, self.last_clock = 0., 0., None
        self.encoder, self.joints, self.gate = None, None, None
        self.encoder_wall = self.joints_wall = self.gate_wall = 0.
        self.permit, self.permit_wall, self.emergency = False, 0., True
        self.grasp, self.grasp_wall = False, 0.
        self.prepared, self.latched, self.was_enabled = False, False, False
        self.state, self.reason = 'SAFE_STOP', 'prepare and arm required'
        self.enabled_since, self.home_sent, self.homed = None, False, False
        self.home_last_send = 0.
        self.goal_data, self.last_goal_stamp, self.last_velocity, self.last_tick_sim = None, 0., 0., None
        self.accepted, self.rejected, self.completed, self.last_rejection = 0, 0, 0, ''
        self.base_pub = self.create_publisher(TwistStamped, 'base_cmd', 1)
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_joint_cmd', 1)
        self.state_pub = self.create_publisher(String, 'transport/state', 10)
        self.stop_client = self.create_client(Trigger, '/transport/safety/stop')
        self.stop_future = None
        input_qos=local_sensor_qos(self.source_watchdog)
        self.create_subscription(ClockMessage, '/clock', self.clock, input_qos)
        self.create_subscription(Odometry, 'mecanum_drive_controller/odometry', self.odometry, input_qos)
        self.create_subscription(JointState, 'joint_states', self.joint_state, input_qos)
        self.create_subscription(String, '/transport/safety/'+self.robot, self.gate_status, 10)
        self.create_subscription(Bool, '/transport/safety_permit', self.allow, 1)
        self.create_subscription(Bool, '/transport/emergency_stop', self.stop_signal, LATCHED)
        self.create_subscription(String, 'grasp/state', self.grasp_state, 10)
        if self.p('actuator_ack_required'):
            self.create_subscription(String,'/transport/interlock/'+self.robot,self.actuator_status,10)
        if leader:
            self.create_subscription(goal_message_type, goal_topic, self.goal, 1)
        expected = {'/clock', '/transport/safety/'+self.robot, '/transport/safety_permit', '/transport/emergency_stop',
                    *['/'+self.robot+'/'+s for s in ('mecanum_drive_controller/odometry', 'joint_states', 'grasp/state')]}
        if leader:
            expected.add('/leader/'+goal_topic)
        if self.p('actuator_ack_required'):expected.add('/transport/interlock/'+self.robot)
        outputs = {'/'+self.robot+'/'+s for s in ('base_cmd', 'arm_joint_cmd', 'transport/state')}
        actual_outputs = {p.topic_name for p in (self.base_pub, self.arm_pub, self.state_pub)}
        if {s.topic_name for s in self.subscriptions} != expected or actual_outputs != outputs:
            self.destroy_node()
            raise ValueError('transport local input/output boundary violated by remapping')
        self.create_service(Trigger, 'transport/prepare', self.prepare)
        self.create_service(Trigger, 'transport/reset', self.reset)
        self.create_service(Trigger, 'transport/enable_test_delay', self.enable_test_delay)
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def clock(self, msg):
        value = msg.clock.sec+msg.clock.nanosec*1e-9
        if self.last_clock is not None and value < self.last_clock and self.was_enabled:
            self.trip('clock moved backwards')
        if self.last_clock is None or value > self.last_clock:
            self.clock_wall = time.monotonic()
        self.sim, self.last_clock = value, value

    def odometry(self, msg):
        self.encoder, self.encoder_wall = msg, time.monotonic()

    def joint_state(self, msg):
        self.joints, self.joints_wall = msg, time.monotonic()

    def gate_status(self, msg):
        try:
            value = json.loads(msg.data)
            if value['robot'] != self.robot or not isinstance(value['ready'], bool):
                raise ValueError('invalid local gate identity')
            if self.gate is not None and value['sequence'] <= self.gate['sequence']:
                raise ValueError('local gate heartbeat did not advance')
            self.gate, self.gate_wall = value, time.monotonic()
        except (ValueError, KeyError, TypeError):
            if self.was_enabled:
                self.trip('invalid local gate heartbeat')

    def actuator_status(self,msg):
        try:
            value=json.loads(msg.data)
            if (value['robot']!=self.robot or not isinstance(value['active'],bool)
                    or (self.actuator is not None and value['sequence']<=self.actuator['sequence'])):
                raise ValueError()
            self.actuator,self.actuator_wall=value,time.monotonic()
        except (ValueError,KeyError,TypeError):
            if self.was_enabled:self.trip('invalid local actuator acknowledgement')

    def allow(self, msg):
        self.permit, self.permit_wall = msg.data, time.monotonic()

    def stop_signal(self, msg):
        self.emergency = msg.data
        if msg.data and self.was_enabled:
            self.trip('shared emergency stop')

    def grasp_state(self, msg):
        self.grasp, self.grasp_wall = msg.data == 'attached', time.monotonic()

    def local_health(self):
        wall = time.monotonic()
        if self.p('actuator_ack_required') and (self.actuator is None or wall-self.actuator_wall>self.p('timeout_s')
                or not self.actuator['ready'] or self.actuator.get('fault')):
            return False,'local actuator acknowledgement not ready'
        if self.encoder is None or self.joints is None or self.gate is None or self.sim <= 0:
            return False, 'waiting for local sensors and gate'
        sensor_receipts=() if self.source_watchdog else (self.encoder_wall,self.joints_wall)
        if max(wall-v for v in (self.clock_wall,self.gate_wall,*sensor_receipts)) > self.p('timeout_s'):
            return False, 'local clock/state heartbeat missing'
        if not self.gate['ready'] or self.gate.get('fault'):
            return False, 'local safety gate not ready'
        try:
            o, j = self.encoder, self.joints
            if o.header.frame_id != self.robot+'/odom' or o.child_frame_id != self.robot+'/base_footprint':
                raise ValueError('encoder frame identity mismatch')
            ages = [self.sim-stamp(m) for m in (o, j)]
            if min(ages) < -.02 or max(ages) > .05:
                raise ValueError('stale/future local state')
            pose, twist = o.pose.pose, o.twist.twist
            finite([pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                    twist.linear.x, twist.linear.y, twist.angular.z], (10,))
            if abs(sum(getattr(pose.orientation, k)**2 for k in 'xyzw')-1.) > .001:
                raise ValueError('invalid encoder quaternion')
            if len(set(j.name)) != len(j.name) or len(j.position) != len(j.name):
                raise ValueError('invalid local joint state')
            mapping = dict(zip(j.name, j.position))
            q = finite([mapping[n] for n in self.model.names], (5,))
            if np.any(q < self.model.lower) or np.any(q > self.model.upper):
                raise ValueError('joint state outside model')
            return True, ''
        except (ValueError, KeyError) as exc:
            return False, str(exc)

    def prepare(self, request, response):
        # Graph discovery can block; perform it explicitly while disarmed,
        # never in the periodic actuation/watchdog callback.
        good, reason = self.local_health()
        if self.permit or self.latched:
            good, reason = False, 'disarm/reset before prepare'
        for topic, node, ns in [('mecanum_drive_controller/odometry', 'mecanum_drive_controller', '/'+self.robot),
                ('joint_states', 'joint_state_broadcaster', '/'+self.robot),
                ('base_cmd', self.get_name(), '/'+self.robot), ('arm_joint_cmd', self.get_name(), '/'+self.robot)]:
            writers = self.get_publishers_info_by_topic('/'+self.robot+'/'+topic)
            if len(writers) != 1 or writers[0].node_name != node or writers[0].node_namespace != ns:
                good, reason = False, 'unverified publisher for '+topic
        if self.p('actuator_ack_required'):
            writers=self.get_publishers_info_by_topic('/transport/interlock/'+self.robot)
            if len(writers)!=1 or writers[0].node_name!='actuator_interlock' or writers[0].node_namespace!='/'+self.robot:
                good,reason=False,'unverified local actuator acknowledgement publisher'
        self.prepared = good
        response.success, response.message = good, 'local sources and command ownership verified' if good else reason
        return response

    def reset(self, request, response):
        good, reason = self.local_health()
        stopped = self.encoder is not None and max(abs(self.encoder.twist.twist.linear.x),
            abs(self.encoder.twist.twist.linear.y), abs(self.encoder.twist.twist.angular.z)) < .0001
        response.success = good and stopped and not self.permit and self.emergency
        if response.success:
            self.latched, self.was_enabled, self.prepared = False, False, False
            self.delay_active = False
            self.pending_commands.clear()
            self.goal_data, self.enabled_since = None, None
            self.home_sent, self.homed = False, False
            self.home_last_send = 0.
            self.stop_future = None
            self.state, self.reason = 'SAFE_STOP', 'explicit prepare and arm required'
        response.message = self.reason if response.success else reason or 'disarm and stop first'
        return response

    def trip(self, reason):
        if not self.latched:
            self.reason = reason
        self.latched, self.state, self.goal_data, self.last_velocity = True, 'SAFE_STOP', None, 0.
        self.pending_commands.clear()
        if self.stop_future is None and self.stop_client.service_is_ready():
            self.stop_future = self.stop_client.call_async(Trigger.Request())

    def goal(self, msg):
        try:
            if (self.state != 'READY' or self.latched or not self.local_health()[0]
                    or not self.permit or self.emergency or time.monotonic()-self.permit_wall > self.p('timeout_s')):
                raise ValueError('leader is not READY')
            if not self.grasp or time.monotonic()-self.grasp_wall > 1.:
                raise ValueError('local grasp is not confirmed')
            distance, y, z = finite([msg.vector.x, msg.vector.y, msg.vector.z], (3,))
            if (msg.header.frame_id != 'leader/base_footprint' or
                    not 0 < abs(distance) <= self.p('max_distance_m') or y != 0 or z != 0 or
                    stamp(msg) <= self.last_goal_stamp or not -.02 <= self.sim-stamp(msg) <= self.p('timeout_s')):
                raise ValueError('goal must be fresh local relative x within configured envelope')
            self.goal_data = (self.sim+.05, float(distance), self.encoder.pose.pose.position.x,
                              self.encoder.pose.pose.position.y,
                              2*math.atan2(self.encoder.pose.pose.orientation.z, self.encoder.pose.pose.orientation.w))
            self.last_goal_stamp, self.last_velocity = stamp(msg), 0.
            self.accepted += 1
            self.state, self.reason = 'TRANSPORT', ''
        except ValueError as exc:
            self.rejected += 1
            self.last_rejection = str(exc)

    def base_request(self, command):
        return np.array([command, 0., 0.])

    def publish_command(self, publisher, message):
        # Opt-in command-transport experiment. Preserve original header stamps;
        # never hide an old request from the downstream safety checks.
        if self.delay_active and self.p('command_delay_s') > 0 and not self.latched:
            if len(self.pending_commands) >= 256:
                self.trip('command delay queue overflow')
                return
            self.pending_commands.append((self.sim+self.p('command_delay_s'), publisher, message))
        else:
            publisher.publish(message)

    def enable_test_delay(self, request, response):
        response.success = self.homed and self.permit and not self.latched and self.local_health()[0]
        if response.success: self.delay_active = True
        response.message = 'test command delay enabled; original stamps retained' if response.success else 'local ready/armed state required'
        return response

    def tick(self):
        wall = time.monotonic()
        good, reason = self.local_health()
        enabled = (good and self.prepared and self.permit and not self.emergency
                   and wall-self.permit_wall <= self.p('timeout_s') and not self.latched)
        if self.was_enabled and not enabled:
            self.trip(reason or 'shared permission lost')
        command = 0.
        if enabled:
            self.was_enabled = True
            acknowledged = (self.gate.get('armed', False) and
                            self.sim > self.gate.get('armed_sim_s', self.sim)+.05)
            if self.p('actuator_ack_required'):
                acknowledged = acknowledged and self.actuator.get('active',False)
            if self.enabled_since is None and acknowledged:
                self.enabled_since = self.sim
            # Start only after this local gate acknowledges its own arm epoch.
            if self.enabled_since is not None and acknowledged:
                mapping = dict(zip(self.joints.name, self.joints.position))
                home_error = max(abs(mapping[n]-v) for n, v in zip(self.model.names, self.home))
                if not self.homed and self.sim-self.enabled_since > 5.:
                    self.trip('local home did not converge')
                if not self.latched and (not self.home_sent or (not self.homed and home_error >= .001
                        and self.sim-self.home_last_send >= .2)):
                    msg = JointTrajectory(joint_names=list(self.model.names))
                    msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), self.robot+'/base_footprint'
                    msg.points = [JointTrajectoryPoint(positions=self.home.tolist())]
                    # End-to-end measured convergence acknowledges home. A lost
                    # one-shot must not leave the arm at its initial hold. Fresh
                    # bounded retries keep the ORIGINAL absolute home deadline.
                    remaining_ns=int(max(.1,self.enabled_since+3.-self.sim)*1e9)
                    msg.points[0].time_from_start.sec,msg.points[0].time_from_start.nanosec=divmod(remaining_ns,1000000000)
                    self.publish_command(self.arm_pub, msg)
                    self.home_sent,self.home_last_send = True,self.sim
                if not self.homed and self.sim-self.enabled_since > 3.2 and home_error < .001:
                    self.homed, self.state, self.reason = True, 'READY', ''
                if self.leader and self.base_trajectory and self.state == 'TRANSPORT' and self.goal_data is not None:
                    start, distance, x0, y0, yaw = self.goal_data
                    t = self.sim-start
                    reference_acceleration = self.p('straight_reference_acceleration_m_s2') or self.p('acceleration_m_s2')
                    ref, velocity, duration = straight_reference(t, distance, self.p('speed_m_s'), reference_acceleration)
                    x = ((self.encoder.pose.pose.position.x-x0)*math.cos(yaw)
                         +(self.encoder.pose.pose.position.y-y0)*math.sin(yaw))
                    at_target = t >= duration and abs(distance-x) <= self.p('goal_tolerance_m')
                    desired = 0. if at_target else velocity+self.p('position_gain_s_inv')*(ref-x)
                    command = float(np.clip(desired, -self.p('speed_m_s'), self.p('speed_m_s')))
                    dt = max(0., self.sim-self.last_tick_sim) if self.last_tick_sim is not None else 0.
                    bound = self.p('acceleration_m_s2')*dt
                    command = float(np.clip(command, self.last_velocity-bound, self.last_velocity+bound))
                    if at_target and abs(command) < 1e-12:
                        self.completed += 1
                        self.state, self.goal_data, command = 'READY', None, 0.
                    elif t > duration+3.:
                        self.trip('leader local tracking timeout')
                        command = 0.
                if self.latched:
                    command = 0.
        actual_request = np.zeros(3)
        if self.latched or (enabled and self.enabled_since is not None and acknowledged):
            msg = TwistStamped()
            msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), self.robot+'/base_footprint'
            actual_request = finite(self.base_request(command), (3,)) if not self.latched else np.zeros(3)
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = map(float, actual_request)
            self.publish_command(self.base_pub, msg)
        if enabled and not self.latched:
            while self.pending_commands and self.pending_commands[0][0] <= self.sim:
                _, publisher, message = self.pending_commands.popleft()
                publisher.publish(message)
        else:
            self.pending_commands.clear()
        self.last_velocity, self.last_tick_sim = command, self.sim
        self.state_pub.publish(String(data=json.dumps({'state': self.state, 'reason': self.reason,
            'local_ready': good, 'local_reason': reason,
            'role': self.role, 'prepared': self.prepared,
            'homed': self.homed, 'accepted_goals': self.accepted, 'rejected_goals': self.rejected,
            'completed_goals': self.completed, 'last_rejection': self.last_rejection,
            'command_vx': float(actual_request[0]), 'command_vy': float(actual_request[1]),
            'command_wz': float(actual_request[2]), 'sim_time': self.sim})))


def run(leader, args=None):
    rclpy.init(args=args)
    node = TransportEndpoint(leader=leader)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def leader_main(args=None):
    run(True, args)


def follower_main(args=None):
    run(False, args)
