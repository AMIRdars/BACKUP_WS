"""Local command interlock and independent shared-stop supervisor.

No leader state, trajectory, TF, or simulator truth enters a follower gate.
Thresholds on joint reaction torque come directly from the selected URDF.
"""
import hashlib
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .wrench import LocalModel, finite
from .sensor_coherence import coherent_stamps,local_sensor_qos
from .arm_servo_response import validate_response
from .servo_streaming_gate import servo_reference_ratio


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


class RobotSafetyGate(Node):
    def __init__(self, **kwargs):
        super().__init__('safety_gate', **kwargs)
        for key, value in {'urdf_file': '', 'profile_file': '', 'max_age_s': .05,
                'max_skew_s': .02, 'heartbeat_timeout_s': .15,
                'command_timeout_s': .15}.items():
            self.declare_parameter(key, value)
        self.declare_parameter('base_zero_only', False)
        self.declare_parameter('coherent_sensor_history', False)
        self.declare_parameter('simulation_source_watchdog',False)
        self.declare_parameter('servo_streaming_response_file','')
        self.source_watchdog=self.get_parameter('simulation_source_watchdog').value
        self.coherent_history=self.get_parameter('coherent_sensor_history').value
        if not isinstance(self.coherent_history,bool):raise ValueError('boolean coherent_sensor_history required')
        self.base_zero_only = self.get_parameter('base_zero_only').value
        if not isinstance(self.base_zero_only, bool): raise ValueError('boolean base_zero_only required')
        self.p = lambda key: self.get_parameter(key).value
        self.robot = self.get_namespace().strip('/')
        if not self.robot or '/' in self.robot or not self.get_parameter('use_sim_time').value:
            raise ValueError('single local namespace and use_sim_time=true required')
        xml = Path(self.p('urdf_file')).read_text()
        self.model = LocalModel(xml)
        self.servo_tau=None;self.servo_period=None;self.servo_reference_ratio_peak=0.
        if self.p('servo_streaming_response_file'):
            if not self.base_zero_only or not self.source_watchdog:
                raise ValueError('servo streaming requires explicit local zero-base simulation scope')
            cert=json.loads(Path(self.p('servo_streaming_response_file')).read_text())
            self.servo_tau=validate_response(cert,xml,Path(self.p('urdf_file')).parent)
            if (np.any(self.servo_tau<=0) or cert.get('open_loop_control') is not True
                    or hashlib.sha256(Path('/opt/ros/humble/lib/libgz_hardware_plugins.so').read_bytes()).hexdigest()!=cert['plugin_library_sha256']):
                raise ValueError('five measured joint responses and matching simulation plugin required')
            self.servo_period=1/float(cert['controller_update_rate_hz'])
        self.profile = json.loads(Path(self.p('profile_file')).read_text())
        if (self.profile.get('robot') != self.robot or self.profile.get('urdf_sha256') !=
                hashlib.sha256(xml.encode()).hexdigest() or not self.profile.get('simulation_only')
                or not self.profile.get('joint_reaction_projection_verified')):
            raise ValueError('matching simulation safety profile required')
        self.max_base = finite(self.profile['max_base_velocity'], (3,))
        if 'controller_yaml_file' in self.profile and hashlib.sha256(
                Path(self.profile['controller_yaml_file']).read_bytes()).hexdigest() != self.profile.get('controller_yaml_sha256'):
            raise ValueError('controller configuration differs from measured safety profile')
        if 'base_controller_yaml_file' in self.profile and hashlib.sha256(
                Path(self.profile['base_controller_yaml_file']).read_bytes()).hexdigest() != self.profile.get('base_controller_yaml_sha256'):
            raise ValueError('base controller differs from measured safety profile')
        self.margin = float(self.profile['joint_margin_rad'])
        if (np.any(self.max_base <= 0) or not 0 < self.margin < np.min(self.model.upper-self.model.lower)/2
                or np.any(self.model.effort <= 0) or np.any(self.model.velocity <= 0)):
            raise ValueError('invalid model/motion limits')
        for key in ('max_age_s', 'max_skew_s', 'heartbeat_timeout_s', 'command_timeout_s'):
            if not np.isfinite(self.p(key)) or self.p(key) <= 0:
                raise ValueError('positive watchdog interval required')
        self.samples, self.received = {}, {}
        self.sensor_history, self.coherent_sample_stamps = {}, {}
        self.sim, self.last_clock, self.clock_wall = 0., None, 0.
        self.fault, self.problem = '', 'waiting for local sensors'
        self.fault_context = {}
        self.permit, self.permit_wall, self.shared_stop = False, 0., True
        self.armed_once = False
        self.armed_sim = 0.
        self.grasp, self.grasp_ever, self.grasp_sim, self.grasp_wall = False, False, 0., 0.
        self.command, self.command_wall, self.command_stamp = np.zeros(3), 0., 0.
        self.q, self.hold_q, self.torques = None, None, None
        self.peak_torque = np.zeros(5)
        self.output_active, self.sequence, self.last_tick_wall = False, 0, 0.
        self.base_pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 1)
        self.status_pub = self.create_publisher(String, '/transport/safety/'+self.robot, 10)
        self.debug_pub = self.create_publisher(String, 'safety_debug', 10)
        input_qos=local_sensor_qos(self.source_watchdog)
        self.create_subscription(ClockMessage, '/clock', self.clock, input_qos)
        self.create_subscription(JointState, 'joint_states', lambda m: self.receive('joints', m), input_qos)
        self.create_subscription(Odometry, 'mecanum_drive_controller/odometry',
                                 lambda m: self.receive('encoder', m), input_qos)
        for i, name in enumerate(self.model.names):
            topic = 'ft_sensor' if i == 4 else 'joint_load/'+name
            self.create_subscription(WrenchStamped, topic,
                                     lambda m, k=name: self.receive(k, m), input_qos)
        self.create_subscription(TwistStamped, 'base_cmd', self.base_command, 1)
        self.create_subscription(JointTrajectory, 'arm_joint_cmd', self.arm_command, 1)
        self.create_subscription(String, 'grasp/state', self.grasp_state, 10)
        self.create_subscription(Bool, '/transport/safety_permit', self.allow, 1)
        self.create_subscription(Bool, '/transport/emergency_stop', self.emergency, LATCHED)
        expected = {'/clock', '/transport/safety_permit', '/transport/emergency_stop',
                    *[f'/{self.robot}/{s}' for s in ('joint_states', 'mecanum_drive_controller/odometry',
                      'ft_sensor', 'base_cmd', 'arm_joint_cmd', 'grasp/state', *('joint_load/'+n for n in self.model.names[:-1]))]}
        if {s.topic_name for s in self.subscriptions} != expected:
            raise ValueError('local safety input boundary violated by remapping')
        self.create_service(Trigger, 'safety/reset', self.reset)
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def clock(self, msg):
        current = msg.clock.sec+msg.clock.nanosec*1e-9
        if self.last_clock is not None and current < self.last_clock:
            self.trip('simulation clock moved backwards')
            self.samples.clear()
            self.sensor_history.clear()
        if self.last_clock is None or current > self.last_clock:
            self.clock_wall = time.monotonic()
        self.last_clock, self.sim = current, current

    def grasp_state(self, msg):
        self.grasp = msg.data == 'attached'
        self.grasp_sim, self.grasp_wall = self.sim, time.monotonic()
        self.grasp_ever = self.grasp_ever or self.grasp
        if self.grasp_ever and not self.grasp:
            self.trip('local grasp detached')

    def receive(self, key, msg):
        s = stamp(msg)
        if s <= 0 or (key in self.samples and s <= stamp(self.samples[key])):
            self.trip(key+': timestamp did not advance')
            return
        self.samples[key], self.received[key] = msg, time.monotonic()
        if self.coherent_history:
            self.sensor_history.setdefault(key,deque(maxlen=64)).append((s,self.received[key]))
        if key in self.model.names:
            index = self.model.names.index(key)
            expected = (self.robot+'/Joint_5/wrist_ft_sensor' if index == 4 else
                        self.robot+'/'+key+'/joint_load_'+key)
            w = msg.wrench
            try:
                vector = finite([w.force.x, w.force.y, w.force.z,
                                 w.torque.x, w.torque.y, w.torque.z], (6,))
                if msg.header.frame_id != expected:
                    raise ValueError('unexpected sensor frame')
                torque = abs(float(self.model.axes[index]@vector[3:]))
                self.peak_torque[index] = max(self.peak_torque[index], torque)
                if torque >= self.model.effort[index]:
                    self.trip(f'{key}: measured reaction torque {torque:.9g} Nm >= URDF effort {self.model.effort[index]:.9g} Nm at {s:.9f} s')
            except ValueError as exc:
                self.trip(key+': '+str(exc))

    def allow(self, msg):
        was_allowed = self.permit
        self.permit, self.permit_wall = msg.data, time.monotonic()
        if msg.data and not was_allowed:
            self.command[:] = 0
            self.command_wall = self.command_stamp = 0.
            self.hold_q = None
            self.armed_once = True
            self.armed_sim = self.sim
            self.peak_torque[:] = 0.

    def emergency(self, msg):
        self.shared_stop = msg.data
        if msg.data:
            self.command[:] = 0

    def trip(self, reason):
        if self.armed_once or self.output_active:
            if not self.fault:
                self.fault = reason
                wall = time.monotonic()
                self.fault_context = {'sim_time': self.sim, 'reason': reason,
                    'sample_stamps_s': {key: stamp(msg) for key, msg in self.samples.items()},
                    'sample_wall_ages_s': {key: wall-value for key, value in self.received.items()}}
        self.problem = reason
        self.command[:] = 0

    def healthy(self):
        wall = time.monotonic()
        needed = {'joints', 'encoder', *self.model.names}
        if not needed.issubset(self.samples) or self.sim <= 0:
            return False, 'missing local sensors/clock'
        if wall-self.clock_wall > self.p('heartbeat_timeout_s'):
            return False, 'simulation clock stalled'
        if self.grasp_ever and (not self.grasp or self.sim-self.grasp_sim > .35 or wall-self.grasp_wall > 1.):
            return False, 'local grasp heartbeat missing/detached'
        stamps = np.array([stamp(self.samples[k]) for k in needed])
        self.coherent_sample_stamps={}
        if self.coherent_history:
            self.coherent_sample_stamps=coherent_stamps(self.sensor_history,needed,self.sim,wall,
                self.p('max_age_s'),self.p('max_skew_s'),
                float('inf') if self.source_watchdog else self.p('heartbeat_timeout_s')) or {}
        if (np.max(self.sim-stamps) > self.p('max_age_s') or
                np.max(stamps-self.sim) > self.p('max_skew_s') or
                (not self.coherent_sample_stamps if self.coherent_history else np.ptp(stamps) > self.p('max_skew_s')) or
                (not self.source_watchdog and max(wall-self.received[k] for k in needed) > self.p('heartbeat_timeout_s'))):
            return False, 'local sensors stale, future or skewed'
        try:
            msg = self.samples['joints']
            if (len(set(msg.name)) != len(msg.name) or len(msg.position) != len(msg.name)
                    or len(msg.velocity) != len(msg.name)):
                raise ValueError('malformed joint state')
            q = dict(zip(msg.name, msg.position))
            v = dict(zip(msg.name, msg.velocity))
            q, v = finite([q[n] for n in self.model.names], (5,)), finite([v[n] for n in self.model.names], (5,))
            if np.any(q <= self.model.lower+self.margin) or np.any(q >= self.model.upper-self.margin):
                raise ValueError('joint position reached software margin')
            if np.any(np.abs(v) > self.model.velocity+1e-6):
                raise ValueError('joint velocity exceeds URDF limit')
            encoder = self.samples['encoder']
            if (encoder.header.frame_id != self.robot+'/odom' or
                    encoder.child_frame_id != self.robot+'/base_footprint'):
                raise ValueError('unexpected encoder frame')
            t = encoder.twist.twist
            finite([t.linear.x, t.linear.y, t.angular.z], (3,))
            pose = encoder.pose.pose
            finite([pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], (7,))
            norm = sum(getattr(pose.orientation, k)**2 for k in 'xyzw')
            if abs(norm-1.) > 1e-3:
                raise ValueError('invalid encoder orientation')
            torques = []
            for i, name in enumerate(self.model.names):
                m = self.samples[name]
                expected = (self.robot+'/Joint_5/wrist_ft_sensor' if i == 4 else
                            self.robot+'/'+name+'/joint_load_'+name)
                if m.header.frame_id != expected:
                    raise ValueError('unexpected joint reaction frame')
                w = m.wrench
                vector = finite([w.force.x, w.force.y, w.force.z,
                                 w.torque.x, w.torque.y, w.torque.z], (6,))
                torques.append(abs(float(self.model.axes[i]@vector[3:])))
            if np.any(np.asarray(torques) >= self.model.effort):
                raise ValueError('measured reaction torque exceeds URDF effort')
            self.q, self.torques = q, torques
            return True, ''
        except (ValueError, KeyError) as exc:
            return False, str(exc)

    def enabled(self):
        return (self.permit and not self.shared_stop and not self.fault and
                time.monotonic()-self.permit_wall <= self.p('heartbeat_timeout_s'))

    def base_command(self, msg):
        if not self.enabled():
            return
        try:
            t = msg.twist
            v = finite([t.linear.x, t.linear.y, t.angular.z], (3,))
            if self.base_zero_only:
                if np.any(finite([t.linear.x, t.linear.y, t.linear.z,
                                  t.angular.x, t.angular.y, t.angular.z], (6,)) != 0.):
                    raise ValueError('ARM_ONLY rejected nonzero base command')
                # ARM_ONLY emits zero even before arming. A queued pre-arm zero
                # cannot authorize motion; discard it at the epoch boundary.
                # Freshness checks for post-arm samples remain unchanged.
                if msg.header.frame_id == self.robot+'/base_footprint' and stamp(msg) <= self.armed_sim:
                    return
            if np.any(v) and self.profile.get('require_grasp_for_base_motion') and not self.grasp:
                raise ValueError('base motion requires confirmed local grasp')
            if (msg.header.frame_id != self.robot+'/base_footprint' or
                    stamp(msg) <= self.armed_sim or
                    not -self.p('max_skew_s') <= self.sim-stamp(msg) <= self.p('command_timeout_s') or
                    np.any(np.abs(v) > self.max_base+1e-9) or
                    any(abs(x) > 1e-12 for x in (t.linear.z, t.angular.x, t.angular.y))):
                raise ValueError('invalid, stale or out-of-envelope base command')
            self.command, self.command_stamp, self.command_wall = v, stamp(msg), time.monotonic()
        except ValueError as exc:
            self.trip(str(exc))

    def arm_command(self, msg):
        if not self.enabled():
            return
        try:
            if (msg.header.frame_id != self.robot+'/base_footprint' or
                    not self.armed_sim < stamp(msg) <= self.sim+self.p('max_skew_s') or
                    self.sim-stamp(msg) > self.p('command_timeout_s')):
                raise ValueError('stale or invalid arm request header: '
                    f'frame={msg.header.frame_id!r}, stamp={stamp(msg):.9f}, '
                    f'armed={self.armed_sim:.9f}, now={self.sim:.9f}')
            if tuple(msg.joint_names) != self.model.names or not msg.points or self.q is None:
                raise ValueError('complete local arm trajectory required')
            servo_now=False
            tau=getattr(self,'servo_tau',None)
            if tau is not None and len(msg.points)==2:
                first=msg.points[0];last=msg.points[1]
                t0=first.time_from_start.sec+first.time_from_start.nanosec*1e-9
                t1=last.time_from_start.sec+last.time_from_start.nanosec*1e-9
                servo_now=(abs(t0-self.servo_period)<1e-9 and abs(stamp(msg)+t0-self.sim)<=self.p('max_skew_s'))
                if servo_now:
                    good,reason=self.healthy()
                    if not good:raise ValueError('servo command without fresh local state: '+reason)
                    ratio=servo_reference_ratio(self.q,first.positions,last.positions,
                        stamp(msg)+t0,stamp(msg)+t1,self.sim,tau,self.model.velocity)
                    self.servo_reference_ratio_peak=max(self.servo_reference_ratio_peak,ratio)
            previous, previous_time = self.q, 0.
            for index,point in enumerate(msg.points):
                if point.effort or point.accelerations or point.velocities:
                    raise ValueError('positions-only arm requests required by this gate')
                q = finite(point.positions, (5,))
                t = point.time_from_start.sec+point.time_from_start.nanosec*1e-9
                if (t <= previous_time or np.any(q <= self.model.lower+self.margin)
                        or np.any(q >= self.model.upper-self.margin)
                        or (not (servo_now and index==0) and np.any(np.abs(q-previous)/(t-previous_time) > self.model.velocity))):
                    raise ValueError('arm command violates model limits')
                previous, previous_time = q, t
            if stamp(msg)+previous_time <= self.sim:
                raise ValueError('arm trajectory ended before receipt')
            good,reason=self.healthy()
            if not good:
                raise ValueError('arm command without fresh local state: '+reason)
            self.arm_pub.publish(msg)
        except ValueError as exc:
            self.trip(str(exc))

    def reset(self, request, response):
        good, reason = self.healthy()
        encoder = self.samples.get('encoder')
        stopped = encoder is not None and max(abs(encoder.twist.twist.linear.x),
            abs(encoder.twist.twist.linear.y), abs(encoder.twist.twist.angular.z)) < .001
        response.success = good and stopped and not self.permit
        if response.success:
            self.fault, self.armed_once = '', False
            self.fault_context = {}
            self.command[:] = 0
        response.message = 'reset; explicit shared arm still required' if response.success else reason or 'disarm and stop first'
        return response

    def tick(self):
        good, reason = self.healthy()
        wall = time.monotonic()
        if not good:
            self.trip(reason)
        if self.permit and wall-self.permit_wall > self.p('heartbeat_timeout_s'):
            self.trip('shared permit heartbeat missing')
        if np.any(self.command) and (wall-self.command_wall > self.p('command_timeout_s') or
                                   self.sim-self.command_stamp > self.p('command_timeout_s')):
            self.trip('base command watchdog')
        active = good and self.enabled()
        output = Twist()
        if active:
            output.linear.x, output.linear.y, output.angular.z = map(float, self.command)
        else:
            self.command[:] = 0
            if self.q is not None:
                if self.hold_q is None or self.output_active:
                    self.hold_q = self.q.copy()
                trajectory = JointTrajectory(joint_names=list(self.model.names))
                point = JointTrajectoryPoint(positions=self.hold_q.tolist())
                point.time_from_start.nanosec = 100000000
                trajectory.points = [point]
                self.arm_pub.publish(trajectory)
        self.base_pub.publish(output)
        self.output_active = active
        self.sequence += 1
        self.status_pub.publish(String(data=json.dumps({'robot': self.robot, 'ready': good,
            'fault': self.fault, 'reason': reason, 'sequence': self.sequence,
            'armed': self.enabled(), 'armed_sim_s': self.armed_sim, 'active': active,
            'base_zero_only': self.base_zero_only})))
        self.debug_pub.publish(String(data=json.dumps({'sim_time': self.sim,
            'coherent_sensor_history':self.coherent_history,
            'simulation_source_watchdog':self.source_watchdog,
            'coherent_sample_stamps_s':self.coherent_sample_stamps,
            'active': active, 'fault': self.fault, 'reason': reason,
            'first_fault_context': self.fault_context,
            'tick_gap_wall_s': wall-self.last_tick_wall if self.last_tick_wall else 0.,
            'servo_streaming_model_enabled':self.servo_tau is not None,
            'servo_reference_URDF_velocity_ratio_peak':self.servo_reference_ratio_peak,
            'reaction_torque_Nm': self.torques, 'peak_reaction_torque_Nm': self.peak_torque.tolist(),
            'effort_limit_Nm': self.model.effort.tolist()})))
        self.last_tick_wall = wall


class SafetySupervisor(Node):
    def __init__(self, **kwargs):
        super().__init__('safety_supervisor', **kwargs)
        self.declare_parameter('robots', ['leader', 'follower'])
        self.declare_parameter('heartbeat_timeout_s', .15)
        self.declare_parameter('check_writers_on_arm', True)
        self.declare_parameter('require_interlocks', False)
        self.robots = self.get_parameter('robots').value
        if len(self.robots) != 2 or len(set(self.robots)) != 2:
            raise ValueError('two distinct robot safety endpoints required')
        self.status, self.seen, self.armed, self.fault = {}, {}, False, ''
        self.pending_arm = False
        self.arm_deadline = 0.
        self.stop_pub = self.create_publisher(Bool, '/transport/emergency_stop', LATCHED)
        self.permit_pub = self.create_publisher(Bool, '/transport/safety_permit', 1)
        self.state_pub = self.create_publisher(String, '/transport/safety_status', 10)
        for robot in self.robots:
            self.create_subscription(String, '/transport/safety/'+robot,
                                     lambda m, r=robot: self.receive(r, m), 10)
        self.interlocks, self.interlock_seen = {}, {}
        self.controller_stops = {}
        if self.get_parameter('require_interlocks').value:
            from controller_manager_msgs.srv import SwitchController
            from action_msgs.srv import CancelGoal
            for robot in self.robots:
                self.create_subscription(String, '/transport/interlock/'+robot,
                    lambda m,r=robot: self.interlock_status(r,m), 10)
                self.controller_stops[robot] = (
                    self.create_client(SwitchController, '/'+robot+'/controller_manager/switch_controller'),
                    self.create_client(CancelGoal, '/'+robot+'/arm_controller/follow_joint_trajectory/_action/cancel_goal'))
        self.stop_futures, self.last_controller_stop = {}, 0.
        self.create_service(Trigger, '/transport/safety/arm', self.arm)
        self.create_service(Trigger, '/transport/safety/reset', self.reset)
        self.create_service(Trigger, '/transport/safety/stop', self.stop)
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.stop_pub.publish(Bool(data=True))

    def receive(self, robot, msg):
        try:
            value = json.loads(msg.data)
            if (value['robot'] != robot or not isinstance(value['ready'], bool) or
                    (robot in self.status and value['sequence'] <= self.status[robot]['sequence'])):
                raise ValueError('invalid safety heartbeat')
            self.status[robot], self.seen[robot] = value, time.monotonic()
        except (ValueError, KeyError, TypeError) as exc:
            if self.armed:
                self.fault = robot+': '+str(exc)

    def healthy(self):
        for robot in self.robots:
            value = self.status.get(robot)
            if value is None or time.monotonic()-self.seen[robot] > self.get_parameter('heartbeat_timeout_s').value:
                return False, robot+': safety heartbeat missing'
            if not value['ready'] or value['fault']:
                return False, robot+': '+(value['fault'] or value.get('reason', 'not ready'))
        if self.get_parameter('require_interlocks').value:
            for robot in self.robots:
                value = self.interlocks.get(robot)
                if value is None or time.monotonic()-self.interlock_seen[robot] > self.get_parameter('heartbeat_timeout_s').value:
                    return False, robot+': actuator interlock heartbeat missing'
                if not value['ready'] or value['fault']:
                    return False, robot+': '+(value['fault'] or value.get('reason','interlock not ready'))
        return True, ''

    def interlock_status(self, robot, msg):
        try:
            value = json.loads(msg.data)
            if (value['robot']!=robot or not isinstance(value['ready'],bool)
                    or (robot in self.interlocks and value['sequence']<=self.interlocks[robot]['sequence'])):
                raise ValueError('invalid interlock heartbeat')
            self.interlocks[robot], self.interlock_seen[robot] = value, time.monotonic()
        except (ValueError,KeyError,TypeError):
            if self.armed: self.fault = robot+': invalid interlock heartbeat'

    def stop_controllers(self):
        # Service stop is independent of the final command process. Never activate
        # controllers automatically; manual recovery/re-arm is required.
        if not self.controller_stops or time.monotonic()-self.last_controller_stop < .2:
            return
        from controller_manager_msgs.srv import SwitchController
        from action_msgs.srv import CancelGoal
        self.last_controller_stop = time.monotonic()
        for robot,(switch,cancel) in self.controller_stops.items():
            future = self.stop_futures.get(robot)
            if future is not None and not future.done(): continue
            if future is not None and future.done() and future.exception() is None and future.result().ok: continue
            if cancel.service_is_ready(): cancel.call_async(CancelGoal.Request())
            if switch.service_is_ready():
                req = SwitchController.Request(deactivate_controllers=['mecanum_drive_controller','arm_controller'],
                                               strictness=SwitchController.Request.BEST_EFFORT)
                req.timeout.sec = 1
                self.stop_futures[robot] = switch.call_async(req)

    def arm(self, request, response):
        good, reason = self.healthy()
        # Discovery queries may block on middleware locks. Keep them outside
        # either robot's periodic actuation/sensor-watchdog callback.
        if good and self.get_parameter('check_writers_on_arm').value:
            for robot in self.robots:
                owner = 'actuator_interlock' if self.get_parameter('require_interlocks').value else 'safety_gate'
                for suffix, expected in [('rover_twist', owner),
                    ('arm_controller/joint_trajectory', owner),
                    ('mecanum_drive_controller/reference_unstamped', 'rover_twist_relay_ign')]:
                    writers = self.get_publishers_info_by_topic('/'+robot+'/'+suffix)
                    if len(writers) != 1 or writers[0].node_name != expected or writers[0].node_namespace != '/'+robot:
                        good, reason = False, robot+': command writer ownership not confirmed for '+suffix
        response.success = good and not self.fault and not self.armed and not self.pending_arm
        if response.success:
            self.pending_arm = True
            self.arm_deadline = time.monotonic()+2*self.get_parameter('heartbeat_timeout_s').value
        response.message = 'arm accepted; awaiting fresh safety heartbeats' if response.success else self.fault or reason
        return response

    def reset(self, request, response):
        good, reason = self.healthy()
        response.success = good and not self.armed
        if response.success:
            self.fault = ''
            self.pending_arm = False
            self.stop_futures.clear()
            self.last_controller_stop = 0.
            self.stop_pub.publish(Bool(data=True))
        response.message = 'reset; explicit arm required' if response.success else reason or 'disarm first'
        return response

    def stop(self, request, response):
        self.fault, self.armed, self.pending_arm = self.fault or 'requested stop', False, False
        self.stop_pub.publish(Bool(data=True))
        self.permit_pub.publish(Bool(data=False))
        response.success, response.message = True, self.fault
        return response

    def tick(self):
        good, reason = self.healthy()
        if self.pending_arm and time.monotonic() > self.arm_deadline:
            self.fault, self.pending_arm = 'arm handshake heartbeat timeout', False
        if self.pending_arm and good and not self.fault:
            self.armed, self.pending_arm = True, False
            self.stop_pub.publish(Bool(data=False))
        if self.armed and (not good or self.fault):
            self.fault, self.armed = self.fault or reason, False
            self.stop_pub.publish(Bool(data=True))
        if self.fault:
            self.stop_controllers()
        self.permit_pub.publish(Bool(data=self.armed and good and not self.fault))
        self.state_pub.publish(String(data=json.dumps({'armed': self.armed,
            'pending_arm': self.pending_arm,
            'fault': self.fault, 'ready': good, 'reason': reason})))


def run(kind, args):
    rclpy.init(args=args)
    node = kind()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def gate_main(args=None):
    run(RobotSafetyGate, args)


def supervisor_main(args=None):
    run(SafetySupervisor, args)
