"""Independent 100 Hz local follower, with measured calibration gates.

No TF listener is used: exact local sensor/grasp transforms come from the
same stamped joint state via the audited URDF chain. No /odom GT is read.
"""
import hashlib
import json
from collections import deque
from pathlib import Path
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .core import (Admittance4D, ArmURDF, Recenter, WrenchObserver, allocate,
                   allowed_subscriptions, check_timing, finite, interpolate_joints, transform_wrench)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_controller', start_parameter_services=False,
                         enable_rosout=True)
        defaults = {
            'robot_description': '', 'calibration_file': '', 'log_file': '',
            'axes_enabled': [True, False, False, False], 'mode': 'B',
            'control_rate_hz': 100., 'bias_samples': 100, 'cutoff_hz': 8.,
            'max_age_s': .05, 'max_skew_s': .02,
            'grasp_timeout_s': .35,
            'sensor_sign': 0., 'sensor_header_frame': '',
            'max_abs_wrench': [50., 50., 100., 5., 5., 5.],
            'deadband': [.2, .2, .2, .02], 'soft_limit': [20., 20., 20., 2.],
            'recenter_enabled': False, 'residual_limit': .08,
            'recenter_start_m':.02, 'recenter_stop_m':.008,
            'max_tracking_error_rad': .04,
            'gravity_B': [0.,0.,-9.81],
            'admittance.mass': [3.,3.,3.,.4],
            'admittance.damping': [35.,35.,35.,4.],
            'admittance.stiffness': [20.,20.,20.,2.],
            'admittance.max_velocity': [.03,.03,.02,.08],
            'admittance.max_offset': [.05,.05,.03,.1],
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        if not self.has_parameter('use_sim_time') or not self.p('use_sim_time'):
            raise ValueError('use_sim_time=true is required')
        self.robot = self.get_namespace().strip('/')
        if not self.robot or '/' in self.robot:
            raise ValueError('one explicit robot namespace required')
        self.prefix = '/' + self.robot
        self.arm = ArmURDF(self.p('robot_description'))
        self.adm = self._new_admittance()
        for key,size in [('max_abs_wrench',6),('soft_limit',4)]:
            if np.any(finite(self.p(key),(size,)) <= 0):
                raise ValueError('positive wrench limits required')
        if np.any(finite(self.p('deadband'),(4,)) < 0):
            raise ValueError('nonnegative deadband required')
        self.observer = WrenchObserver(int(self.p('bias_samples')), self.p('cutoff_hz'))
        self.recenter = self._new_recenter()
        self.state, self.fault = 'DISABLED', ''
        self.qdot = np.zeros(5)
        self.target = None
        self.ref = None
        self.samples = {}
        self.joint_history = deque(maxlen=128)
        self.wrench_history = deque(maxlen=64)
        self.grasp, self.grasp_seen = False, -np.inf
        self.last_tick = None
        self.last_wrench_stamp = None
        self.last_wall_clock = time.monotonic()
        self.last_sim = None
        self.enabled, self.owned = False, False
        self.controller_checked = -np.inf
        self.controllers_ok = False
        self.pending = None
        self.base_mismatch_s = 0.
        self.rtf_anchor = (time.monotonic(), self.get_clock().now().nanoseconds*1e-9)
        self.rtf_estimate = None
        self.sign = float(self.p('sensor_sign'))
        self.certificate = None
        if self.p('calibration_file'):
            self.certificate = json.loads(Path(self.p('calibration_file')).read_text())
            c = self.certificate
            if c['urdf_sha256'] != hashlib.sha256(self.p('robot_description').encode()).hexdigest():
                raise ValueError('calibration URDF fingerprint mismatch')
            if c['robot'] != self.robot or c['sensor_link'] != self.arm.sensor:
                raise ValueError('calibration model/frame mismatch')
            if c['header_frame'] != self.p('sensor_header_frame') or c['sign'] != self.sign:
                raise ValueError('calibration header/sign mismatch')
            if not c.get('phase0_passed') or not c.get('time_alignment_passed'):
                raise ValueError('Phase 0/1 calibration gates incomplete')
            needed = {name for name, enabled in zip(['fx', 'fy', 'fz', 'tz'], self.adm.axes) if enabled}
            if not needed.issubset(set(c['verified_axes'])):
                raise ValueError('selected axis has no physical calibration evidence')
            passed = set(c.get('phase2_axes_passed', []))
            order = ['fx','fy','fz','tz']
            for axis in needed:
                if not set(order[:order.index(axis)]).issubset(passed):
                    raise ValueError('previous single-axis admittance gates incomplete')
            for evidence in c['evidence_files']:
                if not Path(evidence).is_file():
                    raise ValueError('missing calibration evidence file')
            if not c['evidence_files']:
                raise ValueError('physical calibration evidence required')
        self.arm_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 1)
        # Use the existing Twist relay; sending directly would race its zero commands.
        self.base_pub = self.create_publisher(Twist, 'rover_twist', 1)
        self.status_pub = self.create_publisher(String, 'follower/status', 10)
        self.create_subscription(WrenchStamped, 'ft_sensor',
                                 lambda m: self._receive('wrench', m), qos_profile_sensor_data)
        self.create_subscription(JointState, 'joint_states',
                                 lambda m: self._receive('joints', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'mecanum_drive_controller/odometry',
                                 lambda m: self._receive('odom', m), qos_profile_sensor_data)
        self.create_subscription(String, 'grasp/state', self._grasp, 10)
        self.create_service(Trigger, 'follower/calibrate', self._calibrate)
        self.create_service(SetBool, 'follower/enable', self._enable)
        self.create_service(Trigger, 'follower/reset_fault', self._reset)
        self.cm = self.create_client(ListControllers, 'controller_manager/list_controllers')
        self.create_timer(.5, self._poll_controllers)
        rate = float(self.p('control_rate_hz'))
        if not 1 <= rate <= 200:
            raise ValueError('control rate must be measured before exceeding 200 Hz')
        self.create_timer(1./rate, self._tick)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self._wall_watchdog, clock=self.wall_clock)
        self.log = None
        if self.p('log_file'):
            path = Path(self.p('log_file')).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.log = path.open('a', buffering=1)
        self._audit_subscriptions()

    def _new_admittance(self):
        return Admittance4D(self.p('axes_enabled'),self.p('admittance.mass'),
                            self.p('admittance.damping'),self.p('admittance.stiffness'),
                            self.p('admittance.max_velocity'),self.p('admittance.max_offset'))

    def _new_recenter(self):
        return Recenter(self.p('recenter_start_m'),self.p('recenter_stop_m'))

    def _audit_subscriptions(self):
        actual = {name for name, _ in self.get_subscriber_names_and_types_by_node(
            self.get_name(), self.get_namespace())}
        # Discovery can lag; resolved names are checked directly as well.
        resolved = {self.resolve_topic_name(x) for x in [
            'ft_sensor', 'joint_states', 'mecanum_drive_controller/odometry', 'grasp/state']}
        if not (actual | resolved).issubset(allowed_subscriptions(self.robot)):
            raise ValueError('subscription/remap outside local whitelist')
        if (self.arm_pub.topic_name != self.prefix+'/arm_controller/joint_trajectory' or
                self.base_pub.topic_name != self.prefix+'/rover_twist'):
            raise ValueError('command remap outside own robot namespace')

    def _receive(self, key, msg):
        self.samples[key] = msg
        if key == 'wrench':
            if not self.wrench_history or stamp_s(msg) > stamp_s(self.wrench_history[-1]):
                self.wrench_history.append(msg)
        elif key == 'joints':
            try:
                mapping = dict(zip(msg.name,msg.position))
                q = finite([mapping[n] for n in self.arm.names], (5,))
                stamp = stamp_s(msg)
                if not self.joint_history or stamp > self.joint_history[-1][0]:
                    self.joint_history.append((stamp,q))
            except (ValueError,KeyError):
                if self.state != 'DISABLED':
                    self._trip('invalid local joint sample')

    def _grasp(self, msg):
        self.grasp = msg.data == 'attached'
        self.grasp_seen = self.get_clock().now().nanoseconds * 1e-9
        if self.owned and not self.grasp:
            self._trip('grasp loss')

    def _poll_controllers(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                controllers = self.pending.result().controller
                active = {c.name: c.type for c in controllers if c.state == 'active'}
                self.controllers_ok = (
                    active.get('arm_controller') == 'joint_trajectory_controller/JointTrajectoryController'
                    and active.get('mecanum_drive_controller') == 'mecanum_drive_controller/MecanumDriveController'
                    and 'forward_position_controller' not in active)
                self.controller_checked = self.get_clock().now().nanoseconds * 1e-9
            except Exception:
                self.controllers_ok = False
            self.pending = None
        if self.cm.service_is_ready():
            self.pending = self.cm.call_async(ListControllers.Request())

    def _calibrate(self, request, response):
        if self.owned or self.enabled or self.state == 'FAULT':
            response.success, response.message = False, 'disable/reset first'
        elif self.certificate is None or self.sign not in (-1., 1.):
            response.success, response.message = False, 'measured Phase 0/1 calibration certificate required'
        elif not self.controllers_ok or self.get_clock().now().nanoseconds*1e-9-self.controller_checked>2.:
            response.success, response.message = False, 'audited controllers not ready; retry calibration after discovery'
        else:
            self.observer = WrenchObserver(int(self.p('bias_samples')), self.p('cutoff_hz'))
            self.adm = self._new_admittance()
            self.recenter = self._new_recenter()
            self.qdot[:] = 0.
            self.state = 'CALIBRATING'
            self.last_wrench_stamp = None
            response.success, response.message = True, 'collecting stationary grasp samples'
        return response

    def _enable(self, request, response):
        if not request.data:
            self.enabled = False
            if self.owned:
                self.state = 'STOPPING'
            elif self.state != 'FAULT':
                self.state = 'DISABLED'
            response.success, response.message = True, 'stopping and holding gripper'
        elif self.state != 'READY':
            response.success, response.message = False, 'READY required after measured calibration'
        elif self.p('mode') not in ('A', 'B', 'C'):
            response.success, response.message = False, 'mode must be A, B or C'
        elif self.p('mode') == 'C' and not (self.certificate or {}).get('base_axes_verified'):
            response.success, response.message = False, 'Phase 3 physical base-axis calibration required'
        elif self.p('mode') == 'C' and not {'fx','fy','fz','tz'}.issubset(
                set((self.certificate or {}).get('phase2_axes_passed',[]))):
            response.success, response.message = False, 'Phase 2 physical four-axis gates required before whole-body integration'
        elif self.p('mode') == 'C' and not self.p('recenter_enabled'):
            response.success, response.message = False, 'mode C requires recenter_enabled'
        else:
            # Services cannot silently take ownership from approach/home/another controller.
            arm_writers = self.get_publishers_info_by_topic(self.arm_pub.topic_name)
            base_writers = self.get_publishers_info_by_topic(self.base_pub.topic_name)
            if len(arm_writers)!=1 or len(base_writers)!=1 or any(
                   p.node_name != self.get_name() or p.node_namespace != self.get_namespace()
                   for p in arm_writers+base_writers):
                response.success, response.message = False, 'command publisher conflict; finish startup handoff first'
            elif not self.controllers_ok:
                response.success, response.message = False, 'audited controllers must be active'
            else:
                self.enabled, self.owned = True, True
                self.state = 'FOLLOWING'
                response.success, response.message = True, 'local follower owns arm/base commands'
        return response

    def _reset(self, request, response):
        if self.owned or self.enabled:
            response.success, response.message = False, 'disable first'
        else:
            self.state, self.fault = 'DISABLED', ''
            self.target, self.last_tick = None, None
            self.joint_history.clear()
            self.wrench_history.clear()
            self.last_wrench_stamp = None
            self.qdot[:] = 0
            self.adm = self._new_admittance()
            self.recenter = self._new_recenter()
            response.success, response.message = True, 'fault reset; recalibration required'
        return response

    def _trip(self, reason):
        self.state, self.fault, self.enabled = 'FAULT', reason, False
        if self.owned:
            self.base_pub.publish(Twist())
            if self.target is not None:
                # Hold a fresh measured posture instead of continuing toward
                # the position target that may have caused a clamp overload.
                joints=self.samples.get('joints')
                if joints is not None:
                    try:
                        now=self.get_clock().now().nanoseconds*1e-9
                        if 0<=now-stamp_s(joints)<=self.p('max_age_s'):
                            mapping=dict(zip(joints.name,joints.position))
                            measured=finite([mapping[n] for n in self.arm.names],(5,))
                            if np.all(measured>=self.arm.lower) and np.all(measured<=self.arm.upper):
                                self.target=measured
                    except (ValueError,KeyError):
                        pass
                self._arm_command(self.target, np.zeros(5), .03)
            self.owned = False
        self._record({'state': self.state, 'fault': reason})

    def _wall_watchdog(self):
        sim = self.get_clock().now().nanoseconds
        now = time.monotonic()
        if sim != self.last_sim:
            self.last_sim, self.last_wall_clock = sim, now
        # On pause, do not advance integrations or publish commands. After a long
        # pause require reset; the zero/hold is sent once after time resumes.
        elif self.owned and now-self.last_wall_clock > .5:
            self.pause_fault = True

    def _arm_command(self, q, dq, horizon):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.arm.names)
        point = JointTrajectoryPoint(positions=list(q), velocities=list(dq))
        point.time_from_start.sec = int(horizon)
        point.time_from_start.nanosec = int((horizon-int(horizon))*1e9)
        msg.points = [point]
        self.arm_pub.publish(msg)

    def _record(self, values):
        values.setdefault('sim_time', self.get_clock().now().nanoseconds*1e-9)
        if self.log:
            self.log.write(json.dumps(values, allow_nan=False)+'\n')

    def _tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_tick is None:
            self.last_tick = now
            return
        dt, self.last_tick = now-self.last_tick, now
        if dt == 0:
            return
        self.status_pub.publish(String(data=self.state))
        if self.state == 'DISABLED':
            self.pause_fault = False
            if dt < 0:
                self.joint_history.clear()
                self.wrench_history.clear()
            return
        if dt < 0 or dt > .05 or getattr(self, 'pause_fault', False):
            self.pause_fault = False
            self._trip('clock jump, prolonged pause or excessive control interval')
            return
        if self.state in ('DISABLED', 'FAULT'):
            return
        if self.state == 'STOPPING':
            self.recenter.command += np.clip(-self.recenter.command, -dt*.04, dt*.04)
            self._publish(self.target, np.zeros(5), self.recenter.command, dt)
            if np.linalg.norm(self.recenter.command) < 1e-9:
                self.state, self.owned = 'DISABLED', False
            return
        try:
            wall_now = time.monotonic()
            if wall_now-self.rtf_anchor[0] >= 1.:
                self.rtf_estimate = (now-self.rtf_anchor[1])/(wall_now-self.rtf_anchor[0])
                self.rtf_anchor = (wall_now,now)
            if set(self.samples) != {'wrench', 'joints', 'odom'}:
                raise ValueError('missing local state')
            w, joints, odom = [self.samples[k] for k in ('wrench', 'joints', 'odom')]
            if not self.joint_history:
                raise ValueError('missing joint history')
            candidates = [m for m in self.wrench_history if
                          self.joint_history[0][0]-1e-9 <= stamp_s(m) <=
                          self.joint_history[-1][0]+1e-9]
            if not candidates:
                raise ValueError('no joint bracket for wrench time')
            w = candidates[-1]
            odom_twist = finite([odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                                 odom.twist.twist.angular.z], (3,))
            ages = check_timing(now, [stamp_s(m) for m in (w,joints,odom)],
                                self.p('max_age_s'), self.p('max_skew_s'))
            # The scene adapter periodically confirms the actual constraint.
            # Missing heartbeat or a detached report stops local control.
            if not self.grasp:
                raise ValueError('own grasp not attached')
            if now-self.grasp_seen > self.p('grasp_timeout_s'):
                raise ValueError('own grasp heartbeat missing or stale')
            if not self.controllers_ok or now-self.controller_checked > 2.:
                raise ValueError('controller state missing or stale')
            mapping = dict(zip(joints.name, joints.position))
            vmap = dict(zip(joints.name, joints.velocity))
            q = finite([mapping[n] for n in self.arm.names], (5,))
            dq = finite([vmap[n] for n in self.arm.names], (5,))
            if np.any(np.abs(dq) > self.arm.velocity*1.05):
                raise ValueError('joint velocity exceeds URDF limit')
            if w.header.frame_id != self.p('sensor_header_frame'):
                raise ValueError('unexpected sensor header frame')
            raw = finite([w.wrench.force.x,w.wrench.force.y,w.wrench.force.z,
                          w.wrench.torque.x,w.wrench.torque.y,w.wrench.torque.z], (6,))
            if np.any(np.abs(raw) > finite(self.p('max_abs_wrench'), (6,))):
                raise ValueError('raw six-axis wrench overload')
            tg, ts, jac = self.arm.evaluate(q)
            q_at_wrench = interpolate_joints(self.joint_history,stamp_s(w), self.p('max_skew_s'))
            tg_at_wrench, ts_at_wrench, full_jac_at_wrench = self.arm.evaluate(
                q_at_wrench,full_jacobian=True)
            sensor_axis=ts_at_wrench[:3,:3].T@full_jac_at_wrench[3:,self.arm.sensor_joint_index]
            sensor_joint_torque=float(sensor_axis@raw[3:])
            if abs(sensor_joint_torque)>self.arm.effort[self.arm.sensor_joint_index]:
                raise ValueError('instrumented joint transmitted torque exceeds URDF effort')
            static_load=self.arm.quasistatic_joint_load(q_at_wrench,raw,self.sign,self.p('gravity_B'))
            if np.any(np.abs(static_load)>self.arm.effort):
                index=int(np.argmax(np.abs(static_load)/self.arm.effort))
                self._record({'state':self.state,'guard_event':'quasistatic load exceeds URDF effort',
                              'quasistatic_joint_load_Nm':static_load.tolist(),
                              'raw':raw.tolist(),'joint_q_at_wrench_stamp':q_at_wrench.tolist()})
                raise ValueError(f'quasistatic model load exceeds URDF effort: {self.arm.names[index]}')
            transformed = transform_wrench(raw, ts_at_wrench, tg_at_wrench[:3,3], self.sign)
            stamp = stamp_s(w)
            if stamp != self.last_wrench_stamp:
                sensor_dt = dt if self.last_wrench_stamp is None else stamp-self.last_wrench_stamp
                if self.state == 'CALIBRATING':
                    twist = odom.twist.twist
                    if (np.max(np.abs(dq)) > .01 or
                        np.linalg.norm([twist.linear.x,twist.linear.y,twist.angular.z]) > .005):
                        raise ValueError('calibration requires stationary arm and base')
                self.error = self.observer.update(transformed, sensor_dt, self.state == 'CALIBRATING')
                self.last_wrench_stamp = stamp
            if self.state == 'CALIBRATING':
                if self.observer.count >= self.observer.samples:
                    self.state, self.ref, self.target = 'READY', tg.copy(), q.copy()
                self._record({'state':self.state, 'raw':raw.tolist(),
                              'quasistatic_joint_load_Nm':static_load.tolist(),
                              'wrench_G_B':transformed.tolist(), 'age':ages.tolist()})
                return
            if self.state == 'READY':
                return
            if np.max(np.abs(self.target-q)) > self.p('max_tracking_error_rad'):
                raise ValueError('arm command tracking discontinuity')
            desired = np.zeros(4)
            if self.p('mode') != 'A':
                err = np.sign(self.error)*np.maximum(np.abs(self.error)-self.p('deadband'), 0)
                desired = self.adm.step(np.clip(err, -np.array(self.p('soft_limit')),
                                               self.p('soft_limit')), dt)
            relative = tg[:3,:3] @ self.ref[:3,:3].T
            yaw_error = np.arctan2(relative[1,0], relative[0,0])
            base = self.recenter.step(tg[:3,3]-self.ref[:3,3], yaw_error, dt,
                                     self.p('mode') == 'C' and self.p('recenter_enabled'))
            mismatch = base-odom_twist
            self.base_mismatch_s = self.base_mismatch_s+dt if (
                np.linalg.norm(mismatch[:2]) > .03 or abs(mismatch[2]) > .08) else 0.
            if self.base_mismatch_s > .5:
                raise ValueError('base command/encoder odometry mismatch')
            if self.p('mode') == 'A':
                q_command, qdot = self.target, np.zeros(5)
                residual, sigma = np.zeros(4), np.linalg.svd(jac, compute_uv=False)
            else:
                result = allocate(self.arm, q, self.qdot, jac, desired, base, dt, tg[:3,3])
                qdot, residual, sigma = result.qdot, result.residual, result.singular_values
                q_command = self.target + dt*qdot
                if np.any(q_command < self.arm.lower+.05) or np.any(q_command > self.arm.upper-.05):
                    raise ValueError('integrated target at joint margin')
                if (np.max(np.abs(residual[:3])) > self.p('residual_limit') or
                        abs(residual[3]) > .2):
                    raise ValueError('task allocation residual exceeds safe limit')
            self.target, self.qdot = q_command.copy(), qdot.copy()
            self.state = 'RECENTERING' if self.recenter.active else 'FOLLOWING'
            self._publish(q_command, qdot, base, dt)
            self._record({'state':self.state,'dt':dt,'raw':raw.tolist(),
                          'wrench_G_B':transformed.tolist(), 'hold':self.observer.mean.tolist(),
                          'error':self.error.tolist(),'offset':self.adm.x.tolist(),
                          'velocity':desired.tolist(),'q':q.tolist(),'dq':dq.tolist(),
                          'q_des':q_command.tolist(),'base_cmd':base.tolist(),
                          'uncommanded_angular_velocity_xy':(
                              self.arm.evaluate(q,full_jacobian=True)[2][3:5]@qdot).tolist(),
                          'sensor_joint_transmitted_torque_Nm':sensor_joint_torque,
                          'quasistatic_joint_load_Nm':static_load.tolist(),
                          'joint_q_at_wrench_stamp':q_at_wrench.tolist(),
                          'wrench_stamp':stamp_s(w),'joint_stamp':stamp_s(joints),
                          'rtf_estimate':self.rtf_estimate,
                          'base_odom_twist':[odom.twist.twist.linear.x,odom.twist.twist.linear.y,
                                             odom.twist.twist.angular.z],
                          'residual':residual.tolist(),'sigma':sigma.tolist(),'age':ages.tolist(),
                          'joint_margin':float(np.min(np.minimum(q-self.arm.lower,self.arm.upper-q)))})
        except (ValueError, KeyError, IndexError) as exc:
            self._trip(str(exc))

    def _publish(self, q, dq, base, dt):
        self._arm_command(q, dq, max(dt, .01))
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, base)
        self.base_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._trip('shutdown')
        if node.log:
            node.log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
