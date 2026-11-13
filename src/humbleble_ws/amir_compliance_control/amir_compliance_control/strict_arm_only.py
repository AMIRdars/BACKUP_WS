"""Opt-in AF controller; existing CP5/ARM_BASE/WHOLE_BODY are unchanged."""
import json
import time
import hashlib
from functools import partial
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .arm_admittance import ArmAdmittance
from .arm_only_dls import xy_velocity_dls, nullspace_pose_velocity
from .admittance import rotation_error
from .transport import TransportEndpoint
from .wrench import finite
from .arm_servo_response import validate_response
from .arm_trajectory_bounds import streaming_knots,trajectory_times


class StrictArmOnly(ArmAdmittance):
    def __init__(self, **kwargs):
        super().__init__(node_name='follower_arm_only', mechanics_defaults={
            'virtual_mass': [2., 2.], 'virtual_damping': [34.6, 34.6],
            'virtual_stiffness': [150., 150.], 'displacement_limit_m': [.012, .012],
            'velocity_limit_m_s': [.01, .01]}, **kwargs)
        self.role = 'ARM_ONLY'
        for key, value in {'control_mode': 'ARM_ONLY', 'rotation_tracking_enabled': False,
                'enable_base_assist': False, 'wrench_source_mode': 'sensor',
                'force_deadband_N': 0.,
                'joint_request_acceleration_rad_s2': 0.,
                'simulation_micro_test': False, 'allow_ungrasped_test': False,
                'simulation_fault_injection_enabled': False,
                'tracking_gain_s_inv': 4., 'dls_damping_m': .001,
                'singular_stop_m': .001, 'joint_margin_rad': .01,
                'arm_displacement_stop_m': .01, 'height_error_stop_m': .002,
                'rotation_error_stop_rad': .01, 'base_drift_stop_m': .001,
                'base_yaw_drift_stop_rad': .002, 'base_speed_stop': .001,
                'arm_command_horizon_s': .04,
                'arm_trajectory_start_delay_s': 0.,
                'arm_command_reference': 'integrated',
                'control_on_wrench': False,
                'arm_servo_response_file': '',
                'arm_nullspace_pose_hold': False, 'nullspace_pose_gain_s_inv': 4.,
                'nullspace_angular_weight_m': .1,
                # No guessed force ratings: must be supplied explicitly by the
                # simulation experiment from its measured excitation envelope.
                'force_increment_stop_N': 0., 'torque_increment_stop_Nm': 0.}.items():
            self.declare_parameter(key, value)
        if self.p('control_mode') != 'ARM_ONLY' or self.p('rotation_tracking_enabled'):
            raise ValueError('AF launch permits ARM_ONLY with rotation disabled only')
        if self.p('enable_base_assist') is not False or self.p('wrench_source_mode') != 'sensor':
            self.destroy_node()
            raise ValueError('Phase 1 requires enable_base_assist=false and wrench_source_mode=sensor')
        if not self.p('simulation_micro_test'):
            raise ValueError('uncalibrated AF controller is simulation micro-test only')
        if not np.isfinite(self.p('force_deadband_N')) or self.p('force_deadband_N') < 0:
            raise ValueError('finite nonnegative force deadband required')
        if not np.isfinite(self.p('joint_request_acceleration_rad_s2')) or self.p('joint_request_acceleration_rad_s2') < 0:
            raise ValueError('finite nonnegative joint acceleration policy required')
        if not isinstance(self.p('arm_nullspace_pose_hold'), bool):
            raise ValueError('boolean arm_nullspace_pose_hold required')
        if self.p('arm_command_reference') not in ('integrated','servo_trajectory'):
            raise ValueError('explicit supported arm command reference required')
        if not isinstance(self.p('control_on_wrench'),bool):
            raise ValueError('boolean control_on_wrench required')
        for key in ('tracking_gain_s_inv', 'dls_damping_m', 'singular_stop_m', 'joint_margin_rad',
                    'arm_displacement_stop_m', 'height_error_stop_m', 'rotation_error_stop_rad',
                    'base_drift_stop_m', 'base_yaw_drift_stop_rad', 'base_speed_stop',
                    'arm_command_horizon_s', 'force_increment_stop_N', 'torque_increment_stop_Nm',
                    'nullspace_pose_gain_s_inv', 'nullspace_angular_weight_m'):
            if not np.isfinite(self.p(key)) or self.p(key) <= 0:
                raise ValueError('explicit positive AF experiment policy required: '+key)
        if not .02 <= self.p('arm_command_horizon_s') <= .1:
            raise ValueError('AF JTC horizon must be 20..100 ms')
        if not np.isfinite(self.p('arm_trajectory_start_delay_s')) or not 0<=self.p('arm_trajectory_start_delay_s')<=.1:
            raise ValueError('bounded AF trajectory start delay required')
        self.servo_compensation = np.zeros(5)
        self.servo_period = None
        if self.p('arm_command_reference')=='servo_trajectory' and not self.p('arm_servo_response_file'):
            raise ValueError('servo_trajectory requires measured actuator response')
        if self.p('arm_servo_response_file'):
            if self.p('arm_command_reference')!='servo_trajectory':
                raise ValueError('servo response compensation requires servo_trajectory')
            cert=json.loads(Path(self.p('arm_servo_response_file')).read_text())
            urdf=Path(self.p('urdf_file')).resolve()
            self.servo_compensation=validate_response(cert,urdf.read_text(),urdf.parent)
            self.servo_period=1/float(cert['controller_update_rate_hz'])
            if hashlib.sha256(Path('/opt/ros/humble/lib/libgz_hardware_plugins.so').read_bytes()).hexdigest()!=cert['plugin_library_sha256']:
                raise ValueError('servo response plugin library changed')
        self.measurements = {}
        self.rotation_home = self.base_home = None
        self.last_wrench = None
        self.qdot_last = np.zeros(5)
        self.qdot_slew = np.zeros(5)
        self.zero_sent = False
        self.fault_debug = {}
        self.test_fault_pending = None
        self.test_fault_services = []
        if self.p('simulation_fault_injection_enabled'):
            # Explicit AF-3 test hooks only. No alternate sensor publisher,
            # unsafe joint excursion, geometry change or ordinary data input.
            for fault in ('joint_margin','singularity','base_drift','dt'):
                self.test_fault_services.append(self.create_service(Trigger,
                    'arm_only/test/'+fault, partial(self.inject_test_fault, fault=fault)))
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='AF experiment policies are immutable; restart disarmed with reviewed settings'))

    def filtered(self, msg):
        super().filtered(msg)
        w = msg.wrench
        if self.force is not None:
            self.last_wrench = finite([w.force.x, w.force.y, w.force.z,
                                       w.torque.x, w.torque.y, w.torque.z], (6,))
            if getattr(self, 'core', None) is not None:
                if (np.linalg.norm(self.last_wrench[:3]) >= self.p('force_increment_stop_N')
                        or np.linalg.norm(self.last_wrench[3:]) >= self.p('torque_increment_stop_Nm')):
                    self.trip('ARM_ONLY incremental wrench experiment envelope')
            if getattr(self,'core',None) is not None and self.p('control_on_wrench'):
                # Each source stamp advances the arm at most once. The existing
                # steady timer continues zero-base output and dropout watchdogs.
                self.tick()

    def base_request(self, command):
        return np.zeros(3)

    def control_force(self):
        value = super().control_force()
        # Continuous dead zone, measured/tared base-frame force only.
        return np.sign(value)*np.maximum(np.abs(value)-self.p('force_deadband_N'),0.)

    def inject_test_fault(self, request, response, *, fault):
        response.success = self.core is not None and not self.latched and self.test_fault_pending is None
        if response.success:self.test_fault_pending = fault
        response.message = 'synthetic guard input queued; physical feedback unchanged' if response.success else 'active simulation test required'
        return response

    def test_guard_inputs(self, q, drift):
        fault = self.test_fault_pending
        if fault is None:return drift
        self.test_fault_pending = None
        self.fault_debug.update(simulation_fault_injection=fault,
            injection_scope='synthetic validation input only; actual measured state unchanged')
        if fault == 'joint_margin':
            test_q=q.copy();test_q[0]=self.model.lower[0]+self.p('joint_margin_rad')*.5
            self.dls(test_q,np.zeros(2))
        elif fault == 'singularity':
            test_model=SimpleNamespace(lower=self.model.lower,upper=self.model.upper,velocity=self.model.velocity,
                                       jacobian=lambda _:np.zeros((6,5)))
            xy_velocity_dls(test_model,q,np.zeros(2),damping=self.p('dls_damping_m'),
                singular_stop=self.p('singular_stop_m'),margin=self.p('joint_margin_rad'),
                speed=self.p('joint_request_speed_rad_s'))
        elif fault == 'dt':
            self.core.step(self.control_force(),.051)  # actual integrator rejects before changing state
        elif fault == 'base_drift':
            drift=drift.copy();drift[0]=1.01*self.p('base_drift_stop_m')
        return drift

    def local_health(self):
        good, reason = super().local_health()
        if good and self.p('actuator_ack_required'):
            if (self.gate.get('base_zero_only') is not True
                    or self.actuator.get('base_zero_only') is not True
                    or self.actuator.get('request_writer_name') != self.get_name()):
                return False, 'ARM_ONLY zero lock and command ownership audit required'
        return good, reason

    def publish_command(self, publisher, message):
        if publisher is self.base_pub:
            # Guard the final request even if an inherited path changes later.
            message.twist.linear.x = message.twist.linear.y = message.twist.linear.z = 0.
            message.twist.angular.x = message.twist.angular.y = message.twist.angular.z = 0.
            self.zero_sent = True
            publisher.publish(message)
        else:
            super().publish_command(publisher, message)

    def measured_q(self):
        mapping = dict(zip(self.joints.name, self.joints.position))
        return finite([mapping[n] for n in self.model.names], (5,))

    def base_pose(self):
        p = self.encoder.pose.pose
        return np.array([p.position.x, p.position.y,
            np.arctan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                       1-2*(p.orientation.y**2+p.orientation.z**2))])

    def dls(self, q, v):
        return xy_velocity_dls(self.model, q, v, damping=self.p('dls_damping_m'),
            singular_stop=self.p('singular_stop_m'), margin=self.p('joint_margin_rad'),
            speed=self.p('joint_request_speed_rad_s'))

    def enable(self, request, response):
        response.success = False
        try:
            if not (self.state == 'READY' and not self.latched and self.permit and not self.emergency
                    and self.local_health()[0] and self.wrench_ready() and self.core is None):
                raise ValueError('fresh tared local READY state required')
            if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                raise ValueError('confirmed stable local grasp required')
            if (self.last_wrench is None or np.linalg.norm(self.last_wrench[:3]) >= self.p('force_increment_stop_N')
                    or np.linalg.norm(self.last_wrench[3:]) >= self.p('torque_increment_stop_Nm')):
                raise ValueError('initial wrench outside AF experiment envelope')
            q = self.measured_q()
            self.dls(q, np.zeros(2))
            self.reference, sensor = self.model.frames(q)
            self.rotation_home = sensor[:3, :3].copy()
            self.base_home = self.base_pose()
            self.q_request, self.qdot_last = q.copy(), np.zeros(5)
            self.qdot_slew[:] = 0.
            self.q_nominal = q.copy()
            self.q_start_request = q.copy()
            self.core, self.last_step_stamp = self.make_core(), self.force_stamp
            self.state, self.reason = 'TRANSPORT', ''
            response.success, response.message = True, 'ARM_ONLY home latched from measured FK; base always zero'
        except (ValueError, KeyError) as exc:
            response.message = str(exc)
        return response

    def trip(self, reason):
        if not getattr(self, 'latched', False):
            self.fault_debug = {'fault_observed_sim_time_s': self.sim,
                'fault_observed_wall_time_s': time.monotonic(), 'fault_observed_reason': str(reason),
                **{k:v for k,v in getattr(self,'fault_debug',{}).items()
                   if k in ('simulation_fault_injection','injection_scope')}}
            if getattr(self, 'last_wrench', None) is not None:
                self.fault_debug['wrench_at_fault'] = self.last_wrench.tolist()
            if getattr(self, 'core', None) is not None:
                self.fault_debug.update(delta_p_at_fault_m=self.core.x.tolist(),
                                        admittance_limit_at_fault_m=self.core.limit.tolist())
        super().trip(reason)
        if hasattr(self, 'qdot_last'):
            self.qdot_last[:] = 0.
        if hasattr(self, 'measurements'):
            self.measurements['qdot_cmd_rad_s'] = [0.]*5

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.fault_debug = {}
            self.test_fault_pending = None
        return response

    def tick(self):
        self.zero_sent = False
        TransportEndpoint.tick(self)
        # Publish in INIT/disarmed/hold too. No second base writer or mux path.
        if not self.zero_sent:
            msg = TwistStamped()
            msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'follower/base_footprint'
            self.publish_command(self.base_pub, msg)
        if not hasattr(self, 'measurements'):
            return
        if self.core is not None:
            try:
                if not self.wrench_ready():
                    raise ValueError('ARM_ONLY filtered wrench watchdog')
                if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                    raise ValueError('ARM_ONLY local grasp missing')
                q = self.measured_q()
                actual, sensor = self.model.frames(q)
                arm = actual-self.reference
                pose_error = rotation_error(sensor[:3, :3], self.rotation_home)
                drift = self.base_pose()-self.base_home
                drift[2] = np.arctan2(np.sin(drift[2]), np.cos(drift[2]))
                guard_drift = self.test_guard_inputs(q, drift)
                twist = self.encoder.twist.twist
                base_speed = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))
                if (np.linalg.norm(guard_drift[:2]) >= self.p('base_drift_stop_m')
                        or abs(guard_drift[2]) >= self.p('base_yaw_drift_stop_rad')
                        or base_speed >= self.p('base_speed_stop')):
                    raise ValueError('ARM_ONLY measured base drift/speed')
                if np.linalg.norm(arm[:2]) >= self.p('arm_displacement_stop_m'):
                    raise ValueError('ARM_ONLY measured arm displacement')
                if abs(arm[2]) >= self.p('height_error_stop_m') or np.linalg.norm(pose_error) >= self.p('rotation_error_stop_rad'):
                    raise ValueError('ARM_ONLY measured height/orientation envelope')
                dt = self.force_stamp-self.last_step_stamp
                if dt > 0:
                    if np.any(np.abs(self.core.x) >= self.core.limit-1e-12):
                        raise ValueError('ARM_ONLY admittance displacement saturation')
                    displacement = self.core.step(self.control_force(), dt)
                    if np.any(np.abs(displacement) >= self.core.limit-1e-12):
                        raise ValueError('ARM_ONLY admittance displacement saturation')
                    target = self.reference[:2]+displacement
                    track = target-actual[:2]
                    velocity = np.clip(self.core.v+self.p('tracking_gain_s_inv')*track,
                                       -self.core.vmax, self.core.vmax)
                    qdot, sigma, margin, damping = self.dls(q, velocity)
                    secondary_debug = {}
                    primary_qdot = qdot.copy()
                    if self.p('arm_nullspace_pose_hold'):
                        desired_secondary = -self.p('nullspace_pose_gain_s_inv')*np.r_[arm[2], pose_error]
                        qdot, secondary_debug = nullspace_pose_velocity(self.model, q, qdot, desired_secondary,
                            damping=damping, angular_weight=self.p('nullspace_angular_weight_m'),
                            speed=self.p('joint_request_speed_rad_s'))
                    acceleration=self.p('joint_request_acceleration_rad_s2')
                    if acceleration:
                        qdot=np.clip(qdot,self.qdot_slew-acceleration*dt,self.qdot_slew+acceleration*dt)
                    self.qdot_slew=qdot.copy()
                    # Integrate the velocity request, while the measured FK
                    # feedback closes tracking. Bound both per-cycle motion
                    # and lookahead relative to the actual JTC state.
                    horizon = self.p('arm_command_horizon_s')
                    limits = np.minimum(self.model.velocity, self.p('joint_request_speed_rad_s'))
                    previous = self.q_request.copy()
                    command_lower = np.maximum(previous-limits*dt, q-limits*horizon)
                    command_upper = np.minimum(previous+limits*dt, q+limits*horizon)
                    if np.any(command_lower > command_upper):
                        raise ValueError('ARM_ONLY measured/requested joint separation')
                    # Keep the established integral as the default. The
                    # measured-servo experiment supplies explicit current and
                    # future position knots through that same position API.
                    lead = qdot*(horizon+self.servo_compensation)
                    mode = self.p('arm_command_reference')
                    if mode == 'servo_trajectory':
                        # Keep the velocity integral independent of measured
                        # servo lag. Predict the interpolation endpoint and
                        # compensate only empirically verified servo response.
                        # Rebase after limiting to prevent reference windup.
                        proposed = self.q_nominal+qdot*dt+lead
                    else:
                        proposed = previous+qdot*dt
                    if mode == 'servo_trajectory':
                        # The first sample of a replacement one-point JTC
                        # trajectory starts at the previous desired state.
                        # Supply an explicit current-time knot so streaming
                        # never restarts that artificial initial segment.
                        start_lower = np.maximum(self.q_start_request-limits*dt,q-limits*horizon)
                        start_upper = np.minimum(self.q_start_request+limits*dt,q+limits*horizon)
                        start,candidate,stream_velocity=streaming_knots(proposed,qdot,horizon,limits,
                            start_lower,start_upper,command_lower,command_upper)
                        secondary_debug['stream_velocity_rad_s']=stream_velocity.tolist()
                        secondary_debug['stream_velocity_limited']=bool(np.any(abs(stream_velocity-qdot)>1e-12))
                    else:
                        candidate = np.clip(proposed, command_lower, command_upper)
                    applied_qdot = (candidate-previous)/dt
                    lower, upper = self.model.lower+self.p('joint_margin_rad'), self.model.upper-self.p('joint_margin_rad')
                    if np.any(candidate <= lower) or np.any(candidate >= upper):
                        raise ValueError('ARM_ONLY predicted joint margin')
                    self.q_request = candidate
                    if mode == 'servo_trajectory':
                        self.q_nominal = candidate-lead
                    msg = JointTrajectory(joint_names=list(self.model.names))
                    msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'follower/base_footprint'
                    point = JointTrajectoryPoint(positions=candidate.tolist())
                    point.time_from_start = Duration(sec=int(horizon), nanosec=int(round(horizon%1*1e9)))
                    msg.points = [point]
                    if mode == 'servo_trajectory':
                        if np.any(start <= lower) or np.any(start >= upper):
                            raise ValueError('ARM_ONLY streaming start joint margin')
                        period=self.servo_period
                        delay=self.p('arm_trajectory_start_delay_s')
                        header,first_time,end_time=trajectory_times(self.sim,period,horizon,delay)
                        msg.header.stamp=Time(seconds=header).to_msg()
                        # Positive duration is required by the unchanged gate;
                        # its absolute knot time is the current local clock.
                        first=JointTrajectoryPoint(positions=start.tolist())
                        first.time_from_start=Duration(sec=int(first_time),nanosec=int(round(first_time%1*1e9)))
                        point.time_from_start=Duration(sec=int(end_time),nanosec=int(round(end_time%1*1e9)))
                        msg.points=[first,point]
                        self.q_start_request=start
                    self.publish_command(self.arm_pub, msg)
                    self.last_step_stamp, self.qdot_last = self.force_stamp, applied_qdot
                    self.measurements = {**secondary_debug,
                        'arm_nullspace_pose_hold': self.p('arm_nullspace_pose_hold'),
                        'arm_command_reference': self.p('arm_command_reference'),
                        'arm_command_horizon_s': horizon,
                        'arm_trajectory_start_delay_s':self.p('arm_trajectory_start_delay_s'),
                        'control_on_wrench': self.p('control_on_wrench'),
                        'servo_compensation_s': self.servo_compensation.tolist(),
                        'qdot_primary_xy_rad_s': primary_qdot.tolist(),
                        'delta_p_adm_m': displacement.tolist(), 'delta_p_dot_adm_m_s': self.core.v.tolist(),
                        'p_home_m': self.reference.tolist(), 'p_ref_xy_m': target.tolist(),
                        'p_actual_m': actual.tolist(), 'e_track_xy_m': track.tolist(), 'e_arm_m': arm.tolist(),
                        'q_actual_rad': q.tolist(), 'q_cmd_rad': candidate.tolist(),
                        'q_nominal_rad': self.q_nominal.tolist(),
                        'q_start_cmd_rad': self.q_start_request.tolist(),
                        'trajectory_points':len(msg.points),
                        'qdot_dls_rad_s': qdot.tolist(), 'qdot_cmd_rad_s': applied_qdot.tolist(),
                        'r_arm_xy_m': float(np.linalg.norm(arm[:2])),
                        'ik_residual_xy_m_s': (self.model.jacobian(q)[:2]@qdot-velocity).tolist(),
                        'min_joint_margin_rad': margin, 'min_task_singular_value_m': sigma, 'dls_damping_m': damping,
                        'orientation_error_rad': pose_error.tolist(), 'base_drift_xy_yaw': drift.tolist(),
                        'wrench_increment': None if self.last_wrench is None else self.last_wrench.tolist(),
                        'dt_control_s': dt, 'v_ee_cmd_xy_m_s': velocity.tolist(),
                        'source_stamp_s': self.force_stamp}
            except (ValueError, KeyError, np.linalg.LinAlgError) as exc:
                self.trip(str(exc))
        self.debug_pub.publish(String(data=json.dumps({**self.measurements, **self.fault_debug,
            'task_frame': 'follower/base_footprint',
            'mode': 'ARM_ONLY', 'rotation_tracking_enabled': False, 'state': self.state,
            'phase_mode': 'SAFE_STOP' if self.latched or self.state == 'SAFE_STOP' else
                          'ARM_ONLY' if self.core is not None else 'INIT',
            'enable_base_assist': False, 'wrench_source_mode': self.p('wrench_source_mode'),
            'reason': self.reason, 'enabled': self.core is not None, 'sim_time': self.sim,
            'base_cmd': [0., 0., 0.], 'sample_is_last_control_step': self.core is None})))


def main(args=None):
    rclpy.init(args=args)
    node = StrictArmOnly()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
