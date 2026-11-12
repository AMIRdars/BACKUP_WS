"""Opt-in CP-8 follower: own F/T and measured arm deflection only."""
import json
import time
import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .arm_admittance import ArmAdmittance
from .admittance import PlanarAdmittance, rotation_error
from .recentering import BaseRecentering
from .transport import TransportEndpoint
from .whole_body import WholeBodyAllocator, base_jacobian
from .wrench import finite


class WholeBodyFollower(ArmAdmittance):
    def __init__(self, **kwargs):
        super().__init__(node_name='follower_whole_body', mechanics_defaults={
            'virtual_mass': [20., 20.], 'virtual_damping': [1200., 1200.]}, **kwargs)
        self.role = 'WHOLE_BODY'
        for key, value in {'base_cutoff_hz': .2, 'base_gains': [.5, .5, 1.],
                'base_integral_gains': [0., 0., 0.],
                'base_force_acceleration_gains': [0., 0., 0.], 'base_force_filter_hz': .5,
                'base_stationary_hold': False,
                'base_velocity_limits': [.0005, .0005, .001],
                'base_acceleration_limits': [.001, .001, .002],
                'allocation_base_filter_hz': 0.,
                'allocation_base_cost_velocity': [0., 0., 0.],
                'joint_acceleration_rad_s2': .15, 'pose_gain_s_inv': 2.,
                'arm_command_horizon_s': .04,
                'allocation_on_wrench': False,
                'allocation_use_clock': False,
                'virtual_yaw_inertia': .2, 'virtual_yaw_damping': 12., 'virtual_yaw_stiffness': 4.,
                'yaw_displacement_limit_rad': .001, 'yaw_velocity_limit_rad_s': .002,
                'yaw_acceleration_limit_rad_s2': .02, 'height_error_limit_m': .0001,
                'rotation_error_limit_rad': .01, 'allow_ungrasped_test': False}.items():
            self.declare_parameter(key, value)
        for key in ('pose_gain_s_inv', 'height_error_limit_m', 'rotation_error_limit_rad', 'arm_command_horizon_s'):
            if not np.isfinite(self.p(key)) or self.p(key) <= 0: raise ValueError('positive pose policy required')
        self.yaw_core = self.make_yaw()
        self.new_allocator()
        self.new_recentering()
        self.allocator = self.base = None
        self.base_twist, self.measured_error = np.zeros(3), np.zeros(3)
        self.base_wall, self.wrench6, self.reference_rotation = 0., np.zeros(6), None
        self.allocation_wall_ms = 0.
        self.step_dt = 0.
        self.was_local_hold = False
        self.first_control_fault = {}
        self.base_debug = self.create_publisher(String, 'recentering/debug', 10)
        self.allocation_debug = self.create_publisher(String, 'allocation/debug', 10)
        if self.base_debug.topic_name != '/follower/recentering/debug' or self.allocation_debug.topic_name != '/follower/allocation/debug':
            raise ValueError('whole body output boundary violated')

    def make_yaw(self):
        return PlanarAdmittance(*([self.p(k)]*2 for k in ('virtual_yaw_inertia', 'virtual_yaw_damping',
            'virtual_yaw_stiffness', 'yaw_displacement_limit_rad', 'yaw_velocity_limit_rad_s', 'yaw_acceleration_limit_rad_s2')))

    def new_allocator(self):
        cost = finite(self.p('allocation_base_cost_velocity'), (3,))
        return WholeBodyAllocator(self.model, arm_speed=self.p('joint_request_speed_rad_s'),
            arm_acceleration=self.p('joint_acceleration_rad_s2'), base_speed=self.p('base_velocity_limits'),
            base_acceleration=self.p('base_acceleration_limits'), base_filter_hz=self.p('allocation_base_filter_hz'),
            base_cost_velocity=None if np.all(cost == 0.) else cost)

    def new_recentering(self):
        return BaseRecentering(self.p('base_cutoff_hz'), self.p('base_gains'),
            self.p('base_velocity_limits'), self.p('base_acceleration_limits'),
            integral_gains=self.p('base_integral_gains'),
            force_acceleration_gains=self.p('base_force_acceleration_gains'),
            force_filter_hz=self.p('base_force_filter_hz'), stationary_hold=self.p('base_stationary_hold'))

    def filtered(self, msg):
        super().filtered(msg)
        if self.force is not None:
            w = msg.wrench
            self.wrench6 = finite([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], (6,))
            if (self.p('allocation_on_wrench') and self.core is not None and not self.latched
                    and self.permit and not self.emergency and self.local_health()[0]
                    and time.monotonic()-self.permit_wall <= self.p('timeout_s')):
                self.update_allocation()

    def enable(self, request, response):
        if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
            response.success, response.message = False, 'whole body requires confirmed local grasp'
            return response
        response = super().enable(request, response)
        if response.success:
            if self.p('allocation_use_clock'):
                self.last_step_stamp = self.sim
            self.reference_rotation = self.model.frames(self.q_request)[1][:3, :3]
            self.allocator, self.yaw_core = self.new_allocator(), self.make_yaw()
            self.base = self.new_recentering()
            self.was_local_hold = False
            self.first_control_fault = {}
            self.base_twist[:] = 0.
            self.base_wall = time.monotonic()
            response.message = 'WHOLE_BODY enabled from measured local TCP; low-pass base authority'
        return response

    def base_request(self, command):
        # Consume an already received force sample BEFORE testing allocation
        # freshness and publishing the base command. Publishing first could
        # insert a single zero velocity even though this tick had fresh input.
        if self.core is not None and not self.latched:
            self.update_allocation()
            if self.core is not None and time.monotonic()-self.base_wall > .05:
                self.trip('whole body allocation watchdog')
        return self.base_twist if self.core is not None and not self.latched else np.zeros(3)

    def trip(self, reason):
        if not self.latched and hasattr(self, 'base_wall'):
            wall=time.monotonic()
            self.first_control_fault={'reason':reason,'sim_time':self.sim,
                'allocation_age_wall_s':wall-self.base_wall,
                'force_age_wall_s':wall-self.force_wall,'force_stamp_s':self.force_stamp,
                'last_step_stamp_s':self.last_step_stamp,'calculation_wall_ms':self.allocation_wall_ms}
            self.get_logger().error('First whole-body stop: '+json.dumps(self.first_control_fault))
        super().trip(reason)
        self.allocator = self.base = None
        self.was_local_hold = False
        if hasattr(self, 'base_twist'): self.base_twist[:] = 0.

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.allocator = self.base = None
            self.base_twist[:] = 0.
        return response

    def update_allocation(self):
        if self.core is not None:
            try:
                if not self.wrench_ready(): raise ValueError('filtered wrench/clock watchdog')
                if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                    raise ValueError('local grasp missing during whole body transport')
                step_stamp = self.sim if self.p('allocation_use_clock') else self.force_stamp
                dt = step_stamp-self.last_step_stamp
                self.step_dt = dt
                if dt > 0:
                    calculation_started = time.perf_counter()
                    mapping = dict(zip(self.joints.name, self.joints.position))
                    q = finite([mapping[n] for n in self.model.names], (5,))
                    position, sensor = self.model.frames(q)
                    angular = rotation_error(sensor[:3, :3], self.reference_rotation)
                    error = np.r_[position-self.reference, angular]
                    if (np.any(abs(error[:2]) > np.asarray(self.p('displacement_limit_m'))+1e-7)
                            or abs(error[2]) > self.p('height_error_limit_m')
                            or np.linalg.norm(angular) > self.p('rotation_error_limit_rad')):
                        raise ValueError('measured local grasp pose left validated envelope')
                    self.measured_error = error[[0, 1, 5]]
                    xy = self.core.step(self.force, dt)
                    yaw = self.yaw_core.step([self.wrench6[5], 0.], dt)[0]
                    target = np.array([xy[0], xy[1], 0., 0., 0., yaw])
                    virtual_velocity = np.r_[self.core.v, 0., 0., 0., self.yaw_core.v[0]]
                    hold_allowed = True
                    base_velocity = None
                    if self.base.stationary_hold:
                        twist = self.encoder.twist.twist
                        base_velocity = finite([twist.linear.x,twist.linear.y,twist.angular.z],(3,))
                        hold_allowed = np.all(abs(target[:2]-error[:2]) <= .0001) and abs(target[5]-error[5]) <= .001
                        if not self.was_local_hold:
                            hold_allowed = hold_allowed and np.all(abs(virtual_velocity[:2]) <= .00001)
                    preferred = self.base.step(self.measured_error, position, dt,
                        measured_force=self.wrench6[[0,1,5]] if self.base.force_memory_enabled else None,
                        measured_base_velocity=base_velocity, allow_stationary_hold=hold_allowed)
                    desired = virtual_velocity+self.p('pose_gain_s_inv')*(target-error)
                    desired += base_jacobian(position)@preferred
                    # Scale is explicitly a software displacement envelope.
                    credit = self.base.authority(np.r_[self.p('displacement_limit_m'), self.p('yaw_displacement_limit_rad')])
                    local_hold = self.base.stationary_hold and bool(np.any(self.base.hold))
                    if local_hold:
                        if not self.was_local_hold:
                            self.q_request = q.copy()
                            self.allocator = self.new_allocator()
                        velocity = np.zeros(8)
                    else:
                        velocity = self.allocator.step(q, self.q_request, desired, preferred, credit, dt)
                    self.was_local_hold = local_hold
                    self.q_request += velocity[:5]*dt
                    self.base_twist = velocity[5:]
                    self.base_wall = time.monotonic()
                    msg = JointTrajectory(joint_names=list(self.model.names))
                    msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'follower/base_footprint'
                    point = JointTrajectoryPoint(positions=self.q_request.tolist())
                    duration_ns = int(round(self.p('arm_command_horizon_s')*1e9))
                    point.time_from_start.sec,point.time_from_start.nanosec = divmod(duration_ns,1000000000)
                    msg.points = [point]
                    self.publish_command(self.arm_pub, msg)
                    self.last_step_stamp = step_stamp
                    self.allocation_wall_ms = (time.perf_counter()-calculation_started)*1000.
            except ValueError as exc:
                self.trip(str(exc)+f'; dt={self.step_dt:.12g} force_stamp={self.force_stamp} last_step={self.last_step_stamp}')

    def tick(self):
        # Keep the CP-7 home/arm epochs, ownership, command transport and stop
        # path. Replace only the CP-5 IK step with the constrained allocator.
        TransportEndpoint.tick(self)
        if not hasattr(self, 'allocator'): return
        self.update_allocation()
        self.debug_pub.publish(String(data=json.dumps({'state': self.state, 'reason': self.reason,
            'enabled': self.core is not None, 'sim_time': self.sim, 'force_xy_N': None if self.force is None else self.force.tolist(),
            'reference_xyz_m': None if self.reference is None else self.reference.tolist(),
            'displacement_xy_m': None if self.core is None else self.core.x.tolist(),
            'velocity_xy_m_s': None if self.core is None else self.core.v.tolist(),
            'saturated_xy': None if self.core is None else self.core.saturated.tolist()})))
        self.base_debug.publish(String(data=json.dumps({'mode': 'WHOLE_BODY', 'sim_time': self.sim,
            'enabled': self.core is not None and self.base is not None, 'state': self.state, 'reason': self.reason,
            'measured_error_xy_yaw': self.measured_error.tolist(), 'base_twist': self.base_twist.tolist(),
            'low_error_xy_yaw': None if self.base is None else self.base.low.tolist(),
            'integral_velocity_xy_yaw': None if self.base is None else self.base.integral.tolist(),
            'force_memory_velocity_xy_yaw': None if self.base is None else self.base.force_memory.tolist(),
            'stationary_hold_axes': None if self.base is None else self.base.hold.tolist()})))
        self.allocation_debug.publish(String(data=json.dumps({'sim_time': self.sim, 'enabled': self.allocator is not None,
            'calculation_wall_ms': self.allocation_wall_ms,
            'force_stamp_s': self.force_stamp, 'last_step_stamp_s': self.last_step_stamp, 'step_dt_s': self.step_dt,
            'first_control_fault':self.first_control_fault,
            **({} if self.allocator is None else self.allocator.last)})))


def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyFollower()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
