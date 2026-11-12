"""Opt-in ARM_BASE xy+yaw, using only own FT/FK/joints/encoder and safety.

SO(3) log is used at the actual near-singular Euler home. No leader goal,
position, speed, angle, centre, evaluation or ground-truth input is admitted.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .admittance import PlanarAdmittance, rotation_error
from .arm_admittance import ArmAdmittance
from .local_yaw import ArmSoftTask, YawRecentering
from .transport import TransportEndpoint
from .safety import stamp
from .wrench import finite, rotation


class YawArmBase(ArmAdmittance):
    def __init__(self, *, node_name='follower_yaw_arm_base', **kwargs):
        super().__init__(node_name=node_name, mechanics_defaults={
            'virtual_mass': [20., 20.], 'virtual_damping': [1200., 1200.]}, **kwargs)
        self.role = 'ARM_BASE_XY_YAW'
        settings = {'base_cutoff_hz': .2, 'base_gains': [.5, .5, 1.],
            'base_velocity_limits': [.0005, .0005, .001],
            'base_acceleration_limits': [.001, .001, .002],
            'base_force_acceleration_gains': [0., 0., 0.], 'base_force_velocity_gains': [0., 0., 0.], 'base_force_filter_hz': .1,
            'virtual_yaw_inertia': .2, 'virtual_yaw_damping': 12., 'virtual_yaw_stiffness': 4.,
            'yaw_displacement_limit_rad': .001, 'yaw_velocity_limit_rad_s': .002,
            'yaw_acceleration_limit_rad_s2': .02, 'pose_gain_s_inv': 2.,
            'arm_task_metric': [1., 1., 1., .1, .1, .1],
            'joint_acceleration_rad_s2': .15, 'joint_margin_rad': .01,
            'wrench_force_limit_N': 0., 'wrench_torque_limit_Nm': 0.,
            'height_error_limit_m': .0001, 'rotation_error_limit_rad': .01,
            'allow_ungrasped_test': False}
        for k, v in settings.items():
            self.declare_parameter(k, v)
        for k in ('pose_gain_s_inv', 'height_error_limit_m', 'rotation_error_limit_rad',
                  'wrench_force_limit_N', 'wrench_torque_limit_Nm'):
            if not np.isfinite(self.p(k)) or self.p(k) <= 0:
                raise ValueError('positive local pose monitor policy required')
        self.new_arm(), self.new_base(), self.new_yaw()
        self.arm_task = self.base = self.yaw_core = None
        self.wrench6, self.base_twist = np.zeros(6), np.zeros(3)
        self.reference_rotation = None
        self.base_wall = 0.
        self.raw_wrench, self.raw_bias = None, None
        self.wrench_violation = None
        self.raw_stamp, self.raw_wall = 0., 0.
        self.measured_error = np.zeros(3)
        self.base_debug = self.create_publisher(String, 'recentering/debug', 10)
        self.yaw_debug = self.create_publisher(String, 'rotation_follow/debug', 10)
        raw_sub = self.create_subscription(WrenchStamped, 'wrench_at_grasp_raw', self.raw, qos_profile_sensor_data)
        if self.base_debug.topic_name != '/follower/recentering/debug' or self.yaw_debug.topic_name != '/follower/rotation_follow/debug':
            raise ValueError('local yaw debug boundary violated')
        if raw_sub.topic_name != '/follower/wrench_at_grasp_raw':
            raise ValueError('local raw wrench boundary violated')

    def prepare(self, request, response):
        response = super().prepare(request, response)
        sources = self.get_publishers_info_by_topic('/follower/wrench_at_grasp_raw')
        if len(sources) != 1 or sources[0].node_name != 'wrench_preprocessor' or sources[0].node_namespace != '/follower':
            self.prepared = response.success = False
            response.message = 'unverified local raw TCP wrench publisher'
        return response

    def raw(self, msg):
        try:
            t = stamp(msg)
            if t <= self.raw_stamp or msg.header.frame_id != 'follower/grasp_aligned':
                raise ValueError('invalid local raw TCP wrench frame/timestamp')
            w = msg.wrench
            self.raw_wrench = finite([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], (6,))
            self.raw_stamp, self.raw_wall = t, time.monotonic()
            if self.core is not None and self.raw_bias is not None:
                self.check_wrench(self.raw_wrench-self.raw_bias)
        except ValueError as exc:
            self.raw_wrench = None
            if self.core is not None:
                self.trip(str(exc))

    def check_wrench(self, value):
        force, torque = float(np.linalg.norm(value[:3])), float(np.linalg.norm(value[3:]))
        if force >= self.p('wrench_force_limit_N') or torque >= self.p('wrench_torque_limit_Nm'):
            self.wrench_violation = {'wrench': value.tolist(), 'force_norm_N': force, 'torque_norm_Nm': torque,
                                     'sim_time': self.sim}
            raise ValueError(f'local xy+yaw tared TCP wrench envelope exceeded: {force:.6g} N, {torque:.6g} Nm')

    def new_yaw(self):
        return PlanarAdmittance(*([self.p(k)]*2 for k in ('virtual_yaw_inertia', 'virtual_yaw_damping',
            'virtual_yaw_stiffness', 'yaw_displacement_limit_rad', 'yaw_velocity_limit_rad_s', 'yaw_acceleration_limit_rad_s2')))

    def new_arm(self):
        return ArmSoftTask(self.model, metric=self.p('arm_task_metric'), speed=self.p('joint_request_speed_rad_s'),
                           acceleration=self.p('joint_acceleration_rad_s2'), margin=self.p('joint_margin_rad'))

    def new_base(self):
        return YawRecentering(cutoff=self.p('base_cutoff_hz'), gains=self.p('base_gains'),
            velocity=self.p('base_velocity_limits'), acceleration=self.p('base_acceleration_limits'),
            force_gains=self.p('base_force_acceleration_gains'), force_velocity_gains=self.p('base_force_velocity_gains'),
            force_cutoff=self.p('base_force_filter_hz'))

    def filtered(self, msg):
        super().filtered(msg)
        if self.force is not None:
            w = msg.wrench
            self.wrench6 = finite([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], (6,))
            if (self.core is not None and not self.latched and self.permit and not self.emergency
                    and self.local_health()[0] and time.monotonic()-self.permit_wall <= self.p('timeout_s')):
                self.update_local()

    def enable(self, request, response):
        twist = None if self.encoder is None else self.encoder.twist.twist
        if (self.raw_wrench is None or time.monotonic()-self.raw_wall > .05 or twist is None
                or max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z)) >= .0001
                or not -.02 <= self.sim-self.raw_stamp <= .05
                or self.force_stamp is None or abs(self.raw_stamp-self.force_stamp) > .02):
            response.success, response.message = False, 'fresh stationary local raw/filtered wrench pair required'
            return response
        if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
            response.success, response.message = False, 'xy+yaw ARM_BASE requires fresh local grasp'
            return response
        response = super().enable(request, response)
        if response.success:
            self.wrench_violation = None
            self.reference_rotation = self.model.frames(self.q_request)[1][:3, :3].copy()
            self.raw_bias = self.raw_wrench-self.wrench6
            self.arm_task, self.base, self.yaw_core = self.new_arm(), self.new_base(), self.new_yaw()
            self.base_twist[:] = 0.
            self.base_wall = time.monotonic()
            response.message = 'local xy+yaw ARM_BASE enabled; no peer state/goal input'
        return response

    def update_local(self):
        if self.core is None or self.latched:
            return
        try:
            if not self.wrench_ready():
                raise ValueError('local xy+yaw wrench watchdog')
            if (self.raw_wrench is None or time.monotonic()-self.raw_wall > .05
                    or not -.02 <= self.sim-self.raw_stamp <= .05):
                raise ValueError('local unfiltered TCP wrench watchdog')
            self.check_wrench(self.wrench6)
            self.check_wrench(self.raw_wrench-self.raw_bias)
            if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                raise ValueError('local xy+yaw grasp confirmation lost')
            dt = self.force_stamp-self.last_step_stamp
            if dt == 0.:
                return
            if dt < 0.:
                raise ValueError('local xy+yaw time moved backwards')
            mapping = dict(zip(self.joints.name, self.joints.position))
            q = finite([mapping[n] for n in self.model.names], (5,))
            position, sensor = self.model.frames(q)
            angular = rotation_error(sensor[:3, :3], self.reference_rotation)
            error = position-self.reference
            if (np.any(abs(error[:2]) > np.array(self.p('displacement_limit_m'))+1e-7)
                    or abs(error[2]) > self.p('height_error_limit_m')
                    or np.linalg.norm(angular) > self.p('rotation_error_limit_rad')):
                raise ValueError('local xy+yaw measured grasp pose envelope exceeded')
            self.measured_error = np.r_[error[:2], angular[2]]
            xy = self.core.step(self.force, dt)
            yaw = self.yaw_core.step([self.wrench6[5], 0.], dt)[0]
            target_r = rotation([0., 0., 1.], yaw)@self.reference_rotation
            ep = self.reference+np.r_[xy, 0.]-position
            er = rotation_error(target_r, sensor[:3, :3])
            desired = np.r_[self.core.v, 0., 0., 0., self.yaw_core.v[0]]+self.p('pose_gain_s_inv')*np.r_[ep, er]
            velocity = self.arm_task.step(q, self.q_request, desired, dt)
            self.q_request += velocity*dt
            self.base_twist = self.base.step(self.measured_error, position, self.wrench6[[0, 1, 5]], dt)
            self.last_step_stamp, self.base_wall = self.force_stamp, time.monotonic()
            msg = JointTrajectory(joint_names=list(self.model.names))
            msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'follower/base_footprint'
            point = JointTrajectoryPoint(positions=self.q_request.tolist())
            point.time_from_start.nanosec = 40000000
            msg.points = [point]
            self.publish_command(self.arm_pub, msg)
        except (ValueError, KeyError) as exc:
            self.trip(str(exc))

    def base_request(self, command):
        if self.core is not None and not self.latched:
            self.update_local()
            if self.core is not None and time.monotonic()-self.base_wall > .05:
                self.trip('xy+yaw ARM_BASE update watchdog')
        return self.base_twist.copy() if self.core is not None and not self.latched else np.zeros(3)

    def trip(self, reason):
        super().trip(reason)
        self.arm_task = self.base = self.yaw_core = None
        self.raw_bias = None
        if hasattr(self, 'base_twist'):
            self.base_twist[:] = 0.

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.arm_task = self.base = self.yaw_core = self.reference_rotation = None
            self.raw_bias = None
            self.base_twist[:] = 0.
        return response

    def tick(self):
        TransportEndpoint.tick(self)
        if not hasattr(self, 'arm_task'):
            return
        self.update_local()
        debug = {'mode': self.role, 'sim_time': self.sim, 'state': self.state, 'reason': self.reason,
            'enabled': self.core is not None, 'measured_error_xy_yaw': self.measured_error.tolist(),
            'filtered_tcp_wrench': self.wrench6.tolist(),
            'wrench_violation': self.wrench_violation,
            'base_twist': self.base_twist.tolist(), 'tau_z_at_tcp_Nm': float(self.wrench6[5]),
            'raw_tared_tcp_wrench': None if self.raw_wrench is None or self.raw_bias is None else (self.raw_wrench-self.raw_bias).tolist(),
            'force_xy_N': None if self.force is None else self.force.tolist(),
            'reference_xyz_m': None if self.reference is None else self.reference.tolist(),
            'admittance_xy_m': None if self.core is None else self.core.x.tolist(),
            'admittance_yaw_rad': None if self.yaw_core is None else float(self.yaw_core.x[0]),
            'low_error_xy_yaw': None if self.base is None else self.base.low.tolist(),
            'force_memory_velocity_xy_yaw': None if self.base is None else self.base.memory.tolist(),
            **({} if self.arm_task is None else self.arm_task.last),
            **getattr(self,'rotation_diagnostics',{})}
        message = String(data=json.dumps(debug))
        self.base_debug.publish(message)
        self.yaw_debug.publish(message)
        self.debug_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = YawArmBase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
