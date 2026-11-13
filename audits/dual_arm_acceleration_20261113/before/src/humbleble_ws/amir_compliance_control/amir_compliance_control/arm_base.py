"""CP-6 ARM_BASE from actual local arm deflection and local F/T only."""
import json
import time
import numpy as np
import rclpy
from std_msgs.msg import String
from .arm_admittance import ArmAdmittance
from .admittance import solve_pose, rotation_error
from .recentering import BaseRecentering
from .wrench import finite


class ArmBase(ArmAdmittance):
    def __init__(self, **kwargs):
        super().__init__(node_name='follower_arm_base',
            mechanics_defaults={'virtual_mass': [20.,20.], 'virtual_damping': [1200.,1200.]}, **kwargs)
        self.role = 'ARM_BASE'
        for key, value in {'base_cutoff_hz': .2, 'base_gains': [.5, .5, 1.],
                'base_velocity_limits': [.0005, .0005, .001],
                'base_acceleration_limits': [.001, .001, .002],
                'preserve_orientation': True, 'straight_only': True, 'allow_ungrasped_test': False}.items():
            self.declare_parameter(key, value)
        self.declare_parameter('monitor_pose', False)
        self.declare_parameter('height_error_limit_m', .0001)
        self.declare_parameter('rotation_error_limit_rad', .01)
        for key in ('height_error_limit_m','rotation_error_limit_rad'):
            if not np.isfinite(self.p(key)) or self.p(key)<=0: raise ValueError('positive pose-monitor policy required')
        self.new_base()
        self.base = None
        self.base_twist, self.measured_error = np.zeros(3), np.zeros(3)
        self.base_wall, self.base_stamp, self.reference_rotation = 0., None, None
        self.base_debug = self.create_publisher(String, 'recentering/debug', 10)
        if self.base_debug.topic_name != '/follower/recentering/debug':
            raise ValueError('local base output boundary violated')

    def new_base(self):
        return BaseRecentering(self.p('base_cutoff_hz'), self.p('base_gains'),
                               self.p('base_velocity_limits'), self.p('base_acceleration_limits'))

    def enable(self, request, response):
        if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
            response.success, response.message = False, 'ARM_BASE requires confirmed local grasp'
            return response
        response = super().enable(request, response)
        if response.success:
            self.reference_rotation = self.model.frames(self.q_request)[1][:3, :3]
            try:
                if self.p('preserve_orientation'):
                    for direction in (-1., 1.):
                        solve_pose(self.model, self.q_request, self.reference+[direction*.0005, 0., 0.], self.reference_rotation)
                self.base, self.base_stamp = self.new_base(), self.force_stamp
                self.base_twist[:] = 0.
                self.base_wall = time.monotonic()
                response.message = 'ARM_BASE enabled from local measured state'
            except ValueError as exc:
                self.core, self.state = None, 'READY'
                response.success, response.message = False, str(exc)
        return response

    def control_force(self):
        f = self.force.copy()
        if self.p('straight_only'):
            f[1] = 0.
        return f

    def solve_request(self, target):
        if self.p('preserve_orientation'):
            return solve_pose(self.model, self.q_request, target, self.reference_rotation)
        return super().solve_request(target)

    def base_request(self, command):
        if (self.core is None or self.latched or time.monotonic()-self.base_wall > .05):
            return np.zeros(3)
        return self.base_twist

    def trip(self, reason):
        super().trip(reason)
        self.base = None
        if hasattr(self, 'base_twist'):
            self.base_twist[:] = 0.

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.base = None
            self.base_twist[:] = 0.
            self.base_stamp = self.reference_rotation = None
        return response

    def tick(self):
        super().tick()
        if not hasattr(self, 'base'):
            return
        if self.core is not None and self.base is not None and self.force_stamp != self.base_stamp:
            try:
                if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                    raise ValueError('local grasp missing during ARM_BASE')
                dt = self.force_stamp-self.base_stamp
                mapping = dict(zip(self.joints.name, self.joints.position))
                q = finite([mapping[n] for n in self.model.names], (5,))
                grasp, sensor = self.model.frames(q)
                angular = rotation_error(sensor[:3, :3], self.reference_rotation)
                if self.p('monitor_pose') and (np.any(np.abs(grasp[:2]-self.reference[:2])>
                        np.asarray(self.p('displacement_limit_m'))+1e-7)
                        or abs(grasp[2]-self.reference[2])>self.p('height_error_limit_m')
                        or np.linalg.norm(angular)>self.p('rotation_error_limit_rad')):
                    raise ValueError('measured local grasp pose left validated envelope')
                self.measured_error = np.r_[grasp[:2]-self.reference[:2], angular[2]]
                self.base_twist = self.base.step(self.measured_error, grasp, dt)
                self.base_stamp, self.base_wall = self.force_stamp, time.monotonic()
            except ValueError as exc:
                self.trip(str(exc))
        self.base_debug.publish(String(data=json.dumps({'mode': 'ARM_BASE', 'sim_time': self.sim,
            'enabled': self.core is not None and self.base is not None, 'state': self.state, 'reason': self.reason,
            'measured_error_xy_yaw': self.measured_error.tolist(),
            'low_error_xy_yaw': None if self.base is None else self.base.low.tolist(),
            'base_twist': self.base_twist.tolist()})))


def main(args=None):
    rclpy.init(args=args); node = ArmBase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.try_shutdown()
