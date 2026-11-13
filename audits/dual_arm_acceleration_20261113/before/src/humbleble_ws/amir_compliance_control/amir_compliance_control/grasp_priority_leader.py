"""Leader-local trajectory retiming from a boolean grasp-reserve warning.

The follower receives no return state or goal. Once warned in a rotation, the
leader retains a slower rate until completion; it never hunts back to full
speed or restarts the latched centre. Hard stop remains the existing path.
"""
from dataclasses import replace
import time

import numpy as np
import rclpy
from std_msgs.msg import Bool, String
import json
import math

from .object_rotation_leader import ObjectRotationLeader
from .grasp_reserve import native_reserve_risk
from .wrench import rotation


class GraspProgress:
    def __init__(self, slow_rate, deceleration):
        if not np.isfinite([slow_rate, deceleration]).all() or not 0. < slow_rate <= 1. or deceleration <= 0.:
            raise ValueError('bounded positive progress policy required')
        self.slow_rate, self.deceleration = float(slow_rate), float(deceleration)
        self.elapsed, self.rate, self.warned = 0., 1., False

    def step(self, dt, warning):
        if not np.isfinite(dt) or not 0. <= dt <= .05:
            raise ValueError('grasp progress timestep outside envelope')
        self.warned = self.warned or bool(warning)
        target = self.slow_rate if self.warned else 1.
        previous = self.rate
        self.rate = max(target, previous-self.deceleration*dt)
        self.elapsed += .5*(previous+self.rate)*dt
        return 0. if dt == 0. else (self.rate-previous)/dt

    def reference(self, trajectory, rate_dot):
        ref = trajectory.sample(self.elapsed)
        return replace(ref, omega=ref.omega*self.rate, alpha=ref.alpha*self.rate**2+ref.omega*rate_dot,
            base_velocity_world=ref.base_velocity_world*self.rate,
            grasp_velocity_world=ref.grasp_velocity_world*self.rate,
            base_acceleration_world=ref.base_acceleration_world*self.rate**2+ref.base_velocity_world*rate_dot,
            grasp_acceleration_world=ref.grasp_acceleration_world*self.rate**2+ref.grasp_velocity_world*rate_dot)


class GraspPriorityLeader(ObjectRotationLeader):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.declare_parameter('grasp_slow_rate', .5)
        self.declare_parameter('grasp_progress_deceleration_s_inv', .5)
        self.new_progress()
        self.progress, self.progress_stamp = None, None
        self.warning, self.warning_wall = False, 0.
        self.follower_enabled, self.follower_wall = False, 0.
        self.native_grasp, self.native_wall = {}, 0.
        self.declare_parameter('native_grasp_monitor_required', False)
        if self.p('native_grasp_monitor_required'):
            own = self.create_subscription(String, 'grasp/monitor_debug', self.native_sample, 10)
            if own.topic_name != '/leader/grasp/monitor_debug':
                raise ValueError('leader own native grasp boundary violated')
        sub = self.create_subscription(Bool, '/transport/rotation_grasp_warning', self.reserve, 10)
        ready = self.create_subscription(Bool, '/transport/rotation_follower_ready', self.follower_ready, 10)
        self.progress_debug = self.create_publisher(String, 'grasp_priority/debug', 10)
        if (sub.topic_name != '/transport/rotation_grasp_warning' or ready.topic_name != '/transport/rotation_follower_ready'
                or self.progress_debug.topic_name != '/leader/grasp_priority/debug'):
            raise ValueError('grasp-priority boolean safety boundary violated')

    def new_progress(self):
        return GraspProgress(self.p('grasp_slow_rate'), self.p('grasp_progress_deceleration_s_inv'))

    def reserve(self, msg):
        self.warning, self.warning_wall = bool(msg.data), time.monotonic()

    def follower_ready(self, msg):
        self.follower_enabled, self.follower_wall = bool(msg.data), time.monotonic()

    def native_sample(self, msg):
        try:
            self.native_grasp, self.native_wall = json.loads(msg.data), time.monotonic()
        except (ValueError, TypeError):
            self.native_grasp = {}

    def prepare(self, request, response):
        response = super().prepare(request, response)
        for topic in ('/transport/rotation_grasp_warning','/transport/rotation_follower_ready'):
            sources = self.get_publishers_info_by_topic(topic)
            if len(sources) != 1 or sources[0].node_name != 'follower_yaw_whole_body' or sources[0].node_namespace != '/follower':
                self.prepared = response.success = False
                response.message = 'verified local follower boolean safety source required'
        if self.p('native_grasp_monitor_required'):
            own = self.get_publishers_info_by_topic('/leader/grasp/monitor_debug')
            if len(own) != 1 or own[0].node_name != 'native_grasp_monitor' or own[0].node_namespace != '/leader':
                self.prepared = response.success = False
                response.message = 'verified own native grasp monitor required'
        return response

    def rotation_goal(self, msg):
        if not self.follower_enabled or time.monotonic()-self.follower_wall > self.p('timeout_s'):
            self.rejected += 1
            self.last_rejection = 'fresh enabled follower boolean required before rotation'
            return
        previous = self.accepted
        super().rotation_goal(msg)
        if self.accepted > previous:
            self.progress = self.new_progress()
            self.progress_stamp = self.rotation_session[0]

    def planning_object_pose(self, object_pose):
        if not self.p('native_grasp_monitor_required'):
            return object_pose
        # C is reconstructed with the FULL calibrated transform first. The
        # planning O axes are gravity aligned, while the full measured grasp
        # orientation remains the reference and rotates as Rz(phi)*R_grasp0.
        # Thus a tiny initial contact tilt is retained, not levelled by force.
        tilt = math.acos(float(np.clip(object_pose[2,2],-1.,1.)))
        if tilt > self.policy['rotation_arm_angle_limit_rad']:
            raise ValueError('native initial tilt exceeds the existing pose envelope')
        result = object_pose.copy()
        heading = math.atan2(object_pose[1,0],object_pose[0,0])
        result[:3,:3] = rotation([0.,0.,1.],heading)
        self.initial_object_tilt_rad = tilt
        return result

    def rotation_reference(self, start, trajectory):
        if not self.follower_enabled or time.monotonic()-self.follower_wall > self.p('timeout_s'):
            raise ValueError('follower control disabled or readiness heartbeat lost')
        if time.monotonic()-self.warning_wall > self.p('timeout_s'):
            raise ValueError('grasp-reserve boolean heartbeat lost')
        if self.sim < start:
            return self.sim-start, trajectory.sample(self.sim-start)
        dt = self.sim-max(start, self.progress_stamp)
        warning = self.warning
        if self.p('native_grasp_monitor_required'):
            own_risk = native_reserve_risk(self.native_grasp,self.sim,time.monotonic()-self.native_wall)
            warning = warning or own_risk >= .6
        rate_dot = self.progress.step(dt, warning)
        self.progress_stamp = self.sim
        ref = self.progress.reference(trajectory, rate_dot)
        self.progress_debug.publish(String(data=json.dumps({'sim_time': self.sim,
            'progress_time_s': self.progress.elapsed, 'rate': self.progress.rate,
            'warning_latched': self.progress.warned, 'warning': self.warning,
            'initial_object_tilt_rad':getattr(self,'initial_object_tilt_rad',0.)})))
        return self.progress.elapsed, ref

    def trip(self, reason):
        super().trip(reason)
        self.progress = self.progress_stamp = None


def main(args=None):
    rclpy.init(args=args)
    node = GraspPriorityLeader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
