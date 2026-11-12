#!/usr/bin/env python3
"""EVALUATION CONTROL ONLY: common known trajectory, never local-following mode.

This deliberately synchronized comparison lives outside the control package.
Do not deploy it as a follower: it consumes a common angle/time/fixture schedule.
It still uses each robot's own encoder, exclusive base_cmd and dual safety path.
"""
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from std_msgs.msg import Float64, String

from amir_compliance_control.object_center_rotation_trajectory import ObjectCenterRotation, wrap_pi
from amir_compliance_control.planar_leader import yaw_of
from amir_compliance_control.transport import TransportEndpoint
from amir_compliance_control.wrench import rotation


class SynchronizedReference(TransportEndpoint):
    def __init__(self, robot, **kwargs):
        super().__init__(leader=robot == 'leader', node_name='evaluation_synchronized_reference', **kwargs)
        self.declare_parameter('schedule_file', '')
        self.schedule_file = Path(self.p('schedule_file'))
        self.path = None
        self.velocity = np.zeros(3)
        self.create_subscription(Float64, '/experiment/sync_start', self.start_reference, 1)
        self.debug = self.create_publisher(String, '/experiment/sync_'+robot+'/debug', 10)

    def start_reference(self, msg):
        if self.state != 'READY' or self.path is not None or self.latched:
            return
        if not self.grasp or time.monotonic()-self.grasp_wall > 1.:
            self.trip('evaluation schedule requires fixed grasp')
            return
        schedule = json.loads(self.schedule_file.read_text())
        data = schedule[self.robot]
        base = np.eye(4)
        p = self.encoder.pose.pose
        base[:3, 3] = [p.position.x, p.position.y, p.position.z]
        base[:3, :3] = rotation([0., 0., 1.], yaw_of(p))
        mapping = dict(zip(self.joints.name, self.joints.position))
        tcp, sensor = self.model.frames([mapping[n] for n in self.model.names])
        local_grasp = sensor.copy()
        local_grasp[:3, 3] = tcp
        path = ObjectCenterRotation(data['object0_odom'], base, base@local_grasp,
                                    duration=schedule['duration_s'], delta_yaw_rad=schedule['delta_rad'])
        self.path = (msg.data, path, schedule['duration_s'], local_grasp)
        self.state, self.accepted = 'ROTATE_EVALUATION_SYNCHRONIZED', self.accepted+1

    def goal(self, msg):
        self.rejected += 1
        self.last_rejection = 'synchronized evaluation accepts only its explicit schedule'

    def base_request(self, command):
        if self.path is None or self.latched:
            return np.zeros(3)
        start, path, duration, grasp0 = self.path
        ref = path.sample(self.sim-start)
        p = self.encoder.pose.pose
        angle = yaw_of(p)
        actual = np.array([p.position.x, p.position.y, p.position.z])
        error = ref.base_pose[:3, 3]-actual
        angle_error = wrap_pi(ref.base_yaw_unwrapped-angle)
        mapping = dict(zip(self.joints.name, self.joints.position))
        tcp, sensor = self.model.frames([mapping[n] for n in self.model.names])
        arm_error = np.linalg.norm(tcp-grasp0[:3, 3])
        arm_angle = math.acos(float(np.clip((np.trace(sensor[:3, :3]@grasp0[:3, :3].T)-1.)/2., -1., 1.)))
        if np.linalg.norm(error[:2]) > .01 or abs(angle_error) > .01 or arm_error > .0005 or arm_angle > .01:
            self.trip('evaluation tracking/grasp pose envelope exceeded')
            return np.zeros(3)
        vw = ref.base_velocity_world+error
        body = rotation([0., 0., 1.], angle).T@vw
        desired = np.r_[body[:2], ref.omega+angle_error]
        length = np.linalg.norm(desired[:2])
        if length > .002:
            desired[:2] *= .002/length
        desired[2] = np.clip(desired[2], -.002, .002)
        dt = self.sim-self.last_tick_sim if self.last_tick_sim is not None else 0.
        if not 0. <= dt <= .05:
            self.trip('evaluation timestep outside envelope')
            return np.zeros(3)
        d = desired-self.velocity
        size = np.linalg.norm(d[:2])
        if size > .001*dt:
            d[:2] *= .001*dt/max(size, 1e-15)
        d[2] = np.clip(d[2], -.001*dt, .001*dt)
        self.velocity += d
        if self.sim-start >= duration+1. and np.linalg.norm(error[:2]) < .0001 and abs(angle_error) < .0001:
            self.velocity[:] = 0.
            self.path, self.state, self.completed = None, 'READY', self.completed+1
        elif self.sim-start > duration+20.:
            self.trip('evaluation terminal timeout')
            return np.zeros(3)
        self.debug.publish(String(data=json.dumps({'sim_time': self.sim, 'phi_rad': ref.phi,
            'omega_rad_s': ref.omega, 'base_error_xy_m': error[:2].tolist(), 'yaw_error_rad': angle_error,
            'arm_position_error_m': float(arm_error), 'arm_angle_error_rad': arm_angle,
            'center_odom_xyz': ref.object_pose[:3, 3].tolist(),
            'command_body_vx_vy_wz': self.velocity.tolist()})))
        return self.velocity.copy()

    def tick(self):
        if getattr(self, 'path', None) is not None and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
            self.trip('evaluation local fixed grasp lost')
        super().tick()

    def trip(self, reason):
        super().trip(reason)
        self.path = None
        if hasattr(self, 'velocity'):
            self.velocity[:] = 0.


def main():
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot', choices=['leader', 'follower'], required=True)
    args, remaining = p.parse_known_args()
    rclpy.init(args=remaining)
    node = SynchronizedReference(args.robot, namespace='/'+args.robot)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
