#!/usr/bin/env python3
"""Exact ROS timestamp comparisons for the isolated CP-13 simulation gate.

All limits, sensor subscriptions and shared stop behavior are inherited from
RobotSafetyGate. Only healthy() timestamp arithmetic changes to integer ns.
The normal CP-12 entry continues to use its unchanged original gate.
"""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from amir_compliance_control.safety import RobotSafetyGate
from amir_compliance_control.wrench import finite


class PreciseSafetyGate(RobotSafetyGate):
    def __init__(self, **kwargs):
        self.sim_ns=0
        super().__init__(**kwargs)

    def clock(self, msg):
        self.sim_ns=msg.clock.sec*1_000_000_000+msg.clock.nanosec
        super().clock(msg)

    def healthy(self):
        wall = time.monotonic()
        needed = {'joints', 'encoder', *self.model.names}
        if not needed.issubset(self.samples) or self.sim <= 0:
            return False, 'missing local sensors/clock'
        if wall-self.clock_wall > self.p('heartbeat_timeout_s'):
            return False, 'simulation clock stalled'
        if self.grasp_ever and (not self.grasp or self.sim-self.grasp_sim > .35 or wall-self.grasp_wall > 1.):
            return False, 'local grasp heartbeat missing/detached'
        # ROS stamps are exact integer nanoseconds. Subtracting float seconds
        # can turn exactly 20 ms into 0.020000000000003 s and spuriously trip
        # the inclusive timing boundary. Preserve the same age/skew limits.
        stamps = np.array([self.samples[k].header.stamp.sec*1_000_000_000+
                           self.samples[k].header.stamp.nanosec for k in needed], dtype=np.int64)
        max_age_ns = round(self.p('max_age_s')*1_000_000_000)
        max_skew_ns = round(self.p('max_skew_s')*1_000_000_000)
        if (np.max(self.sim_ns-stamps) > max_age_ns or
                np.max(stamps-self.sim_ns) > max_skew_ns or
                np.ptp(stamps) > max_skew_ns or
                max(wall-self.received[k] for k in needed) > self.p('heartbeat_timeout_s')):
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


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',choices=['leader','follower'],required=True)
    parser.add_argument('--run-directory',type=Path,required=True)
    a=parser.parse_args()
    if (os.getenv('ROS_LOCALHOST_ONLY')!='1'
            or not os.getenv('IGN_PARTITION','').startswith('amir_cp9_round_trip_')):
        raise ValueError('isolated round-trip simulation required')
    folder=a.run_directory.resolve()
    rclpy.init(args=[])
    node=PreciseSafetyGate(namespace='/'+a.robot,parameter_overrides=[
        Parameter('use_sim_time',value=True),
        Parameter('urdf_file',value=str(folder/(a.robot+'.urdf'))),
        Parameter('profile_file',value=str(folder/(a.robot+'_safety.json')))],
        cli_args=['--ros-args','-r','rover_twist:=interlock/base_request',
                  '-r','arm_controller/joint_trajectory:=interlock/arm_request'])
    try:rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':
    main()
