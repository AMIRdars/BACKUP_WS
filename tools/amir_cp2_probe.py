#!/usr/bin/env python3
"""Local sensor calibration evaluator for the ungrasped CP-2 fixture.

Excitation amplitudes are identification inputs, not accepted load limits.
All transforms use joint states interpolated at the F/T timestamp.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from amir_follower_control.core import ArmURDF, interpolate_joints, transform_wrench


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--finish-file', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    xml = args.urdf.read_text()
    arm = ArmURDF(xml)
    rclpy.init(args=[])
    node = rclpy.create_node('cp2_local_sensor_probe')
    pub = node.create_publisher(Wrench, '/experiment/follower/calibration_wrench', 1)
    history, pending, samples = deque(maxlen=200), deque(maxlen=20), []
    now, header = [0.], [None]
    phase = {'name': 'waiting', 'command': [0.]*6}
    raw_log = (args.output/'samples.jsonl').open('w', buffering=1)

    def process():
        while pending and history and stamp(pending[0]) <= history[-1][0]:
            msg = pending.popleft()
            try:
                q = interpolate_joints(history, stamp(msg))
            except ValueError:
                continue
            if header[0] is None:
                header[0] = msg.header.frame_id
            if msg.header.frame_id != header[0]:
                raise RuntimeError('sensor frame changed')
            raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
            t, sensor, _ = arm.evaluate(q)
            wrench = transform_wrench(raw, sensor, t[:3, 3], 1.)
            entry = {'stamp': stamp(msg), 'phase': phase['name'], 'command': phase['command'],
                     'header_frame': header[0], 'q': q.tolist(), 'raw': raw.tolist(),
                     'base_at_tcp_sign_plus': wrench.tolist()}
            if not np.all(np.isfinite(wrench)):
                raise RuntimeError('nonfinite wrench')
            samples.append(entry)
            raw_log.write(json.dumps(entry)+'\n')

    def joint(msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values for j in arm.names):
            history.append((stamp(msg), np.array([values[j] for j in arm.names])))
            process()

    def wrench(msg):
        pending.append(msg)
        process()

    subscriptions = [node.create_subscription(JointState, '/follower/joint_states', joint, qos_profile_sensor_data),
        node.create_subscription(WrenchStamped, '/follower/ft_sensor', wrench, qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', lambda m: now.__setitem__(0,
            m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)]

    def run(name, command, duration):
        phase['name'], phase['command'] = name, command
        begin, wall, last_publish = now[0], time.monotonic(), 0.
        selected = len(samples)
        while now[0]-begin < duration:
            if time.monotonic()-wall > 30.:
                raise RuntimeError('clock stalled or too slow')
            if time.monotonic()-last_publish > .025:
                msg = Wrench()
                (msg.force.x, msg.force.y, msg.force.z,
                 msg.torque.x, msg.torque.y, msg.torque.z) = map(float, command)
                pub.publish(msg)
                last_publish = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.005)
        subset = [s for s in samples[selected:] if s['stamp'] > begin+duration*.5]
        if len(subset) < 20:
            raise RuntimeError(f'{name}: insufficient F/T samples')
        return np.mean([s['base_at_tcp_sign_plus'] for s in subset], axis=0)

    results, certificate = [], {}
    try:
        deadline = time.monotonic()+70
        while len(samples) < 100 or pub.get_subscription_count() < 1:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic() > deadline:
                raise RuntimeError('local F/T data or calibration bridge missing')
        run('settle', [0.]*6, 2.)
        for index, name in enumerate(('fx', 'fy', 'fz', 'tx', 'ty', 'tz')):
            for direction in (1., -1.):
                base = run('bias_'+name+str(direction), [0.]*6, 1.)
                command = [0.]*6
                command[index] = direction*(.5 if index < 3 else .05)
                measured = run(name+str(direction), command, 1.)-base
                gain = measured[index]/command[index]
                other = measured-np.array(command)
                # Calibration accuracy criteria, not overload thresholds.
                passed = .8 < gain < 1.2 and np.max(np.abs(other[:3])) < .1 and np.max(np.abs(other[3:])) < .01
                results.append({'axis': name, 'command': command, 'measured_delta': measured.tolist(),
                                'gain_assuming_sign_plus': gain, 'passed': bool(passed)})
        run('final', [0.]*6, 1.)
        certificate = {'robot': 'follower', 'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
            'header_frame': header[0], 'sign': 1. if all(r['passed'] for r in results) else None,
            'verified_axes': sorted(set(r['axis'] for r in results if r['passed'])),
            'sensor_calibration_passed': all(r['passed'] for r in results),
            'grasp_verified': False, 'load_limits_verified': False,
            'sample_count': len(samples), 'tests': results}
        (args.output/'calibration.json').write_text(json.dumps(certificate, indent=2))
        print(json.dumps({'tests': len(results), 'passed': sum(r['passed'] for r in results),
                          'frame': header[0]}, indent=2), flush=True)
        if not certificate['sensor_calibration_passed']:
            raise RuntimeError('physical sensor calibration failed')
    finally:
        pub.publish(Wrench())
        raw_log.close()
        node.destroy_node()
        rclpy.shutdown()
        args.finish_file.touch()


if __name__ == '__main__':
    main()
