#!/usr/bin/env python3
"""Verify the read-only preprocessor with actual Gazebo data and a paused bridge."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--certificate', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--finish-file', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    log = (a.output/'node.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'amir_compliance_control',
        'wrench_preprocessor.launch.py', 'urdf_file:='+str(a.urdf.resolve()),
        'calibration_file:='+str(a.certificate.resolve())],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('cp2_preprocessor_verifier')
    states, values, raw = [], [], []
    subscriptions = [node.create_subscription(String, '/follower/wrench_status',
        lambda m: states.append(m.data), 10),
        node.create_subscription(WrenchStamped, '/follower/wrench_filtered',
            values.append, qos_profile_sensor_data),
        node.create_subscription(WrenchStamped, '/follower/wrench_at_grasp_raw',
            raw.append, qos_profile_sensor_data)]
    tare = node.create_client(Trigger, '/follower/tare_wrench')
    bridge_pid = None
    report = {'passed': False}

    def wait(predicate, timeout=30.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError('preprocessor exited')
            if time.monotonic() > deadline:
                raise RuntimeError('preprocessor state timeout; last='+str(states[-5:]))
            rclpy.spin_once(node, timeout_sec=.01)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)

    def request_tare():
        deadline = time.monotonic()+30.
        while time.monotonic() < deadline:
            future = tare.call_async(Trigger.Request())
            wait(future.done)
            if future.result().success:
                return
            spin(.5)
        raise RuntimeError('tare could not acquire stationary fresh samples')

    try:
        wait(lambda: len(raw) > 100 and tare.service_is_ready(), 60.)
        request_tare()
        wait(lambda: len(values) >= 30)
        vectors = np.array([[m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                             m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z] for m in values[-30:]])
        if np.max(np.abs(vectors[:, :3])) > .01 or np.max(np.abs(vectors[:, 3:])) > .001:
            raise RuntimeError('stationary filtered wrench residual exceeds test tolerance')
        report['stationary_peak_residual'] = np.max(np.abs(vectors), axis=0).tolist()
        # Select only this isolated domain's follower F/T bridge; always resume it.
        candidates = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv = (entry/'cmdline').read_bytes().split(b'\0')
                env = (entry/'environ').read_bytes().split(b'\0')
                if (b'/follower/ft_sensor@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench' in argv
                        and ('ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID']).encode() in env
                        and argv[0].endswith(b'/parameter_bridge')):
                    candidates.append(int(entry.name))
            except (OSError, ProcessLookupError):
                continue
        if len(candidates) != 1:
            raise RuntimeError('cannot uniquely identify this test session F/T bridge')
        bridge_pid = candidates[0]
        os.kill(bridge_pid, signal.SIGSTOP)
        wait(lambda: bool(states) and states[-1].startswith('INVALID'), 5.)
        spin(.1)
        stopped_count = len(values)
        spin(.2)
        if len(values) != stopped_count:
            raise RuntimeError('filtered output continued during sensor dropout')
        report['dropout_detected'] = True
        os.kill(bridge_pid, signal.SIGCONT)
        bridge_pid = None
        wait(lambda: states[-1] == 'UNTARED')
        spin(.2)
        if len(values) != stopped_count:
            raise RuntimeError('sensor recovery reused invalidated bias')
        report['recovery_requires_tare'] = True
        request_tare()
        wait(lambda: len(values) > stopped_count+20)
        wait(lambda: ('wrench_preprocessor', '/follower') in node.get_node_names_and_namespaces())
        inputs = node.get_subscriber_names_and_types_by_node('wrench_preprocessor', '/follower')
        report['subscriptions'] = inputs
        if {t for t, _ in inputs} != {'/clock', '/follower/joint_states', '/follower/ft_sensor',
                                      '/follower/mecanum_drive_controller/odometry'}:
            raise RuntimeError('local input boundary violated')
        report['passed'] = True
        print(json.dumps(report, indent=2), flush=True)
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        if bridge_pid is not None:
            os.kill(bridge_pid, signal.SIGCONT)
        (a.output/'result.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        a.finish_file.touch()


if __name__ == '__main__':
    main()
