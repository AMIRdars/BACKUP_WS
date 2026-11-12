#!/usr/bin/env python3
"""Stage-A static support-removal experiment, guarded by model joint effort.

This evaluates fixed constraints, not finger friction or certified load safety.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, WrenchStamped
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from amir_follower_control.core import ArmURDF, transform_wrench


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--finish-file', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    log = (a.output/'evaluation_bridge.log').open('w')
    robot_names = ('leader', 'follower')
    command = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
        '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
    for robot in robot_names:
        command.extend([f'/{robot}/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                        f'/{robot}/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg'])
    bridge = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('cp2_constraint_evaluator')
    arms = {r: ArmURDF((a.run_directory/(r+'.urdf')).read_text()) for r in robot_names}
    latest, now, object_poses = {}, [0.], []
    joint_log = (a.output/'samples.jsonl').open('w', buffering=1)
    zero, attach, hold, actions, subscriptions = {}, {}, {}, {}, []
    phase = ['startup']

    def receive(robot, kind, msg):
        latest[(robot, kind)] = msg

    def pose(msg):
        for t in msg.transforms:
            if t.child_frame_id == 'cooperative_payload':
                object_poses.append([now[0], t.transform.translation.x,
                    t.transform.translation.y, t.transform.translation.z,
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w])

    for robot in robot_names:
        zero[robot] = node.create_publisher(Twist, f'/{robot}/rover_twist', 1)
        attach[robot] = node.create_publisher(Empty, f'/{robot}/grasp/attach', 1)
        hold[robot] = node.create_publisher(JointTrajectory, f'/{robot}/arm_controller/joint_trajectory', 1)
        actions[robot] = ActionClient(node, FollowJointTrajectory, f'/{robot}/arm_controller/follow_joint_trajectory')
        for kind, topic, typ in [('joints', 'joint_states', JointState),
            ('wrench', 'ft_sensor', WrenchStamped), ('grasp', 'grasp/state', String)]:
            subscriptions.append(node.create_subscription(typ, f'/{robot}/{topic}',
                lambda m, r=robot, k=kind: receive(r, k, m), qos_profile_sensor_data))
    subscriptions.extend([node.create_subscription(Clock, '/clock',
        lambda m: now.__setitem__(0, m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', pose, qos_profile_sensor_data)])
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    records = []
    report = {'passed': False, 'grasp_type': 'fixed_constraint', 'total_payload_mass_kg': .52}
    world = ET.parse(a.run_directory/'world.sdf')
    payload = world.find("world/model[@name='cooperative_payload']")
    report['total_payload_mass_kg'] = sum(float(e.text) for e in payload.findall('link/inertial/mass'))

    def state(robot):
        m, w = latest[(robot, 'joints')], latest[(robot, 'wrench')]
        t = w.header.stamp.sec+w.header.stamp.nanosec*1e-9
        jt = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if max(now[0]-t, now[0]-jt) > .05 or abs(t-jt) > .02:
            raise RuntimeError(robot+f': missing or skewed local F/T/joints: now={now[0]}, ft={t}, joints={jt}')
        mapping = dict(zip(m.name, m.position))
        q = np.array([mapping[n] for n in arms[robot].names])
        if not np.all(np.isfinite(q)) or np.any(q < arms[robot].lower) or np.any(q > arms[robot].upper):
            raise RuntimeError(robot+': invalid/limited joints')
        raw = np.array([w.wrench.force.x, w.wrench.force.y, w.wrench.force.z,
                        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z])
        if not np.all(np.isfinite(raw)):
            raise RuntimeError(robot+': invalid F/T')
        gravity = arms[robot].quasistatic_joint_load(q, np.zeros(6), 1.)
        load = arms[robot].quasistatic_joint_load(q, raw, 1.)
        # Conservative bound avoids assuming the other sensor's sign.
        bound = np.abs(load-gravity)+np.abs(gravity)
        if np.any(bound > arms[robot].effort):
            raise RuntimeError(robot+': model joint effort exceeded '+str(bound.tolist()))
        g, s, _ = arms[robot].evaluate(q)
        return q, transform_wrench(raw, s, g[:3, 3], 1.), bound

    def spin(seconds, attaching=False):
        start, wall = now[0], time.monotonic()
        last_observation = 0.
        while now[0]-start < seconds:
            if time.monotonic()-wall > max(30., seconds*10):
                raise RuntimeError('simulation clock stalled')
            # Drain subscriptions between bounded-rate model calculations.
            rclpy.spin_once(node, timeout_sec=.001)
            if time.monotonic()-last_observation < .05:
                continue
            last_observation = time.monotonic()
            for robot in robot_names:
                zero[robot].publish(Twist())
                if attaching:
                    attach[robot].publish(Empty())
                if all((robot, k) in latest for k in ('joints', 'wrench')):
                    q, wrench, bound = state(robot)
                    row = {'time': now[0], 'phase': phase[0], 'robot': robot,
                           'q': q.tolist(), 'wrench_at_tcp': wrench.tolist(), 'joint_load_bound': bound.tolist()}
                    records.append(row)
                    joint_log.write(json.dumps(row)+'\n')

    def wait(predicate, timeout=40.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('scene/action readiness timeout')
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        wait(lambda: all((r, k) in latest for r in robot_names for k in ('joints', 'wrench', 'grasp'))
             and object_poses and remover.service_is_ready() and all(c.server_is_ready() for c in actions.values()))
        for robot in robot_names:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(arms[robot].names)
            urdf = ET.parse(a.run_directory/(robot+'.urdf'))
            home = [float(urdf.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param").text)
                    for name in arms[robot].names]
            goal.trajectory.points = [JointTrajectoryPoint(positions=home)]
            goal.trajectory.points[0].time_from_start.sec = 3
            future = actions[robot].send_goal_async(goal)
            wait(future.done)
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('home action rejected')
            future = handle.get_result_async()
            wait(future.done)
            if future.result().status != 4:
                raise RuntimeError('home action failed')
        phase[0] = 'baseline'
        spin(2.)
        phase[0] = 'attach_supported'
        spin(2., attaching=True)
        if not all(latest[(r, 'grasp')].data == 'attached' for r in robot_names):
            raise RuntimeError('both physical constraints were not confirmed')
        request = DeleteEntity.Request()
        request.entity.name, request.entity.type = 'grasp_support', Entity.MODEL
        future = remover.call_async(request)
        wait(future.done)
        if not future.result().success:
            raise RuntimeError('support removal failed')
        phase[0] = 'unsupported_hold'
        begin = now[0]
        spin(10.)
        held = [p for p in object_poses if p[0] > begin+2.]
        if len(held) < 20:
            raise RuntimeError('insufficient payload evaluation samples')
        xyz = np.array([p[1:4] for p in held])
        drift = np.ptp(xyz, axis=0)
        if np.max(drift) > .001:
            raise RuntimeError('payload drift exceeds 1 mm static test tolerance: '+str(drift.tolist()))
        differences = {}
        for robot in robot_names:
            baseline = np.mean([r['wrench_at_tcp'][2] for r in records
                                if r['robot'] == robot and r['phase'] == 'baseline'], axis=0)
            loaded = np.mean([r['wrench_at_tcp'][2] for r in records
                              if r['robot'] == robot and r['phase'] == 'unsupported_hold' and r['time'] > begin+2.], axis=0)
            differences[robot] = float(loaded-baseline)
        expected_weight = report['total_payload_mass_kg']*9.81
        measured_weight = abs(sum(differences.values()))
        report.update(payload_drift_xyz_m=drift.tolist(), support_force_change_z_N=differences,
                      measured_payload_weight_N=measured_weight, expected_payload_weight_N=expected_weight)
        if abs(measured_weight-expected_weight) > .1*expected_weight:
            raise RuntimeError('reaction sum inconsistent with model weight')
        report['passed'] = True
        print(json.dumps(report, indent=2), flush=True)
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        for robot in robot_names:
            zero[robot].publish(Twist())
            if (robot, 'joints') in latest:
                mapping = dict(zip(latest[(robot, 'joints')].name, latest[(robot, 'joints')].position))
                trajectory = JointTrajectory(joint_names=list(arms[robot].names))
                point = JointTrajectoryPoint(positions=[mapping[n] for n in arms[robot].names])
                point.time_from_start.nanosec = 100000000
                trajectory.points = [point]
                hold[robot].publish(trajectory)
        (a.output/'result.json').write_text(json.dumps(report, indent=2))
        (a.output/'object_poses.json').write_text(json.dumps(object_poses))
        joint_log.close()
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(bridge.pid, signal.SIGINT)
        try:
            bridge.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(bridge.pid, signal.SIGKILL)
            bridge.wait()
        log.close()
        a.finish_file.touch()


if __name__ == '__main__':
    main()
