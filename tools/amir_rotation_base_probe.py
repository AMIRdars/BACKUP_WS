"""Fresh positive/negative x/y/yaw base probes in an ungrasped Fortress scene.

One base writer, sequential robots, measured model limits, source-time CSV.
This evaluator may read GT; it is never a follower transport controller.
"""
import argparse
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
import yaml

from amir_rotation_base_metrology import AXES, analyze, calibration_limits
from amir_rotation_base_alignment import select_source_pair
from amir_cp10_base_speed_probe import body_delta, reference
from amir_cp8r3_process import reap_partition


ROBOTS = ('leader', 'follower')
WORKSPACE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--limits-policy', type=Path, required=True)
    parser.add_argument('--leader-limits-policy', type=Path, required=True)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--hold-s', type=float, required=True)
    args = parser.parse_args()
    if (os.getenv('ROS_DISTRO') != 'humble' or os.getenv('ROS_LOCALHOST_ONLY') != '1'
            or not os.getenv('ROS_DOMAIN_ID')
            or not os.getenv('IGN_PARTITION', '').startswith('amir_cp8r3_base_metrology_')):
        parser.error('explicit isolated Humble domain and metrology partition required')
    if not math.isfinite(args.hold_s) or args.hold_s < 2.:
        parser.error('at least two seconds of steady measurement required')
    policy = yaml.safe_load(args.limits_policy.read_text())
    params = next(iter(policy.values()))['ros__parameters']
    leader = next(iter(yaml.safe_load(args.leader_limits_policy.read_text()).values()))['ros__parameters']
    try:
        speeds, accelerations = calibration_limits(params, leader)
    except (ValueError, KeyError) as exc:
        parser.error(str(exc))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    generated = output/'generated'
    summary = dict(passed=False, scope='simulation-only independent ungrasped base metrology',
        transport_qualified=False, precision_qualified=False, argv=sys.argv,
        limits_policy=str(args.limits_policy.resolve()),
        limits_policy_sha256=hashlib.sha256(args.limits_policy.read_bytes()).hexdigest(),
        leader_limits_policy=str(args.leader_limits_policy.resolve()),
        leader_limits_policy_sha256=hashlib.sha256(args.leader_limits_policy.read_bytes()).hexdigest(),
        speeds=speeds.tolist(), accelerations=accelerations.tolist(), hold_s=args.hold_s,
        ROS_DOMAIN_ID=os.getenv('ROS_DOMAIN_ID'), IGN_PARTITION=os.getenv('IGN_PARTITION'))
    files = [Path(__file__), *[Path(__file__).with_name(name) for name in (
        'amir_rotation_base_metrology.py', 'amir_rotation_base_fixture.py', 'amir_rotation_base_alignment.py',
        'amir_rotation_integration_audit.py', 'amir_cp10_base_speed_probe.py', 'amir_cp8r3_process.py')]]
    manifest = {}
    for file in files:
        target = output/'source_snapshot'/file.relative_to(WORKSPACE)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(file.read_bytes())
        manifest[str(file)] = hashlib.sha256(file.read_bytes()).hexdigest()
    (output/'source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    processes, logs, data, received, results, final_controllers = [], [], {}, {}, [], {}
    histories = {(r, source): deque(maxlen=32) for r in ROBOTS for source in ('encoder', 'gt')}
    aligned_feedback = {}
    now, clock_wall = [0.], [time.monotonic()]

    def start(name, command):
        stream = (output/(name+'.log')).open('w')
        logs.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))

    def interrupt(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('rotation_independent_base_metrology')
    latest_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs, lists, switches, subscriptions, wheel_limits = {}, {}, {}, [], {}
    for robot in ROBOTS:
        pubs[robot] = node.create_publisher(Twist, '/'+robot+'/rover_twist', 1)
        lists[robot] = node.create_client(ListControllers, '/'+robot+'/controller_manager/list_controllers')
        switches[robot] = node.create_client(SwitchController, '/'+robot+'/controller_manager/switch_controller')
        for key, topic, typ in [('joints', '/'+robot+'/joint_states', JointState),
            ('encoder', '/'+robot+'/mecanum_drive_controller/odometry', Odometry),
            ('gt', '/experiment/'+robot+'/ground_truth/odom', Odometry)]:
            def receive(msg, k=(robot, key)):
                data[k], received[k] = msg, time.monotonic()
                if k in histories:
                    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    histories[k].append((stamp, msg))
            subscriptions.append(node.create_subscription(typ, topic, receive, latest_qos))

    def clock(msg):
        now[0], clock_wall[0] = msg.clock.sec+msg.clock.nanosec*1e-9, time.monotonic()

    subscriptions.append(node.create_subscription(Clock, '/clock', clock, latest_qos))
    stream = (output/'samples.csv').open('w', newline='')
    fields = ['case', 'robot', 'axis', 'sign', 'phase', 'sim_time_s', 'command',
              'encoder_stamp_s', 'gt_stamp_s', 'encoder_x', 'encoder_y', 'encoder_yaw',
              'gt_x', 'gt_y', 'gt_yaw', 'peer_GT_xy_motion_m', 'peer_GT_yaw_motion_rad',
              'leader_wheel_max_rad_s', 'follower_wheel_max_rad_s']
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()

    def zero():
        for pub in pubs.values():
            pub.publish(Twist())

    def wait(predicate, timeout=90.):
        deadline = time.monotonic()+timeout
        while not predicate():
            zero()
            rclpy.spin_once(node, timeout_sec=.005)
            if time.monotonic() > deadline:
                raise RuntimeError('ROS startup/service wait timed out')
            if any(p.poll() is not None for _, p in processes):
                raise RuntimeError('owned scene process exited')

    def pose(robot, source, message=None):
        p = (data[robot, source] if message is None else message).pose.pose
        q = p.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm) or abs(norm-1.) > 1e-5:
            raise RuntimeError('invalid source orientation')
        return [p.position.x, p.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    def stamp(robot, source, message=None):
        t = (data[robot, source] if message is None else message).header.stamp
        return t.sec+t.nanosec*1e-9

    def wheels(robot):
        msg = data[robot, 'joints']
        actual = dict(zip(msg.name, msg.velocity))
        values = [actual[name] for name in wheel_limits[robot]]
        if any(not math.isfinite(v) or abs(v) > wheel_limits[robot][n]*1.01
               for n, v in zip(wheel_limits[robot], values)):
            raise RuntimeError('actual wheel velocity exceeds model limit')
        return max(map(abs, values))

    def graph():
        snapshot = dict(sim_time_s=now[0], valid=True, topics={})
        for robot in ROBOTS:
            for topic, owner, namespace in (
                ('rover_twist', node.get_name(), '/'),
                ('mecanum_drive_controller/reference_unstamped', 'rover_twist_relay_ign', '/'+robot)):
                info = node.get_publishers_info_by_topic('/'+robot+'/'+topic)
                snapshot['topics']['/'+robot+'/'+topic] = [dict(node_name=p.node_name,
                    node_namespace=p.node_namespace) for p in info]
                if len(info) != 1 or info[0].node_name != owner or info[0].node_namespace != namespace:
                    snapshot['valid'] = False
                    (output/'graph.json').write_text(json.dumps(snapshot, indent=2)+'\n')
                    raise RuntimeError('single base writer graph not verified: '+robot+'/'+topic)
        (output/'graph.json').write_text(json.dumps(snapshot, indent=2)+'\n')

    def health():
        wall = time.monotonic()
        if wall-clock_wall[0] > 1. or any(p.poll() is not None for _, p in processes):
            raise RuntimeError('clock/owned process watchdog')
        for robot in ROBOTS:
            for source, frame in (('encoder', robot+'/odom'), ('gt', 'world')):
                msg = data[robot, source]
                if msg.header.frame_id != frame or msg.child_frame_id != robot+'/base_footprint':
                    raise RuntimeError('base feedback frame differs from verified controller/model')
            for key in ('joints', 'encoder', 'gt'):
                age = now[0]-stamp(robot, key)
                if wall-received.get((robot, key), 0.) > 1. or not -.02 <= age <= .05+1e-9:
                    raise RuntimeError(robot+'/'+key+': stale/future feedback')
            for source in ('encoder', 'gt'):
                if not np.isfinite(pose(robot, source)).all():
                    raise RuntimeError('nonfinite base pose')
            try:
                aligned_feedback[robot] = select_source_pair(histories[robot, 'encoder'], histories[robot, 'gt'], now[0])
            except ValueError as exc:
                raise RuntimeError(robot+': '+str(exc)) from exc
            for source, frame in (('encoder', robot+'/odom'), ('gt', 'world')):
                message = aligned_feedback[robot][source][1]
                if message.header.frame_id != frame or message.child_frame_id != robot+'/base_footprint':
                    raise RuntimeError('aligned source feedback frame differs from verified controller/model')
            wheels(robot)

    def interval(seconds, robot=None, axis=None, sign=0, speed=0., acceleration=1., peer_origin=None, record_only=False):
        begin, wall, tick, graph_time = now[0], time.monotonic(), -1., -1.
        ramp = abs(speed)/acceleration
        active = robot is not None and not record_only
        while now[0]-begin < seconds:
            rclpy.spin_once(node, timeout_sec=.002)
            health()
            t = now[0]-begin
            if t-tick < .01:
                if time.monotonic()-wall > max(30., seconds*20.):
                    raise RuntimeError('simulation progress timeout')
                continue
            tick = t
            if t-graph_time >= 1.:
                graph(); graph_time = t
            command = reference(t, speed, acceleration, args.hold_s) if active else 0.
            for name, pub in pubs.items():
                msg = Twist()
                if name == robot:
                    if axis == 'yaw':
                        msg.angular.z = command
                    else:
                        setattr(msg.linear, axis, command)
                pub.publish(msg)
            if robot is not None:
                peer = next(r for r in ROBOTS if r != robot)
                delta = body_delta(peer_origin, pose(peer, 'gt'))
                if math.hypot(*delta[:2]) >= .003 or abs(delta[2]) >= .01:
                    raise RuntimeError('uncommanded peer moved beyond existing CP-1 bound')
                phase = ('ramp' if t < ramp else ('plateau' if t <= ramp+args.hold_s else 'deceleration')) if active else 'stop'
                row = dict(case=robot+'_'+axis+'_'+str(sign), robot=robot, axis=axis, sign=sign,
                    phase=phase, sim_time_s=now[0], command=command,
                    encoder_stamp_s=aligned_feedback[robot]['encoder'][0], gt_stamp_s=aligned_feedback[robot]['gt'][0],
                    peer_GT_xy_motion_m=math.hypot(*delta[:2]), peer_GT_yaw_motion_rad=delta[2],
                    leader_wheel_max_rad_s=wheels('leader'), follower_wheel_max_rad_s=wheels('follower'))
                for source in ('encoder', 'gt'):
                    row.update({source+'_'+k: v for k, v in zip(AXES, pose(robot, source, aligned_feedback[robot][source][1]))})
                # Drop ramp transients from steady fitting, retaining every row in CSV.
                if phase == 'plateau' and (t < ramp+.5 or t > ramp+args.hold_s-.5):
                    row['phase'] = 'plateau_edge'
                writer.writerow(row)
        zero()
        stream.flush()

    def stopped():
        return all(math.hypot(data[r, 'encoder'].twist.twist.linear.x,
                             data[r, 'encoder'].twist.twist.linear.y) < .0001
                   and abs(data[r, 'encoder'].twist.twist.angular.z) < .0001
                   and wheels(r) < .02 for r in ROBOTS)

    try:
        command = [sys.executable, str(Path(__file__).with_name('amir_rotation_base_fixture.py')),
            '--output', str(generated), '--reference', str(args.reference)]
        if args.candidate:
            command += ['--candidate', str(args.candidate)]
        start('scene', command)
        start('evaluation', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_evaluation.launch.py'])
        start('placement_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/cooperative_transport/set_pose@ros_gz_interfaces/srv/SetEntityPose'])
        wait(lambda: now[0] > 0. and all((r, k) in data for r in ROBOTS for k in ('joints', 'encoder', 'gt')))
        def spawners_finished():
            lines = (output/'scene.log').read_text().splitlines()
            return all(any('['+r+'.spawner_'+name+']' in line and 'Configured and activated' in line
                for line in lines) for r in ROBOTS for name in (
                    'joint_state_broadcaster', 'arm_controller', 'mecanum_drive_controller', 'gripper_controller'))
        wait(spawners_finished, 180.)
        for robot in ROBOTS:
            wait(lists[robot].service_is_ready)
            deadline = time.monotonic()+90.
            while True:
                f = lists[robot].call_async(ListControllers.Request()); wait(f.done)
                controllers = {c.name: c.state for c in f.result().controller}
                if all(controllers.get(c) == 'active' for c in (
                        'joint_state_broadcaster', 'arm_controller', 'gripper_controller', 'mecanum_drive_controller')):
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('four expected active controllers required')
            root = ET.parse(generated/(robot+'.urdf'))
            p = next(iter(yaml.safe_load((generated/(robot+'_base_controller.yaml')).read_text()).values()))['ros__parameters']
            if p['base_frame_id'] != robot+'/base_footprint' or p['use_stamped_vel']:
                raise RuntimeError('unexpected existing base command interface/frame')
            radius = p['kinematics']['wheels_radius']
            lever = p['kinematics']['sum_of_robot_center_projection_on_X_Y_axis']
            wheel_limits[robot] = {}
            for key in ('front_left_wheel_command_joint_name', 'front_right_wheel_command_joint_name',
                        'rear_left_wheel_command_joint_name', 'rear_right_wheel_command_joint_name'):
                name = p[key]
                interface = root.find("ros2_control/joint[@name='"+name+"']/command_interface[@name='velocity']")
                limit = min(float(root.find("joint[@name='"+name+"']/limit").get('velocity')),
                    float(interface.find("param[@name='max']").text), -float(interface.find("param[@name='min']").text))
                if max(speeds[0], speeds[1], lever*speeds[2])/radius > limit:
                    raise RuntimeError('requested independent axis speed exceeds actual wheel bound')
                wheel_limits[robot][name] = limit
        interval(2.)
        graph()
        if not stopped():
            raise RuntimeError('zero motion required before ungrasped initial placement')
        placer = node.create_client(SetEntityPose, '/world/cooperative_transport/set_pose')
        wait(placer.service_is_ready)
        for robot, x in zip(ROBOTS, (-10., 10.)):
            request = SetEntityPose.Request()
            request.entity.name, request.entity.type = robot, Entity.MODEL
            request.pose = data[robot, 'gt'].pose.pose
            request.pose.position.x = x
            future = placer.call_async(request); wait(future.done)
            if not future.result().success:
                raise RuntimeError('stationary simulation setup placement rejected')
        node.destroy_client(placer)
        interval(2.)
        if not stopped() or any(abs(pose(r, 'gt')[0]-x) > .1 for r, x in zip(ROBOTS, (-10., 10.))):
            raise RuntimeError('separated stationary ungrasped placement not confirmed')
        summary['initial_placement'] = {r: pose(r, 'gt') for r in ROBOTS}
        for robot in ROBOTS:
            peer = next(r for r in ROBOTS if r != robot)
            for index, axis in enumerate(AXES):
                for sign in (-1, 1):
                    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in manifest.items()):
                        raise RuntimeError('probe implementation changed during run')
                    begin = {r: {s: pose(r, s) for s in ('encoder', 'gt')} for r in ROBOTS}
                    duration = 2*speeds[index]/accelerations[index]+args.hold_s
                    print('START '+robot+'/'+axis+'/'+str(sign), flush=True)
                    interval(duration, robot, axis, sign, sign*speeds[index], accelerations[index], begin[peer]['gt'])
                    interval(1., robot, axis, sign, peer_origin=begin[peer]['gt'], record_only=True)
                    if not stopped():
                        raise RuntimeError('actual encoder/wheel stop not confirmed')
                    results.append(dict(robot=robot, axis=axis, sign=sign,
                        movement_body={r: {s: body_delta(begin[r][s], pose(r, s)) for s in ('encoder', 'gt')} for r in ROBOTS},
                        stopped_wheel_rad_s={r: wheels(r) for r in ROBOTS}))
                    (output/'motion.json').write_text(json.dumps(results, indent=2)+'\n')
        with (output/'samples.csv').open() as rows:
            report = analyze(list(csv.DictReader(rows)))
        (output/'metrology.json').write_text(json.dumps(report, indent=2)+'\n')
        summary['passed'] = True  # Coverage/health/stop only, never a precision threshold.
    except BaseException as exc:
        summary['reason'] = type(exc).__name__+': '+str(exc)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        zero()
        try:
            begin = time.monotonic()
            while time.monotonic()-begin < .5:
                zero(); rclpy.spin_once(node, timeout_sec=.005)
            summary['stopped_outputs_confirmed'] = bool(all(r in wheel_limits for r in ROBOTS) and stopped())
            for robot in ROBOTS:
                if switches[robot].service_is_ready():
                    request = SwitchController.Request(deactivate_controllers=['arm_controller', 'mecanum_drive_controller'],
                        strictness=SwitchController.Request.BEST_EFFORT)
                    future = switches[robot].call_async(request); wait(future.done, 10.)
                    future = lists[robot].call_async(ListControllers.Request()); wait(future.done, 10.)
                    final_controllers[robot] = {c.name: c.state for c in future.result().controller}
            summary['final_controllers'] = final_controllers
            if (not summary['stopped_outputs_confirmed'] or not all(
                    final_controllers.get(r, {}).get(c) == 'inactive'
                    for r in ROBOTS for c in ('arm_controller', 'mecanum_drive_controller'))):
                raise RuntimeError('stop/controller deactivation not confirmed')
        except BaseException as exc:
            summary.update(passed=False, cleanup_error=repr(exc))
        node.destroy_node(); rclpy.shutdown()
        for name, process in reversed(processes):
            for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                break
        summary['owned_process_roots'] = {n: dict(pid=p.pid, returncode=p.poll()) for n, p in processes}
        cleanup = reap_partition(summary['IGN_PARTITION'], summary['ROS_DOMAIN_ID'])
        (output/'process_cleanup.json').write_text(json.dumps(cleanup, indent=2)+'\n')
        stream.close()
        for log in logs:
            log.close()
        summary['completed_cases'] = len(results)
        (output/'result.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
