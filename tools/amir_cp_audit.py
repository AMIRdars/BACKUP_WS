#!/usr/bin/env python3
"""Capture AMIR graph and samples without sending any robot command.

Run inside the intended ROS domain / Gazebo partition. The launched process
group is stopped on exit; never use a global pkill here.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--robots', nargs='+', default=['audit_robot'])
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--observe', type=float, default=5.)
    parser.add_argument('--finish-file', type=Path)
    parser.add_argument('--joint-loads', action='store_true')
    parser.add_argument('launch', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    command = args.launch
    if command and command[0] == '--':
        command = command[1:]
    (args.output/'command.json').write_text(json.dumps({
        'argv': command, 'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'),
        'IGN_PARTITION': os.getenv('IGN_PARTITION'), 'ROS_DISTRO': os.getenv('ROS_DISTRO'),
        'cwd': os.getcwd(), 'robots': args.robots}, indent=2))
    process = None
    log = (args.output/'launch.log').open('w')
    rclpy.init(args=[])
    node = rclpy.create_node('checkpoint_graph_audit')
    counts, first, latest, edges = Counter(), {}, {}, {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []

    def record(topic, msg):
        counts[topic] += 1
        data = clean(message_to_ordereddict(msg))
        first.setdefault(topic, data)
        latest[topic] = data
        if isinstance(msg, TFMessage):
            for t in msg.transforms:
                edges.setdefault(topic, {})[t.child_frame_id] = t.header.frame_id

    for robot in args.robots:
        streams = [('joint_states', JointState), ('odom', Odometry),
                            ('mecanum_drive_controller/odometry', Odometry),
                            ('ft_sensor', WrenchStamped), ('tf', TFMessage),
                            ('tf_static', TFMessage)]
        if args.joint_loads:
            streams += [('joint_load/Joint_'+str(i), WrenchStamped) for i in range(1, 5)]
        for suffix, typ in streams:
            topic = f'/{robot}/{suffix}'
            subscriptions.append(node.create_subscription(typ, topic,
                lambda msg, t=topic: record(t, msg),
                static_qos if suffix == 'tf_static' else qos))
    subscriptions.append(node.create_subscription(Clock, '/clock',
        lambda msg: record('/clock', msg), qos))
    clients = {r: node.create_client(ListControllers,
        f'/{r}/controller_manager/list_controllers') for r in args.robots}
    futures, controllers, last_request = {}, {}, {}
    result = {}
    try:
        if command:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        deadline = time.monotonic() + args.timeout
        ready_at = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for robot, client in clients.items():
                future = futures.get(robot)
                if future is not None and future.done():
                    try:
                        controllers[robot] = clean(message_to_ordereddict(future.result()))
                    except Exception as exc:
                        controllers[robot] = {'error': str(exc)}
                    del futures[robot]
                active = {c['name'] for c in controllers.get(robot, {}).get('controller', [])
                          if c['state'] == 'active'}
                expected = {'joint_state_broadcaster', 'arm_controller',
                            'mecanum_drive_controller', 'gripper_controller'}
                # Do not contend with controller spawners during Gazebo startup.
                observed = all(counts[f'/{r}/mecanum_drive_controller/odometry'] > 0
                               for r in args.robots)
                if (observed and not expected.issubset(active) and robot not in futures
                        and client.service_is_ready()
                        and time.monotonic()-last_request.get(robot, 0.) > 1.):
                    last_request[robot] = time.monotonic()
                    futures[robot] = client.call_async(ListControllers.Request())
            all_ready = all(len([c for c in controllers.get(r, {}).get('controller', [])
                                if c['state'] == 'active']) >= 4 and
                            counts[f'/{r}/joint_states'] > 0 and
                            counts[f'/{r}/mecanum_drive_controller/odometry'] > 0
                            for r in args.robots)
            if all_ready and ready_at is None:
                ready_at = time.monotonic()
            finished = (args.finish_file.exists() if args.finish_file else
                        ready_at and time.monotonic()-ready_at >= args.observe)
            if ready_at and finished:
                break
            if process and process.poll() is not None:
                break
        graph = {}
        for name, namespace in node.get_node_names_and_namespaces():
            full = namespace.rstrip('/')+'/'+name
            try:
                graph[full] = {
                    'subscriptions': node.get_subscriber_names_and_types_by_node(name, namespace),
                    'publishers': node.get_publisher_names_and_types_by_node(name, namespace),
                    'clients': node.get_client_names_and_types_by_node(name, namespace)}
            except Exception as exc:
                graph[full] = {'error': str(exc)}
        endpoints = {}
        for topic, types in node.get_topic_names_and_types():
            def info(endpoint):
                return {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
                        'topic_type': endpoint.topic_type}
            endpoints[topic] = {'types': types,
                'publishers': [info(x) for x in node.get_publishers_info_by_topic(topic)],
                'subscriptions': [info(x) for x in node.get_subscriptions_info_by_topic(topic)]}
        result = {'controllers_ready': ready_at is not None, 'controllers': controllers,
                  'counts': dict(counts), 'first': first, 'latest': latest,
                  'tf_edges': edges, 'nodes': graph, 'topics': endpoints,
                  'launch_returncode_before_shutdown': process.poll() if process else None}
        for name, argv in [('gazebo_topics', ['ign', 'topic', '-l']),
                           ('actions', ['ros2', 'action', 'list', '-t'])]:
            try:
                capture = subprocess.run(argv, text=True, capture_output=True, timeout=12)
                (args.output/(name+'.txt')).write_text(capture.stdout+capture.stderr)
            except subprocess.TimeoutExpired:
                (args.output/(name+'.txt')).write_text('COMMAND TIMED OUT\n')
        (args.output/'runtime.json').write_text(json.dumps(clean(result), indent=2, allow_nan=False))
        print(json.dumps({'output': str(args.output), 'controllers_ready': ready_at is not None,
                          'counts': dict(counts)}, indent=2), flush=True)
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result.get('controllers_ready') else 1


if __name__ == '__main__':
    raise SystemExit(main())
