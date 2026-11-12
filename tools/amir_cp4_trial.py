#!/usr/bin/env python3
"""CP-4 evaluator: send a goal only to leader; never publish motion references."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from amir_compliance_control.wrench import LocalModel, at_grasp
from amir_compliance_control.transport import straight_reference as reference


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--speed', type=float, default=.000025)
    p.add_argument('--distance', type=float, default=.000025)
    p.add_argument('--acceleration', type=float, default=.001)
    p.add_argument('--fault', choices=['none', 'dropout', 'supervisor', 'clock'], default='none')
    p.add_argument('--weld-cfm', type=float, default=0.)
    a = p.parse_args()
    if (not all(math.isfinite(v) for v in (a.speed, a.distance, a.acceleration, a.weld_cfm))
            or min(a.speed, a.distance, a.acceleration) <= 0):
        raise ValueError('positive experiment trajectory settings required')
    a.output = a.output.resolve()
    a.run_directory = a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'command.json').write_text(json.dumps({'argv': __import__('sys').argv,
        'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION'),
        'condition': 'CP4_LEADER_GOAL_LOCAL_FOLLOWER_HOLD'}, indent=2))
    processes, logs, suspended = [], [], []
    def start(name, argv):
        file = (a.output/(name+'.log')).open('w')
        logs.append(file)
        process = subprocess.Popen(argv, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp2_constraint.launch.py',
                    'joint_load_sensors:=true', 'weld_cfm:='+str(a.weld_cfm), 'run_directory:='+str(a.run_directory)])
    rclpy.init(args=[])
    node = rclpy.create_node('cp4_leader_only_evaluator')
    robots = ('leader', 'follower')
    now, latest, poses, statuses, debug = [0.], {}, {}, {}, {}
    attach, subscriptions, endpoint_states = {}, [], {}
    goal_pub = node.create_publisher(Vector3Stamped, '/leader/transport_goal', 1)
    actual_commands, requested, motion_origins = {}, {}, {}
    phase, last_record = ['startup'], [0.]
    rows, raw_log = [], (a.output/'samples.jsonl').open('w', buffering=1)
    models = {}
    output_log = (a.output/'command_outputs.jsonl').open('w', buffering=1)

    def payload_pose(msg):
        for t in msg.transforms:
            if t.child_frame_id in (*robots, 'cooperative_payload'):
                poses[t.child_frame_id] = t.transform

    def output(robot, msg):
        actual_commands[robot] = msg.linear.x
        output_log.write(json.dumps({'wall_time': time.monotonic(), 'sim_time': now[0],
            'robot': robot, 'vx': msg.linear.x, 'vy': msg.linear.y, 'wz': msg.angular.z})+'\n')

    for robot in robots:
        subscriptions.append(node.create_subscription(String, '/'+robot+'/transport/state',
            lambda m, r=robot: endpoint_states.__setitem__(r, json.loads(m.data)), 10))
        subscriptions.append(node.create_subscription(TwistStamped, '/'+robot+'/base_cmd',
            lambda m, r=robot: requested.__setitem__(r, m.twist.linear.x), 10))
        attach[robot] = node.create_publisher(Empty, '/'+robot+'/grasp/attach', 1)
        for key, suffix, typ in [('joints', 'joint_states', JointState), ('wrench', 'ft_sensor', WrenchStamped),
            ('encoder', 'mecanum_drive_controller/odometry', Odometry), ('grasp', 'grasp/state', String)]:
            subscriptions.append(node.create_subscription(typ, '/'+robot+'/'+suffix,
                lambda m, r=robot, k=key: latest.__setitem__((r, k), m), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(String, '/'+robot+'/safety_debug',
            lambda m, r=robot: debug.__setitem__(r, json.loads(m.data)), 10))
        subscriptions.append(node.create_subscription(Twist, '/'+robot+'/rover_twist',
            lambda m, r=robot: output(r, m), 10))
    subscriptions += [node.create_subscription(Clock, '/clock', lambda m: now.__setitem__(0,
        m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', payload_pose, qos_profile_sensor_data),
        node.create_subscription(String, '/transport/safety_status', lambda m: statuses.update(json.loads(m.data)), 10)]
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    control = node.create_client(ControlWorld, '/world/cooperative_transport/control')
    report = {'passed': False, 'condition': 'leader_only_goal_stationary_follower', 'fault_test': a.fault,
              'speed_m_s': a.speed, 'distance_m': a.distance,
              'acceleration_m_s2': a.acceleration, 'position_gain_s_inv': 1.}
    report['weld_cfm'] = a.weld_cfm

    def step():
        rclpy.spin_once(node, timeout_sec=.001)
        wall = time.monotonic()
        if wall-last_record[0] >= .05 and all((r, k) in latest for r in robots for k in ('joints', 'wrench', 'encoder')):
            last_record[0] = wall
            row = {'sim_time': now[0], 'wall_time': wall, 'phase': phase[0], 'fault': statuses.get('fault', '')}
            for robot in robots:
                msg = latest[(robot, 'joints')]
                mapping = dict(zip(msg.name, msg.position))
                q = [mapping[n] for n in models[robot].names] if robot in models else []
                for i, value in enumerate(q):
                    row[f'{robot}_q{i+1}_rad'] = value
                row[robot+'_requested_vx'] = requested.get(robot, 0.)
                row[robot+'_state'] = endpoint_states.get(robot, {}).get('state')
                row[robot+'_output_vx'] = actual_commands.get(robot, 0.)
                encoder = latest[(robot, 'encoder')]
                row[robot+'_encoder_x'] = encoder.pose.pose.position.x
                row[robot+'_encoder_vx'] = encoder.twist.twist.linear.x
                if robot in models:
                    grasp, sensor = models[robot].frames(q)
                    w = latest[(robot, 'wrench')].wrench
                    vector = at_grasp([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], sensor, grasp, 1.)
                    for key, value in zip(('fx', 'fy', 'fz', 'tx', 'ty', 'tz'), vector):
                        row[robot+'_'+key] = float(value)
                for i, value in enumerate(debug.get(robot, {}).get('reaction_torque_Nm') or []):
                    row[f'{robot}_joint_torque_{i+1}_Nm'] = value
                for i, value in enumerate(debug.get(robot, {}).get('peak_reaction_torque_Nm') or []):
                    row[f'{robot}_joint_peak_torque_{i+1}_Nm'] = value
                if robot in poses:
                    row[robot+'_gt_x'] = poses[robot].translation.x
            if 'cooperative_payload' in poses:
                t = poses['cooperative_payload']
                for k in 'xyz':
                    row['payload_'+k] = getattr(t.translation, k)
                for k in 'xyzw':
                    row['payload_q'+k] = getattr(t.rotation, k)
            rows.append(row)
            raw_log.write(json.dumps(row)+'\n')

    def wait(predicate, timeout=60., check_fault=False):
        deadline = time.monotonic()+timeout
        while not predicate():
            if check_fault and statuses.get('fault'):
                raise RuntimeError('shared stop: '+statuses['fault'])
            if time.monotonic() > deadline:
                raise RuntimeError('experiment readiness timeout; safety='+str(statuses)+' debug='+str(debug))
            step()

    def wall_wait(seconds):
        begin = time.monotonic()
        while time.monotonic()-begin < seconds:
            step()

    def sim_wait(seconds, check_fault=True):
        begin = now[0]
        wait(lambda: now[0]-begin >= seconds, max(30., seconds*15), check_fault=check_fault)

    def service(name, retry_discovery=False):
        client = node.create_client(Trigger, name)
        wait(client.service_is_ready, 10.)
        deadline = time.monotonic()+5.
        while True:
            future = client.call_async(Trigger.Request())
            wait(future.done, 10.)
            result = future.result()
            if (result.success or not retry_discovery or time.monotonic() > deadline
                    or not any(v in result.message for v in ('command writer ownership not confirmed',
                        'unverified publisher', 'waiting for local sensors and gate'))):
                break
            wall_wait(.2)
        node.destroy_client(client)
        with (a.output/'service_results.jsonl').open('a') as log:
            log.write(json.dumps({'service': name, 'success': result.success, 'message': result.message,
                                  'wall_time': time.monotonic(), 'sim_time': now[0]})+'\n')
        if not result.success:
            raise RuntimeError(name+': '+result.message)
        return result

    def find_process(argument):
        candidates = []
        for path in Path('/proc').iterdir():
            if not path.name.isdigit():
                continue
            try:
                argv = (path/'cmdline').read_bytes().split(b'\0')
                env = (path/'environ').read_bytes().split(b'\0')
                if (any(argument in v for v in argv) and
                        ('ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID']).encode() in env):
                    candidates.append(int(path.name))
            except OSError:
                pass
        if len(candidates) != 1:
            raise RuntimeError('cannot uniquely identify fault target: '+str(candidates))
        return candidates[0]

    def pause(value):
        req = ControlWorld.Request()
        req.world_control.pause = value
        future = control.call_async(req)
        wait(future.done, 5.)
        if not future.result().success:
            raise RuntimeError('world control failed')

    try:
        wait(lambda: now[0] > 0 and all((r, k) in latest for r in robots for k in ('joints', 'encoder', 'wrench')), 90.)
        for robot in robots:
            xml = (a.run_directory/(robot+'.urdf')).read_text()
            profile = json.loads((a.run_directory/(robot+'_safety.json')).read_text())
            if profile['urdf_sha256'] != hashlib.sha256(xml.encode()).hexdigest():
                raise RuntimeError('generated URDF does not match measured safety profile')
            if profile['world_sha256'] != hashlib.sha256((a.run_directory/'world.sdf').read_bytes()).hexdigest():
                raise RuntimeError('world physics differs from safety profile evidence')
            if profile['controller_yaml_sha256'] != hashlib.sha256((a.run_directory/(robot+'_controllers.yaml')).read_bytes()).hexdigest():
                raise RuntimeError('controller configuration differs from safety profile evidence')
            if profile['base_controller_yaml_sha256'] != hashlib.sha256((a.run_directory/(robot+'_base_controller.yaml')).read_bytes()).hexdigest():
                raise RuntimeError('base controller configuration differs from safety profile evidence')
            models[robot] = LocalModel(xml)
        start('safety', ['ros2', 'launch', 'amir_compliance_control', 'safety.launch.py', 'run_directory:='+str(a.run_directory)])
        bridge_args = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
        for robot in robots:
            bridge_args += [f'/{robot}/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                            f'/{robot}/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg']
        start('evaluation', bridge_args)
        wait(lambda: statuses.get('ready') and all((r, 'grasp') in latest for r in robots)
             and remover.service_is_ready() and control.service_is_ready(), 45.)
        start('transport', ['ros2', 'launch', 'amir_compliance_control', 'leader_only.launch.py',
            'run_directory:='+str(a.run_directory), 'max_distance_m:='+str(a.distance),
            'speed_m_s:='+str(a.speed), 'acceleration_m_s2:='+str(a.acceleration)])
        wait(lambda: all(endpoint_states.get(r, {}).get('local_ready') for r in robots), 20.)
        for robot in robots:
            service('/'+robot+'/transport/prepare', retry_discovery=True)
        service('/transport/safety/arm', retry_discovery=True)
        wait(lambda: statuses.get('armed'), 10., check_fault=True)
        phase[0] = 'local_home'
        wait(lambda: all(endpoint_states[r]['state'] == 'READY' for r in robots), 15., check_fault=True)
        phase[0] = 'attach_supported'
        for _ in range(15):
            for pub in attach.values():
                pub.publish(Empty())
            wall_wait(.05)
        wait(lambda: all(latest[(r, 'grasp')].data == 'attached' for r in robots), 10., check_fault=True)
        req = DeleteEntity.Request()
        req.entity.name, req.entity.type = 'grasp_support', Entity.MODEL
        f = remover.call_async(req)
        wait(f.done, 10., check_fault=True)
        if not f.result().success:
            raise RuntimeError('support removal failed')
        phase[0] = 'hold'
        sim_wait(2.)
        origin = {k: poses[k].translation.x for k in robots+('cooperative_payload',)}
        motion_origins.update({r: latest[(r, 'encoder')].pose.pose.position.x for r in robots})
        phase[0] = 'motion'
        goal = Vector3Stamped()
        goal.header.stamp, goal.header.frame_id = Time(seconds=now[0]).to_msg(), 'leader/base_footprint'
        goal.vector.x = a.distance
        (a.output/'goal.json').write_text(json.dumps({'topic': '/leader/transport_goal',
            'frame_id': goal.header.frame_id, 'stamp_s': now[0], 'relative_x_m': a.distance,
            'relative_y_m': 0., 'relative_z_m': 0.}, indent=2))
        goal_pub.publish(goal)
        wait(lambda: endpoint_states['leader']['accepted_goals'] == 1, 5., check_fault=True)
        duration = reference(0., a.distance, a.speed, a.acceleration)[2]
        if a.fault == 'none':
            wait(lambda: endpoint_states['leader']['completed_goals'] == 1, duration+6., check_fault=True)
            phase[0] = 'settle'
            sim_wait(.5)
            report['gt_distance_m'] = {k: poses[k].translation.x-origin[k] for k in origin}
            report['encoder_distance_m'] = {r: latest[(r, 'encoder')].pose.pose.position.x-motion_origins[r] for r in robots}
            if abs(report['encoder_distance_m']['leader']-a.distance) > .0000025:
                raise RuntimeError('leader encoder target did not converge')
            if endpoint_states['follower']['accepted_goals'] != 0:
                raise RuntimeError('follower unexpectedly received a goal')
        else:
            sim_wait(.25)
            phase[0] = 'fault_injected'
            injection = time.monotonic()
            if a.fault == 'dropout':
                pid = find_process(b'/follower/ft_sensor@geometry_msgs/msg/WrenchStamped[')
                os.kill(pid, signal.SIGSTOP)
                suspended.append(pid)
            elif a.fault == 'supervisor':
                os.kill(find_process(b'/lib/amir_compliance_control/safety_supervisor'), signal.SIGKILL)
            else:
                pause(True)
            wait(lambda: all(r in actual_commands and abs(actual_commands[r]) < 1e-12 for r in robots)
                 and all(not debug.get(r, {}).get('active', True) for r in robots), 2.)
            report['dual_zero_latency_wall_s'] = time.monotonic()-injection
            if report['dual_zero_latency_wall_s'] > .4:
                raise RuntimeError('dual stop exceeded experiment wall-time budget')
            if a.fault == 'dropout':
                os.kill(suspended.pop(), signal.SIGCONT)
            if a.fault == 'clock':
                pause(False)
            wall_wait(.5)
            if any(abs(actual_commands.get(r, 1.)) > 1e-12 for r in robots):
                raise RuntimeError('motion resumed automatically after fault recovery')
            if any(abs(latest[(r, 'encoder')].twist.twist.linear.x) > .0001 for r in robots):
                raise RuntimeError('wheel odometry did not confirm both bases stopped')
            report['no_automatic_restart'] = True
            report['gate_faults'] = {r: debug.get(r, {}).get('fault') for r in robots}
        if any(abs(r.get('follower_requested_vx', 0.)) > 1e-12 for r in rows):
            raise RuntimeError('CP-4 follower issued a nonzero motion request')
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        for pid in suspended:
            os.kill(pid, signal.SIGCONT)
        if a.fault == 'clock' and control.service_is_ready():
            try:
                pause(False)
            except Exception:
                pass
        if a.fault != 'supervisor':
            try:
                service('/transport/safety/stop')
                wall_wait(.15)
            except Exception:
                pass
        keys = sorted({k for row in rows for k in row})
        with (a.output/'timeseries.csv').open('w') as file:
            writer = csv.DictWriter(file, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        report['rows'] = len(rows)
        report['final_safety_state'] = statuses if a.fault != 'supervisor' else None
        report['last_received_safety_state'] = statuses
        report['endpoint_states'] = endpoint_states
        if rows:
            report['peak_joint_reaction_torque_Nm'] = {r: [max(row.get(f'{r}_joint_peak_torque_{i}_Nm', row.get(f'{r}_joint_torque_{i}_Nm', 0.)) for row in rows)
                                                         for i in range(1, 6)] for r in robots}
            hold_rows = [row for row in rows if row['phase'] == 'hold']
            move_rows = [row for row in rows if row['phase'] == 'motion']
            if hold_rows and move_rows:
                report['peak_wrench_change_force_N'] = {}
                for r in robots:
                    bias = np.mean([[row[r+'_'+key] for key in ('fx', 'fy', 'fz')] for row in hold_rows], axis=0)
                    forces = np.array([[row[r+'_'+key] for key in ('fx', 'fy', 'fz')] for row in move_rows])
                    report['peak_wrench_change_force_N'][r] = float(np.max(np.linalg.norm(forces-bias, axis=1)))
            motion = [row for row in rows if row['phase'] in ('motion', 'fault_injected')]
            report['request_onset_sim_s'] = {r: next((row['sim_time'] for row in motion if abs(row.get(r+'_requested_vx', 0.)) > .1*a.speed), None) for r in robots}
        graph = {}
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                graph[namespace.rstrip('/')+'/'+name] = {
                    'subscriptions': node.get_subscriber_names_and_types_by_node(name, namespace),
                    'publishers': node.get_publisher_names_and_types_by_node(name, namespace)}
            except Exception:
                pass
        (a.output/'graph.json').write_text(json.dumps(graph, indent=2))
        follower_inputs = [v[0] for key, value in graph.items() if key in ('/follower/follower_hold', '/follower/safety_gate') for v in value['subscriptions']]
        report['follower_input_boundary_passed'] = not any(t.startswith('/leader/') or 'goal' in t or 'ground_truth' in t or t.endswith('/tf') or '/world/' in t for t in follower_inputs)
        if not report['follower_input_boundary_passed']:
            report['passed'] = False
        (a.output/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        raw_log.close()
        output_log.close()
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for file in logs:
            file.close()


if __name__ == '__main__':
    main()
