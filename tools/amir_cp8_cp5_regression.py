#!/usr/bin/env python3
"""CP-8 regression of CP-5; only extend the bounded DISARMED discovery retry.

Parent tools/amir_cp5_trial.py SHA256: e5d1855c64c6beac61522715bc930bb0efc6018e14961009a9578a846d146774
Motion, calibration, ownership validation, thresholds and assertions are unchanged.
"""
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
from geometry_msgs.msg import Twist, TwistStamped, Wrench, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.srv import ControlWorld
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from amir_compliance_control.wrench import LocalModel, at_grasp


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--calibration-file', type=Path, required=True)
    p.add_argument('--fault', choices=['none', 'dropout', 'preprocessor', 'clock'], default='none')
    a = p.parse_args()
    a.calibration_file = a.calibration_file.resolve()
    a.output = a.output.resolve()
    a.run_directory = a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'command.json').write_text(json.dumps({'argv': __import__('sys').argv,
        'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION'),
        'condition': 'CP5_ARM_ONLY_UNGRASPED_LOCAL_FORCE'}, indent=2))
    processes, logs, suspended = [], [], []
    def start(name, argv):
        file = (a.output/(name+'.log')).open('w')
        logs.append(file)
        process = subprocess.Popen(argv, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp5_sensors.launch.py', 'run_directory:='+str(a.run_directory)])
    rclpy.init(args=[])
    node = rclpy.create_node('cp5_arm_only_evaluator')
    robots = ('leader', 'follower')
    now, latest, poses, statuses, debug = [0.], {}, {}, {}, {}
    subscriptions, endpoint_states, adm_state, filter_state = [], {}, {}, ['']
    excitation = node.create_publisher(Wrench, '/experiment/follower/calibration_wrench', 1)
    applied, last_excitation = [0.]*6, [0.]
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
        for key, suffix, typ in [('joints', 'joint_states', JointState), ('wrench', 'ft_sensor', WrenchStamped),
            ('encoder', 'mecanum_drive_controller/odometry', Odometry)]:
            subscriptions.append(node.create_subscription(typ, '/'+robot+'/'+suffix,
                lambda m, r=robot, k=key: latest.__setitem__((r, k), m), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(String, '/'+robot+'/safety_debug',
            lambda m, r=robot: debug.__setitem__(r, json.loads(m.data)), 10))
        subscriptions.append(node.create_subscription(Twist, '/'+robot+'/rover_twist',
            lambda m, r=robot: output(r, m), 10))
    subscriptions += [node.create_subscription(String, '/follower/admittance/debug', lambda m: adm_state.update(json.loads(m.data)), 10),
        node.create_subscription(String, '/follower/wrench_status', lambda m: filter_state.__setitem__(0, m.data), 10),
        node.create_subscription(Clock, '/clock', lambda m: now.__setitem__(0,
        m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', payload_pose, qos_profile_sensor_data),
        node.create_subscription(String, '/transport/safety_status', lambda m: statuses.update(json.loads(m.data)), 10)]
    control = node.create_client(ControlWorld, '/world/cooperative_transport/control')
    report = {'passed': False, 'condition': 'ARM_ONLY_ungrasped', 'fault_test': a.fault}

    def step():
        rclpy.spin_once(node, timeout_sec=.001)
        wall = time.monotonic()
        if wall-last_excitation[0] >= .025:
            force = Wrench()
            (force.force.x, force.force.y, force.force.z, force.torque.x, force.torque.y, force.torque.z) = applied
            excitation.publish(force); last_excitation[0] = wall
        if wall-last_record[0] >= .05 and all((r, k) in latest for r in robots for k in ('joints', 'wrench', 'encoder')):
            last_record[0] = wall
            row = {'sim_time': now[0], 'wall_time': wall, 'phase': phase[0], 'fault': statuses.get('fault', ''),
                   'applied_fx_N': applied[0], 'applied_fy_N': applied[1]}
            if adm_state.get('reference_xyz_m') is not None:
                for i, k in enumerate('xyz'): row['reference_'+k] = adm_state['reference_xyz_m'][i]
            if adm_state.get('displacement_xy_m') is not None:
                for i, k in enumerate('xy'): row['admittance_'+k] = adm_state['displacement_xy_m'][i]
            row['admittance_enabled'] = adm_state.get('enabled', False)
            row['saturated'] = any(adm_state.get('saturated_xy') or [False])
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
                row[robot+'_encoder_vy'] = encoder.twist.twist.linear.y
                row[robot+'_encoder_wz'] = encoder.twist.twist.angular.z
                if robot in models:
                    grasp, sensor = models[robot].frames(q)
                    for i,k in enumerate('xyz'): row[robot+'_tcp_'+k] = float(grasp[i])
                    row[robot+'_tool_yaw'] = float(np.arctan2(sensor[1,0],sensor[0,0]))
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
                    row[robot+'_gt_y'] = poses[robot].translation.y
                    row[robot+'_gt_z'] = poses[robot].translation.z
                    rotation = poses[robot].rotation
                    row[robot+'_gt_yaw'] = float(np.arctan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                        1-2*(rotation.y**2+rotation.z**2)))
            if 'cooperative_payload' in poses:
                t = poses['cooperative_payload']
                for k in 'xyz':
                    row['payload_'+k] = getattr(t.translation, k)
                for k in 'xyzw':
                    row['payload_q'+k] = getattr(t.rotation, k)
            for i, k in enumerate('xy'):
                row['filtered_f'+k+'_N'] = (adm_state.get('force_xy_N') or [0., 0.])[i]
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
                        'unverified publisher', 'unverified preprocessor publisher', 'waiting for local sensors and gate', 'fresh stationary local samples required'))):
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
            '/world/cooperative_transport/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
        start('evaluation', bridge_args)
        wait(lambda: statuses.get('ready') and control.service_is_ready(), 45.)
        start('transport', ['ros2', 'launch', 'amir_compliance_control', 'arm_only.launch.py',
            'run_directory:='+str(a.run_directory), 'calibration_file:='+str(a.calibration_file)])
        wait(lambda: all(endpoint_states.get(r, {}).get('local_ready') for r in robots), 20.)
        for robot in robots:
            service('/'+robot+'/transport/prepare', retry_discovery=True)
        service('/transport/safety/arm', retry_discovery=True)
        wait(lambda: statuses.get('armed'), 10., check_fault=True)
        phase[0] = 'local_home'
        wait(lambda: all(endpoint_states[r]['state'] == 'READY' for r in robots), 15., check_fault=True)
        phase[0] = 'tare_settle'
        sim_wait(1.)
        service('/follower/tare_wrench', retry_discovery=True)
        wait(lambda: filter_state[0] == 'READY', 5., check_fault=True)
        sim_wait(.2)
        service('/follower/admittance/enable')
        wait(lambda: adm_state.get('enabled'), 5., check_fault=True)
        if a.fault == 'none':
            phases = [('baseline', [0., 0.], 1.), ('fx_positive', [.1, 0.], 2.),
                      ('release_x1', [0., 0.], 2.), ('fx_negative', [-.1, 0.], 2.),
                      ('release_x2', [0., 0.], 2.), ('fy_positive', [0., .1], 2.),
                      ('release_y1', [0., 0.], 2.), ('fy_negative', [0., -.1], 2.),
                      ('release_y2', [0., 0.], 2.), ('saturation', [.5, 0.], 2.),
                      ('recovery', [0., 0.], 3.)]
            for name, command, duration in phases:
                phase[0], applied[:2] = name, command
                sim_wait(duration)
            results = []
            for name, axis, direction in [('fx_positive', 0, 1.), ('fx_negative', 0, -1.),
                                           ('fy_positive', 1, 1.), ('fy_negative', 1, -1.)]:
                selected = [r for r in rows if r['phase'] == name]
                selected = [r for r in selected if r['sim_time'] >= selected[-1]['sim_time']-.5]
                delta = np.mean([[r['follower_tcp_'+k]-r['reference_'+k] for k in 'xyz'] for r in selected], axis=0)
                passed = (.7*.00025 < direction*delta[axis] < 1.3*.00025
                          and abs(delta[1-axis]) < .00005 and abs(delta[2]) < .00002)
                results.append({'phase':name,'mean_measured_delta_xyz_m':delta.tolist(),'passed':bool(passed)})
            report['direction_tests'] = results
            saturated = [r for r in rows if r['phase'] == 'saturation']
            report['saturation_observed'] = any(r['saturated'] for r in saturated)
            max_offset = max(abs(r.get('admittance_'+k, 0.)) for r in rows for k in 'xy')
            report['max_virtual_offset_m'] = max_offset
            final = [r for r in rows if r['phase'] == 'recovery' and r['sim_time'] >= rows[-1]['sim_time']-.5]
            recovery = np.mean([[r['follower_tcp_'+k]-r['reference_'+k] for k in 'xyz'] for r in final], axis=0)
            report['recovery_mean_delta_xyz_m'] = recovery.tolist()
            if not all(r['passed'] for r in results) or not report['saturation_observed'] or max_offset > .000500001 or np.max(np.abs(recovery)) > .00002:
                raise RuntimeError('direction, displacement saturation or recovery check failed')
        else:
            phase[0], applied[:2] = 'force_before_fault', [.1, 0.]
            sim_wait(.4)
            phase[0] = 'fault_injected'
            injection = time.monotonic()
            if a.fault == 'dropout':
                pid = find_process(b'/follower/ft_sensor@geometry_msgs/msg/WrenchStamped[')
                os.kill(pid, signal.SIGSTOP)
                suspended.append(pid)
            elif a.fault == 'preprocessor':
                pid = find_process(b'/lib/amir_compliance_control/wrench_preprocessor')
                os.kill(pid, signal.SIGSTOP); suspended.append(pid)
            else:
                pause(True)
            wait(lambda: all(r in actual_commands and abs(actual_commands[r]) < 1e-12 for r in robots)
                 and all(not debug.get(r, {}).get('active', True) for r in robots), 2.)
            report['dual_zero_latency_wall_s'] = time.monotonic()-injection
            if report['dual_zero_latency_wall_s'] > .4:
                raise RuntimeError('dual stop exceeded experiment wall-time budget')
            if a.fault in ('dropout', 'preprocessor'):
                os.kill(suspended.pop(), signal.SIGCONT)
            if a.fault == 'clock':
                pause(False)
            wall_wait(.5)
            if any(abs(actual_commands.get(r, 1.)) > 1e-12 for r in robots):
                raise RuntimeError('motion resumed automatically after fault recovery')
            if any(abs(latest[(r, 'encoder')].twist.twist.linear.x) > .0001 for r in robots):
                raise RuntimeError('wheel odometry did not confirm both bases stopped')
            if adm_state.get('enabled', True) or any(endpoint_states[r]['state'] != 'SAFE_STOP' for r in robots):
                raise RuntimeError('arm admittance or endpoint resumed after fault recovery')
            late = [row for row in rows if row['wall_time'] > time.monotonic()-.2]
            q_span = [max(row[f'follower_q{i}_rad'] for row in late)-min(row[f'follower_q{i}_rad'] for row in late)
                      for i in range(1, 6)]
            report['post_fault_joint_span_rad'] = q_span
            if max(q_span) > .00001:
                raise RuntimeError('arm did not hold measured position after fault')
            report['no_automatic_restart'] = True
            report['gate_faults'] = {r: debug.get(r, {}).get('fault') for r in robots}
        if any(abs(r.get(robot+'_requested_vx', 0.)) > 1e-12 for r in rows for robot in robots):
            raise RuntimeError('ARM_ONLY issued a nonzero base motion request')
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        applied[:] = [0.]*6
        excitation.publish(Wrench())
        for pid in suspended:
            os.kill(pid, signal.SIGCONT)
        if a.fault == 'clock' and control.service_is_ready():
            try:
                pause(False)
            except Exception:
                pass
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
        report['final_safety_state'] = statuses
        report['last_received_safety_state'] = statuses
        report['endpoint_states'] = endpoint_states
        report['last_admittance_state'] = adm_state
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
        graph = {}
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                graph[namespace.rstrip('/')+'/'+name] = {
                    'subscriptions': node.get_subscriber_names_and_types_by_node(name, namespace),
                    'publishers': node.get_publisher_names_and_types_by_node(name, namespace)}
            except Exception:
                pass
        (a.output/'graph.json').write_text(json.dumps(graph, indent=2))
        required = ('/follower/follower_arm_admittance', '/follower/wrench_preprocessor', '/follower/safety_gate')
        follower_inputs = [v[0] for key, value in graph.items() if key in required for v in value['subscriptions']]
        report['follower_input_boundary_passed'] = all(n in graph for n in required) and not any(t.startswith('/leader/') or 'goal' in t or 'ground_truth' in t or t.endswith('/tf') or '/world/' in t for t in follower_inputs)
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
    if not report['passed']:
        raise RuntimeError('CP-5 graph boundary or experiment validation failed')


if __name__ == '__main__':
    main()
