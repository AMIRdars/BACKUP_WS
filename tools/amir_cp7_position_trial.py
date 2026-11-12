#!/usr/bin/env python3
"""CP-7 comparison-only position baseline on the same guarded fixture.

The baseline uses two independent predefined local references. It is not an
autonomous force follower, and never becomes an input to the control package.
Ground truth is recorded exclusively by this evaluator.
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
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from amir_compliance_control.wrench import LocalModel, at_grasp


def reference(t, distance, speed, acceleration):
    """Symmetric triangular/trapezoidal position and feedforward velocity."""
    speed = min(speed, math.sqrt(distance*acceleration))
    ramp = speed/acceleration
    cruise = distance/speed-ramp
    duration = 2*ramp+cruise
    if t <= 0:
        return 0., 0., duration
    if t < ramp:
        return .5*acceleration*t*t, acceleration*t, duration
    if t < ramp+cruise:
        return .5*speed*ramp+speed*(t-ramp), speed, duration
    if t < duration:
        remaining = duration-t
        return distance-.5*acceleration*remaining**2, acceleration*remaining, duration
    return distance, 0., duration


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--delay', type=float, default=0.)
    p.add_argument('--speed', type=float, default=.00005)
    p.add_argument('--distance', type=float, default=.001)
    p.add_argument('--acceleration', type=float, default=.001)
    p.add_argument('--fault', choices=['none', 'dropout', 'supervisor', 'clock'], default='none')
    p.add_argument('--weld-cfm', type=float, default=.1)
    p.add_argument('--follower-speed-limit', type=float)
    a = p.parse_args()
    if (not all(math.isfinite(v) for v in (a.delay, a.speed, a.distance, a.acceleration, a.weld_cfm))
            or a.delay < 0 or min(a.speed, a.distance, a.acceleration) <= 0
            or (a.follower_speed_limit is not None and
                (not math.isfinite(a.follower_speed_limit) or a.follower_speed_limit <= 0))):
        raise ValueError('positive experiment trajectory settings required')
    a.output = a.output.resolve()
    a.run_directory = a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'profiles_used').mkdir()
    for robot in ('leader','follower'):
        (a.output/'profiles_used'/(robot+'_safety.json')).write_bytes((a.run_directory/(robot+'_safety.json')).read_bytes())
    (a.output/'command.json').write_text(json.dumps({'argv': __import__('sys').argv,
        'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION'),
        'condition': 'COMPARISON_POSITION_BASELINE_ONLY'}, indent=2))
    processes, logs, suspended = [], [], []
    def start(name, argv):
        file = (a.output/(name+'.log')).open('w')
        logs.append(file)
        process = subprocess.Popen(argv, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp6_fixture.launch.py',
                    'weld_cfm:='+str(a.weld_cfm), 'run_directory:='+str(a.run_directory)])
    rclpy.init(args=[])
    node = rclpy.create_node('cp3_position_baseline_evaluator')
    robots = ('leader', 'follower')
    now, latest, poses, statuses, debug = [0.], {}, {}, {}, {}
    request, arm_request, attach, subscriptions = {}, {}, {}, []
    actual_commands, requested, motion_origins, previous = {}, {}, {}, {r: 0. for r in robots}
    phase, app_active, motion_start = ['startup'], [False], [None]
    last_publish, last_record = [0.], [0.]
    rows, raw_log = [], (a.output/'samples.jsonl').open('w', buffering=1)
    models = {}
    from collections import deque
    pending = deque()
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
        request[robot] = node.create_publisher(TwistStamped, '/'+robot+'/base_cmd', 1)
        arm_request[robot] = node.create_publisher(JointTrajectory, '/'+robot+'/arm_joint_cmd', 1)
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
    report = {'passed': False, 'condition': 'comparison_position_baseline', 'fault_test': a.fault,
              'delay_s': a.delay, 'speed_m_s': a.speed, 'distance_m': a.distance,
              'acceleration_m_s2': a.acceleration, 'position_gain_s_inv': 1.}
    report['weld_cfm'] = a.weld_cfm
    report['follower_speed_limit_m_s'] = a.follower_speed_limit or a.speed

    def step():
        rclpy.spin_once(node, timeout_sec=.001)
        wall = time.monotonic()
        if app_active[0] and wall-last_publish[0] >= .025:
            dt = now[0]-last_publish[1] if len(last_publish) > 1 else .025
            last_publish[:] = [wall, now[0]]
            for robot in robots:
                command = 0.
                if motion_start[0] is not None and (robot, 'encoder') in latest:
                    direction = 1. if robot == 'leader' else -1.
                    local_time = now[0]-motion_start[0]
                    goal, velocity, _ = reference(local_time, a.distance, a.speed, a.acceleration)
                    x = latest[(robot, 'encoder')].pose.pose.position.x-motion_origins[robot]
                    cap = min(a.speed, a.follower_speed_limit or a.speed) if robot == 'follower' else a.speed
                    command = np.clip(direction*velocity+(direction*goal-x), -cap, cap)
                    command = float(np.clip(command, previous[robot]-a.acceleration*max(0., dt),
                                            previous[robot]+a.acceleration*max(0., dt)))
                previous[robot] = command
                requested[robot] = command
                msg = TwistStamped()
                msg.header.stamp, msg.header.frame_id = Time(seconds=now[0]).to_msg(), robot+'/base_footprint'
                msg.twist.linear.x = command
                if robot=='follower' and motion_start[0] is not None and a.delay>0:
                    pending.append((now[0]+a.delay,msg))
                else:request[robot].publish(msg)
        if app_active[0]:
            while pending and pending[0][0]<=now[0]:
                _,message=pending.popleft();request['follower'].publish(message)
        else:pending.clear()
        if wall-last_record[0] >= .05 and all((r, k) in latest for r in robots for k in ('joints', 'wrench', 'encoder')):
            last_record[0] = wall
            row = {'sim_time': now[0], 'wall_time': wall, 'phase': phase[0], 'fault': statuses.get('fault', '')}
            for robot in robots:
                msg = latest[(robot, 'joints')]
                mapping = dict(zip(msg.name, msg.position))
                row[robot+'_joint_stamp_s']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                velocities=dict(zip(msg.name,msg.velocity))
                for wheel in ('front_left_joint','front_right_joint','back_left_joint','back_right_joint'):
                    row[robot+'_'+wheel+'_velocity_rad_s']=velocities[wheel]
                q = [mapping[n] for n in models[robot].names] if robot in models else []
                for i, value in enumerate(q):
                    row[f'{robot}_q{i+1}_rad'] = value
                row[robot+'_requested_vx'] = requested.get(robot, 0.)
                row[robot+'_output_vx'] = actual_commands.get(robot, 0.)
                encoder = latest[(robot, 'encoder')]
                row[robot+'_encoder_stamp_s']=encoder.header.stamp.sec+encoder.header.stamp.nanosec*1e-9
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
                    or not any(v in result.message for v in ('command writer ownership not confirmed','unverified'))):
                break
            wall_wait(.2)
        node.destroy_client(client)
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
        # Initial readiness only: do not compete with original controller spawners.
        # No safety arm or autonomous command is allowed before all four are active.
        from controller_manager_msgs.srv import ListControllers
        startup_controllers={}
        for robot in robots:
            client=node.create_client(ListControllers,'/'+robot+'/controller_manager/list_controllers')
            wait(client.service_is_ready,30.)
            deadline=time.monotonic()+120.
            while time.monotonic()<deadline:
                future=client.call_async(ListControllers.Request());reply_deadline=time.monotonic()+2.
                while not future.done() and time.monotonic()<reply_deadline:step()
                if future.done() and future.exception() is None:
                    states={c.name:c.state for c in future.result().controller}
                    startup_controllers[robot]=states
                    if all(states.get(c)=='active' for c in ('joint_state_broadcaster','arm_controller','mecanum_drive_controller','gripper_controller')):break
                else:
                    client.remove_pending_request(future);future.cancel()
                wall_wait(.2)
            else:raise RuntimeError('initial controller readiness timeout: '+robot+' '+str(startup_controllers))
            node.destroy_client(client)
        (a.output/'startup_controllers.json').write_text(json.dumps(startup_controllers,indent=2))
        start('safety', ['ros2', 'launch', 'amir_compliance_control', 'guarded_safety.launch.py', 'run_directory:='+str(a.run_directory)])
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
        service('/transport/safety/arm', retry_discovery=True)
        wait(lambda: statuses.get('armed'), 10., check_fault=True)
        sim_wait(.2)
        app_active[0], phase[0] = True, 'home'
        targets = {}
        for robot in robots:
            root = ET.parse(a.run_directory/(robot+'.urdf'))
            targets[robot] = [float(root.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text)
                              for n in models[robot].names]
            msg = JointTrajectory(joint_names=list(models[robot].names))
            msg.header.stamp, msg.header.frame_id = Time(seconds=now[0]).to_msg(), robot+'/base_footprint'
            msg.points = [JointTrajectoryPoint(positions=targets[robot])]
            msg.points[0].time_from_start.sec = 3
            arm_request[robot].publish(msg)
        sim_wait(4.)
        for robot in robots:
            msg = latest[(robot, 'joints')]
            actual = dict(zip(msg.name, msg.position))
            if max(abs(actual[n]-t) for n, t in zip(models[robot].names, targets[robot])) > .001:
                raise RuntimeError(robot+': home not reached through safety gate')
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
        sim_wait(10.)
        origin = {k: poses[k].translation.x for k in robots+('cooperative_payload',)}
        motion_origins.update({r: latest[(r, 'encoder')].pose.pose.position.x for r in robots})
        motion_start[0], phase[0] = now[0], 'motion'
        duration = reference(0., a.distance, a.speed, a.acceleration)[2]+a.delay
        if a.fault == 'none':
            sim_wait(duration)
            motion_start[0] = None
            phase[0] = 'settle'
            sim_wait(6.)
            report['gt_distance_m'] = {k: poses[k].translation.x-origin[k] for k in origin}
            report['encoder_distance_m'] = {r: latest[(r, 'encoder')].pose.pose.position.x-motion_origins[r] for r in robots}
            report['target_error_m'] = {k: a.distance-v for k, v in report['gt_distance_m'].items()}
            if a.follower_speed_limit is not None and a.follower_speed_limit < a.speed:
                observed = max(abs(r.get('follower_output_vx', 0.)) for r in rows if r['phase'] == 'motion')
                report['observed_follower_speed_limit_m_s'] = observed
                report['tracking_target_reached'] = min(report['gt_distance_m'].values()) >= .8*a.distance
                if observed > a.follower_speed_limit+1e-9 or observed < .8*a.follower_speed_limit:
                    raise RuntimeError('requested follower velocity saturation was not observed')
                if report['gt_distance_m']['cooperative_payload'] <= 0:
                    raise RuntimeError('saturation baseline produced no transport')
            elif min(report['gt_distance_m'].values()) < .8*a.distance:
                raise RuntimeError('baseline did not achieve short transport distance')
        else:
            sim_wait(1.)
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
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        report['error_phase']=phase[0]
        if phase[0] in ('motion','settle'):
            try:
                wait(lambda:all(abs(actual_commands.get(r,1.))<1e-12 for r in robots),2.)
                wall_wait(.3)
                report['safe_stop_observed']=bool(statuses.get('fault')) and all(abs(actual_commands.get(r,1.))<1e-12 for r in robots)
            except Exception:report['safe_stop_observed']=False
        else:report['safe_stop_observed']=False
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
        report['final_safety_state'] = statuses
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
            report['request_onset_sim_s'] = {r: next((row['sim_time'] for row in motion if abs(row.get(r+'_requested_vx', 0.)) > .000025), None) for r in robots}
        graph = {}
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                graph[namespace.rstrip('/')+'/'+name] = {
                    'subscriptions': node.get_subscriber_names_and_types_by_node(name, namespace),
                    'publishers': node.get_publisher_names_and_types_by_node(name, namespace)}
            except Exception:
                pass
        (a.output/'graph.json').write_text(json.dumps(graph, indent=2))
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
