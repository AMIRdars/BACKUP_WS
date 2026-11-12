#!/usr/bin/env python3
"""CP-8R-3 grasp-priority whole-body rotation trial: leader-only goal, no shared centre.

Ground truth belongs to this evaluator only. Control nodes are separate
processes. Only the leader receives its constant, verified fixture calibration.
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
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml

from amir_compliance_control.object_center_rotation_trajectory import inverse_transform
from amir_compliance_control.wrench import LocalModel
from amir_cp8r2_trial import prepare_control_policies as cp8r2_policies
from amir_cp8r3_clearance import BaseClearance


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1.-2*(q.y*q.y+q.z*q.z))


def pose_matrix(position, quaternion):
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > 1e-5:
        raise ValueError('invalid evaluator quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = [position.x, position.y, position.z]
    return t


def prepare_control_policies(folder, output, duration, force_gain, force_velocity_gain, yaw_gain, xy_gain, yaw_metric, sensor_certificate):
    # Reuse the previously verified calibration/explicit policies, with no
    # geometry guess and no changes to the earlier experiment implementation.
    return cp8r2_policies(folder, output, duration, force_gain, force_velocity_gain, yaw_gain, xy_gain, yaw_metric, sensor_certificate)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--controller', choices=['whole_body', 'arm_base'], default='whole_body')
    p.add_argument('--grasp-slow-rate', type=float, default=.5)
    p.add_argument('--warning-fraction', type=float, default=.6)
    p.add_argument('--allocation-filter-hz', type=float, default=0.)
    p.add_argument('--allocation-base-regularization', type=float, default=1000.)
    p.add_argument('--delta-deg', type=float, required=True)
    p.add_argument('--duration-s', type=float, required=True)
    p.add_argument('--force-memory-gain', type=float, required=True)
    p.add_argument('--force-velocity-gain', type=float, required=True)
    p.add_argument('--yaw-gain', type=float, default=2.)
    p.add_argument('--xy-gain', type=float, default=.5)
    p.add_argument('--arm-yaw-metric', type=float, required=True)
    p.add_argument('--settle-s', type=float, default=30.)
    p.add_argument('--stop-observe-s', type=float, default=2.)
    p.add_argument('--wrench-calibration-file', type=Path, required=True)
    a = p.parse_args()
    a.output, a.run_directory = a.output.resolve(), a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    if (not np.isfinite([a.delta_deg, a.duration_s, a.settle_s, a.stop_observe_s, a.force_memory_gain,
                         a.force_velocity_gain, a.yaw_gain, a.xy_gain, a.arm_yaw_metric]).all()
            or not 0. < abs(a.delta_deg) <= 5. or a.duration_s <= 0. or a.settle_s < 0.
            or a.stop_observe_s < .5 or min(a.force_memory_gain, a.force_velocity_gain, a.yaw_gain, a.xy_gain) < 0.
            or a.arm_yaw_metric <= 0.):
        raise ValueError('small-angle positive-duration simulation envelope required')
    grasped = True
    report = {'passed': False, 'condition': 'grasp_priority_'+a.controller, 'delta_deg': a.delta_deg,
              'argv': __import__('sys').argv, 'ROS_DISTRO': os.getenv('ROS_DISTRO'),
              'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION')}
    if report['ROS_DISTRO'] != 'humble' or not report['IGN_PARTITION'] or not report['ROS_DOMAIN_ID']:
        raise ValueError('explicit Humble, isolated ROS_DOMAIN_ID and IGN_PARTITION required')
    policy_file, follower_file = prepare_control_policies(a.run_directory, a.output, a.duration_s,
        a.force_memory_gain, a.force_velocity_gain, a.yaw_gain, a.xy_gain, a.arm_yaw_metric, a.wrench_calibration_file)
    if a.controller == 'whole_body':
        policy = yaml.safe_load(follower_file.read_text()).pop('/follower/follower_yaw_arm_base')
        policy['ros__parameters'].update(allocation_base_filter_hz=a.allocation_filter_hz,
            allocation_base_regularization=a.allocation_base_regularization, grasp_warning_fraction=a.warning_fraction)
        follower_file.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body': policy}))
        leader_policy = yaml.safe_load(policy_file.read_text())
        leader_policy['/leader/leader_object_rotation']['ros__parameters'].update(grasp_slow_rate=a.grasp_slow_rate,
            grasp_progress_deceleration_s_inv=.5)
        policy_file.write_text(yaml.safe_dump(leader_policy))
    report['completion_policy'] = 'grasp retained, rotation program finished, settled; GT angle/centre are diagnostic only'

    profiles = {r: (a.run_directory/(r+'_safety.json')).read_bytes() for r in ('leader', 'follower')}
    for robot, data in profiles.items():
        profile = json.loads(data)
        profile['require_grasp_for_base_motion'] = True
        profile['ungrasped_base_test_only'] = False
        (a.run_directory/(robot+'_safety.json')).write_text(json.dumps(profile, indent=2)+'\n')
        (a.output/(robot+'_safety_used.json')).write_text(json.dumps(profile, indent=2)+'\n')
    code = Path(__file__).resolve().parents[1]/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    dependencies=[Path(__file__),*[Path(__file__).with_name(n) for n in (
        'amir_cp8r2_trial.py','amir_cp8r1_base_clearance.py','amir_cp8r3_clearance.py','amir_cp8r3_process.py',
        'cp9_transport_preparation.py','amir_cp9_grasp_preparation.py')]]
    source_files=[*code.glob('*.py'),*dependencies]
    source_hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files}
    (a.output/'source_sha256.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    workspace=Path(__file__).resolve().parents[1]
    for file in source_files:
        destination=a.output/'source_snapshot'/file.resolve().relative_to(workspace)
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(file.read_bytes())
    processes, logs = [], []

    def start(name, command):
        log = (a.output/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    rclpy.init(args=[])
    node = rclpy.create_node('cp8r3_evaluator')
    latest, states, gates, guards, poses, outputs = {}, {}, {}, {}, {}, {}
    now, safety, ref, rows, phase = [0.], {}, {}, [], ['startup']
    grasp_states, follow_debug, filter_state = {}, {}, ['']
    progress, grasp_loss = {}, []
    def grasp_update(robot, msg):
        grasp_states[robot] = msg.data
        if phase[0] in ('baseline', 'rotation', 'hold', 'safe_stop') and msg.data != 'attached':
            grasp_loss.append({'robot': robot, 'sim_time': now[0], 'state': msg.data})
    models = {r: LocalModel((a.run_directory/(r+'.urdf')).read_text()) for r in ('leader', 'follower')}
    initial_object_grasps = {}
    clearance = BaseClearance(a.run_directory)
    (a.output/'base_geometry.json').write_text(json.dumps({'bounds':clearance.bounds,'reserve_m':clearance.reserve},indent=2)+'\n')
    subs = []
    for robot in ('leader', 'follower'):
        for key, suffix, typ in [('joints', 'joint_states', JointState), ('encoder', 'mecanum_drive_controller/odometry', Odometry),
                                 ('wrench', 'ft_sensor', WrenchStamped)]:
            subs.append(node.create_subscription(typ, f'/{robot}/{suffix}',
                lambda msg, r=robot, k=key: latest.__setitem__((r, k), msg), qos_profile_sensor_data))
        for suffix, target in [('transport/state', states), ('safety_debug', gates), ('interlock/debug', guards)]:
            subs.append(node.create_subscription(String, f'/{robot}/{suffix}',
                lambda msg, r=robot, d=target: d.__setitem__(r, json.loads(msg.data)), 10))
        subs.append(node.create_subscription(Twist, f'/{robot}/rover_twist',
            lambda msg, r=robot: outputs.__setitem__(r, [msg.linear.x, msg.linear.y, msg.angular.z]), 10))
        subs.append(node.create_subscription(String, f'/{robot}/grasp/state',
            lambda msg, r=robot: grasp_update(r, msg), 10))
    subs.append(node.create_subscription(Clock, '/clock', lambda msg: now.__setitem__(0,
        msg.clock.sec+msg.clock.nanosec*1e-9), qos_profile_sensor_data))
    subs.append(node.create_subscription(String, '/transport/safety_status', lambda msg: safety.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/leader/grasp_priority/debug', lambda msg: progress.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/leader/rotation/debug', lambda msg: ref.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/follower/rotation_follow/debug', lambda msg: follow_debug.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/follower/wrench_status', lambda msg: filter_state.__setitem__(0, msg.data), 10))


    def gt(msg):
        for t in msg.transforms:
            if t.child_frame_id in ('leader', 'follower', 'cooperative_payload'):
                poses[t.child_frame_id] = pose_matrix(t.transform.translation, t.transform.rotation)

    subs.append(node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', gt, qos_profile_sensor_data))
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    goal = node.create_publisher(Float64, '/leader/object_rotation_delta', 1)
    attach = {r: node.create_publisher(Empty, '/'+r+'/grasp/attach', 1) for r in ('leader', 'follower')}
    raw = (a.output/'samples.jsonl').open('w', buffering=1)
    last_record = [0.]

    def step():
        rclpy.spin_once(node, timeout_sec=.002)
        if time.monotonic()-last_record[0] < .05:
            return
        last_record[0] = time.monotonic()
        row = {'t_s': now[0], 'wall_s': time.monotonic(), 'phase': phase[0], 'safety_fault': safety.get('fault', ''),
               'leader_state': states.get('leader', {}).get('state', ''),
               'follower_state': states.get('follower', {}).get('state', ''),
               'progress_rate': progress.get('rate', 1.), 'grasp_warning_latched': progress.get('warning_latched', False),
               'grasp_utilization': follow_debug.get('grasp_utilization'),
               'phi_cmd_rad': ref.get('phi_rad'), 'omega_cmd_rad_s': ref.get('omega_rad_s'),
               'yaw_tracking_error_rad': ref.get('yaw_error_rad'),
               'arm_position_error_m': ref.get('arm_position_error_m'), 'arm_angle_error_rad': ref.get('arm_angle_error_rad')}
        for robot in ('leader', 'follower'):
            if (robot, 'encoder') in latest:
                e = latest[robot, 'encoder'].pose.pose
                row.update({robot+'_encoder_x': e.position.x, robot+'_encoder_y': e.position.y, robot+'_encoder_yaw': yaw(e.orientation)})
            if robot in poses:
                for i, k in enumerate('xyz'):
                    row[robot+'_GT_'+k] = float(poses[robot][i, 3])
                row[robot+'_GT_yaw'] = math.atan2(poses[robot][1, 0], poses[robot][0, 0])
            for value, component in zip(outputs.get(robot, [None]*3), ('vx', 'vy', 'wz')):
                row[robot+'_output_'+component] = value
            row[robot+'_grasp_state'] = grasp_states.get(robot, '')
            if (robot, 'wrench') in latest:
                w = latest[robot, 'wrench'].wrench
                for key, value in zip(('fx', 'fy', 'fz', 'tx', 'ty', 'tz'),
                                     (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)):
                    row[robot+'_wrist_raw_'+key] = value
            if (robot, 'joints') in latest:
                j = latest[robot, 'joints']
                mapping = dict(zip(j.name, j.position))
                model = models[robot]
                q = np.array([mapping[n] for n in model.names])
                tcp, sensor = model.frames(q)
                row[robot+'_joint_margin_rad'] = float(np.min(np.r_[q-model.lower, model.upper-q]))
                for i, k in enumerate('xyz'):
                    row[robot+'_ee_in_base_'+k] = float(tcp[i])
                for i, value in enumerate(q):
                    row[robot+'_q'+str(i+1)+'_rad'] = float(value)
                for i, value in enumerate(gates.get(robot, {}).get('reaction_torque_Nm') or []):
                    row[robot+'_reaction_torque_'+str(i+1)+'_Nm'] = value
                if grasped and robot in poses and 'cooperative_payload' in poses:
                    local_grasp = sensor.copy()
                    local_grasp[:3, 3] = tcp
                    world_grasp = poses[robot]@local_grasp
                    if robot in initial_object_grasps:
                        desired = poses['cooperative_payload']@initial_object_grasps[robot]
                        row[robot+'_grasp_constraint_position_error_m'] = float(np.linalg.norm(world_grasp[:3, 3]-desired[:3, 3]))
        if 'center_odom_xyz' in ref:
            for i, k in enumerate('xyz'):
                row['center_odom_'+k] = ref['center_odom_xyz'][i]
        if 'cooperative_payload' in poses and grasped:
            for i, k in enumerate('xyz'):
                row['object_center_GT_'+k] = float(poses['cooperative_payload'][i, 3])
            row['object_yaw_GT_rad'] = math.atan2(poses['cooperative_payload'][1, 0], poses['cooperative_payload'][0, 0])
            row['object_vertical_axis_dot_z'] = float(poses['cooperative_payload'][2, 1])
        for index, key in enumerate(('x', 'y', 'yaw')):
            row['follower_local_error_'+key] = (follow_debug.get('measured_error_xy_yaw') or [0.]*3)[index]
            row['follower_force_memory_'+key] = (follow_debug.get('force_memory_velocity_xy_yaw') or [0.]*3)[index]
        row['follower_tau_z_at_tcp_Nm'] = follow_debug.get('tau_z_at_tcp_Nm')
        row['follower_yaw_admittance_rad'] = follow_debug.get('admittance_yaw_rad')
        row['follower_wrench_status'] = filter_state[0]
        for kind in ('raw_tared_tcp_wrench', 'filtered_tcp_wrench'):
            for i, value in enumerate(follow_debug.get(kind) or []):
                row['follower_'+kind+'_'+str(i)] = value
        for i, value in enumerate(follow_debug.get('residual_twist') or []):
            row['follower_arm_task_residual_'+str(i)] = value
        row['follower_arm_min_metric_singular'] = follow_debug.get('arm_min_metric_singular', follow_debug.get('arm_min_singular'))
        for key in ('whole_min_singular','base_regularization','grasp_warning'):
            row['follower_'+key] = follow_debug.get(key)
        for i,value in enumerate(follow_debug.get('velocity') or []):
            row['follower_allocated_velocity_'+str(i)] = value
        if all(r in poses for r in ('leader','follower')):
            row['base_surface_gap_m'] = clearance.gap(poses['leader'],poses['follower'])
            if phase[0] in ('rotation','hold') and row['base_surface_gap_m'] <= clearance.reserve:
                raise RuntimeError('evaluated base separation below geometry reserve')
        rows.append(row)
        raw.write(json.dumps(row)+'\n')

    def wait(predicate, timeout=60., check_fault=False):
        deadline = time.monotonic()+timeout
        while not predicate():
            if check_fault and grasp_loss:
                raise RuntimeError('grasp loss observed: '+str(grasp_loss[0]))
            if check_fault and safety.get('fault'):
                raise RuntimeError('shared stop: '+safety['fault'])
            if time.monotonic() > deadline:
                raise RuntimeError(f'timeout phase={phase[0]} states={states} safety={safety}')
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError('trial-owned process exited: '+str(process.args))
            step()

    def settle(seconds, check_fault=True):
        begin = now[0]
        wait(lambda: now[0]-begin >= seconds, timeout=max(30., seconds*5.), check_fault=check_fault)

    def service(name):
        client = node.create_client(Trigger, name)
        wait(client.service_is_ready, 15.)
        deadline = time.monotonic()+5.
        while True:
            f = client.call_async(Trigger.Request())
            wait(f.done, 10.)
            result = f.result()
            if result.success or time.monotonic() > deadline:
                break
            until = time.monotonic()+.2
            wait(lambda: time.monotonic() >= until, 2.)
        node.destroy_client(client)
        with (a.output/'services.jsonl').open('a') as stream:
            stream.write(json.dumps({'name': name, 'success': result.success, 'message': result.message, 't_s': now[0]})+'\n')
        if not result.success:
            raise RuntimeError(name+': '+result.message)

    def remove(name):
        req = DeleteEntity.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        f = remover.call_async(req)
        wait(f.done, 10.)
        if not f.result().success:
            raise RuntimeError('cannot remove '+name)

    def measured_world_vertical_force(robot):
        j = latest[robot, 'joints']
        mapping = dict(zip(j.name, j.position))
        model = models[robot]
        _, sensor = model.frames([mapping[n] for n in model.names])
        f = latest[robot, 'wrench'].wrench.force
        return float((poses[robot][:3, :3]@sensor[:3, :3]@np.array([f.x, f.y, f.z]))[2])

    try:
        start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp6_fixture.launch.py',
                        'run_directory:='+str(a.run_directory)])
        start('evaluation_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            *[f'/{r}/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg' for r in ('leader', 'follower')],
            *[f'/{r}/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty' for r in ('leader', 'follower')]])
        wait(lambda: all((r, k) in latest for r in ('leader', 'follower') for k in ('joints', 'encoder'))
             and all(r in poses for r in ('leader', 'follower', 'cooperative_payload')) and remover.service_is_ready(), 100.)
        phase[0] = 'settled_fixture_measurement'
        settle(2., False)
        measured = {}
        for robot in ('leader', 'follower'):
            xml = (a.run_directory/(robot+'.urdf')).read_text()
            model = LocalModel(xml)
            j = latest[robot, 'joints']
            mapping = dict(zip(j.name, j.position))
            q = [mapping[n] for n in model.names]
            tcp, sensor = model.frames(q)
            world_sensor = poses[robot]@sensor
            measured[robot] = {'q_rad': q, 'world_T_base_GT': poses[robot].tolist(),
                'sensor_xyz_in_base': sensor[:3, 3].tolist(), 'sensor_xyz_world_evaluation': world_sensor[:3, 3].tolist(),
                'object_xyz_world_evaluation': poses['cooperative_payload'][:3, 3].tolist()}
        (a.output/'settled_fixture_measurement.json').write_text(json.dumps(measured, indent=2)+'\n')
        if not grasped:
            remove('cooperative_payload')
            remove('grasp_support')
        start('safety', ['ros2', 'launch', 'amir_compliance_control', 'guarded_safety.launch.py',
                         'run_directory:='+str(a.run_directory)])
        start('leader', ['ros2', 'launch', 'amir_compliance_control', ('grasp_priority_leader.launch.py' if a.controller == 'whole_body' else 'object_rotation_leader.launch.py'),
                         'run_directory:='+str(a.run_directory), 'rotation_policy_file:='+str(policy_file)])
        start('follower', ['ros2', 'launch', 'amir_compliance_control', ('yaw_whole_body.launch.py' if a.controller == 'whole_body' else 'yaw_arm_base.launch.py'),
            'run_directory:='+str(a.run_directory), 'follower_policy_file:='+str(follower_file),
            'wrench_calibration_file:='+str(a.wrench_calibration_file.resolve())])
        wait(lambda: safety.get('ready') and all(states.get(r, {}).get('local_ready') for r in ('leader', 'follower')), 45.)
        service('/leader/transport/prepare')
        service('/follower/transport/prepare')
        service('/transport/safety/arm')
        phase[0] = 'home'
        wait(lambda: all(states.get(r, {}).get('state') == 'READY' for r in ('leader', 'follower')), 20., True)
        if grasped:
            phase[0] = 'fixed_grasp'
            baseline_fz = sum(measured_world_vertical_force(r) for r in ('leader', 'follower'))
            for _ in range(10):
                for publisher in attach.values():
                    publisher.publish(Empty())
                until = time.monotonic()+.05
                wait(lambda: time.monotonic() >= until, 1.)
            wait(lambda: all(grasp_states.get(r) == 'attached' for r in ('leader', 'follower')), 10., True)
            remove('grasp_support')
            settle(10.)
            late = [r for r in rows if r['phase'] == 'fixed_grasp' and r['t_s'] > now[0]-2.
                    and 'object_center_GT_z' in r]
            if len(late) < 5:
                raise RuntimeError('insufficient static grasp metrology')
            drift = [float(np.ptp([r['object_center_GT_'+k] for r in late])) for k in 'xyz']
            world = ET.parse(a.run_directory/'world.sdf')
            mass = sum(float(e.text) for e in world.findall(
                "world/model[@name='cooperative_payload']/link/inertial/mass"))
            gravity = abs(float(world.findtext('world/gravity').split()[2]))
            measured_weight = baseline_fz-sum(measured_world_vertical_force(r) for r in ('leader', 'follower'))
            report['static_grasp_check'] = {'mass_kg': mass, 'expected_weight_N': mass*gravity,
                'measured_added_weight_N': measured_weight, 'late_center_span_xyz_m': drift}
            if max(drift) > .0001 or abs(measured_weight-mass*gravity) > .1:
                raise RuntimeError('fixed-grasp static load/centre stability check failed')
        phase[0] = 'tare'
        service('/follower/tare_wrench')
        wait(lambda: filter_state[0] == 'READY', 10., True)
        service('/follower/admittance/enable')
        settle(.3)
        phase[0] = 'baseline'
        settle(1.)
        origin = {r: poses[r].copy() for r in ('leader', 'follower')}
        object_start = poses.get('cooperative_payload', np.eye(4)).copy()
        for robot in ('leader', 'follower'):
            mapping = dict(zip(latest[robot, 'joints'].name, latest[robot, 'joints'].position))
            tcp, sensor = models[robot].frames([mapping[n] for n in models[robot].names])
            sensor[:3, 3] = tcp
            initial_object_grasps[robot] = inverse_transform(object_start)@origin[robot]@sensor
        if any(hashlib.sha256(Path(file).read_bytes()).hexdigest()!=digest for file,digest in source_hashes.items()):
            raise RuntimeError('experiment source changed before goal; motion refused')
        goal.publish(Float64(data=math.radians(a.delta_deg)))
        phase[0] = 'rotation'
        wait(lambda: states.get('leader', {}).get('accepted_goals') == 1, 5., True)
        wait(lambda: states.get('leader', {}).get('completed_goals') == 1, max(1000., a.duration_s*5.), True)
        phase[0] = 'hold'
        hold_begin, quiet_since = now[0], [None]
        def settled_with_grasp():
            error = follow_debug.get('measured_error_xy_yaw')
            command = follow_debug.get('base_twist')
            quiet = (error is not None and command is not None and
                np.linalg.norm([command[0],command[1],.1*command[2]]) < .00001 and
                np.linalg.norm(error[:2]) < .0001 and abs(error[2]) < .0001 and
                all(grasp_states.get(robot)=='attached' for robot in ('leader','follower')))
            quiet_since[0] = (now[0] if quiet_since[0] is None else quiet_since[0]) if quiet else None
            if now[0]-hold_begin > a.settle_s:
                raise RuntimeError('local grip-preserving settling deadline exceeded')
            return quiet_since[0] is not None and now[0]-quiet_since[0] >= 2.
        wait(settled_with_grasp,max(30.,a.settle_s*5.),True)
        report['settling_time_s'] = now[0]-hold_begin
        movement = {r: inverse_transform(origin[r])@poses[r] for r in origin}
        gt_angle = math.atan2(movement['leader'][1, 0], movement['leader'][0, 0])
        report['leader_GT_delta_yaw_rad'] = gt_angle
        report['leader_GT_delta_xyz_m'] = movement['leader'][:3, 3].tolist()
        report['follower_GT_delta_xyz_m'] = movement['follower'][:3, 3].tolist()
        report['leader_encoder_angle_error_rad'] = ref.get('yaw_error_rad')
        report['gt_angle_ratio'] = gt_angle/math.radians(a.delta_deg)
        report['motion_completed_in_own_odom'] = True
        report['passed'] = True  # Angle error is a recorded diagnostic, per user priority.
        late = [r for r in rows if r['phase'] == 'hold' and r['t_s'] > now[0]-1.]
        report['final_follower_speed_m_s'] = float(max(np.linalg.norm([
            r.get('follower_output_vx', 0.), r.get('follower_output_vy', 0.), .1*r.get('follower_output_wz', 0.)]) for r in late))
        report['final_follower_local_error_xy_yaw'] = follow_debug.get('measured_error_xy_yaw')
        report['passed'] = bool(report['passed'] and report['final_follower_speed_m_s'] < .00001
            and np.linalg.norm(report['final_follower_local_error_xy_yaw'][:2]) < .0001
            and abs(report['final_follower_local_error_xy_yaw'][2]) < .0001)
        report['local_following_passed'] = bool(report['passed'] and states.get('follower', {}).get('accepted_goals') == 0)
        if grasped:
            object_delta = inverse_transform(object_start)@poses['cooperative_payload']
            # Object P has rolled axes: use world long-axis heading directly.
            object_yaw = math.atan2(poses['cooperative_payload'][1, 0], poses['cooperative_payload'][0, 0])
            object_yaw0 = math.atan2(object_start[1, 0], object_start[0, 0])
            object_angle = math.atan2(math.sin(object_yaw-object_yaw0), math.cos(object_yaw-object_yaw0))
            report['object_GT_delta_yaw_rad'] = object_angle
            report['object_motion_observed'] = abs(object_angle)>1e-6
            report['object_center_drift_m'] = float(np.linalg.norm(object_delta[:3, 3]))
            moving_centers = np.array([[r['object_center_GT_'+k] for k in 'xyz'] for r in rows
                                      if r['phase'] in ('rotation', 'hold') and 'object_center_GT_z' in r])
            report['object_center_max_drift_m'] = float(np.max(np.linalg.norm(moving_centers-object_start[:3, 3], axis=1)))
            report['evaluation_only_shared_reference'] = False
            report['center_drift_limit_m'] = .001
            report['center_drift_criterion_passed'] = max(report['object_center_drift_m'], report['object_center_max_drift_m']) < .001
            report['passed'] = bool(report['passed'] and states.get('follower', {}).get('accepted_goals') == 0
                and not grasp_loss and report['object_motion_observed'])
        if not report['passed']:
            report['error'] = 'grasp preservation, program completion or local settling criterion failed'
    except Exception as exc:
        report['error'], report['error_phase'] = str(exc), phase[0]
    finally:
        try:
            phase[0] = 'safe_stop'
            service('/transport/safety/stop')
            settle(a.stop_observe_s, False)
            stopped = [r for r in rows if r['phase'] == 'safe_stop']
            # Allow the established stop propagation interval, then require
            # persistent zero output and grasp hold, without any reset/rearm.
            stable = [r for r in stopped if r['t_s'] >= stopped[0]['t_s']+.3]
            report['both_outputs_zero_after_stop'] = bool(stable) and all(
                max(abs(r.get(robot+'_output_'+k, float('inf'))) for k in ('vx', 'vy', 'wz')) < 1e-12
                for r in stable for robot in ('leader', 'follower'))
            report['both_grasps_retained_after_stop'] = bool(stable) and all(
                r.get(robot+'_grasp_state') == 'attached' for r in stable for robot in ('leader', 'follower'))
            report['post_stop_observation_s'] = a.stop_observe_s
        except Exception as exc:
            report['cleanup_stop_error'] = str(exc)
        report['states'], report['final_safety'], report['gate_states'], report['interlocks'] = states, safety, gates, guards
        graph = {}
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                graph[namespace.rstrip('/')+'/'+name] = {
                    'subscriptions': node.get_subscriber_names_and_types_by_node(name, namespace),
                    'publishers': node.get_publisher_names_and_types_by_node(name, namespace)}
            except Exception:
                pass
        (a.output/'graph.json').write_text(json.dumps(graph, indent=2)+'\n')
        report['command_writer_counts'] = {r: len(node.get_publishers_info_by_topic('/'+r+'/base_cmd'))
                                           for r in ('leader', 'follower')}
        report['arm_command_writer_counts'] = {r: len(node.get_publishers_info_by_topic('/'+r+'/arm_joint_cmd'))
                                               for r in ('leader', 'follower')}
        leader_inputs = graph.get('/leader/leader_object_rotation', {}).get('subscriptions', [])
        report['leader_input_boundary_passed'] = bool(leader_inputs) and not any(
            t.startswith(('/follower/', '/experiment/', '/world/')) or 'ground_truth' in t or '/tf' in t for t, _ in leader_inputs)
        follower_names = (('/follower/follower_yaw_whole_body' if a.controller == 'whole_body' else '/follower/follower_yaw_arm_base'), '/follower/wrench_preprocessor',
                          '/follower/safety_gate', '/follower/actuator_interlock', '/follower/controller_audit')
        report['follower_input_boundary_passed'] = all(n in graph for n in follower_names) and not any(
            t.startswith(('/leader/', '/experiment/', '/world/')) or 'goal' in t or 'ground_truth' in t or '/tf' in t
            for name in follower_names for t, _ in graph.get(name, {}).get('subscriptions', []))
        report['grasp_loss_events'] = grasp_loss
        report['last_progress'] = progress
        report['last_follower_debug'], report['last_wrench_status'] = follow_debug, filter_state[0]
        report['passed'] = bool(report['passed'] and report['leader_input_boundary_passed'] and report['follower_input_boundary_passed']
                                and report.get('both_outputs_zero_after_stop') and report.get('both_grasps_retained_after_stop')
                                and not grasp_loss and states.get('leader', {}).get('completed_goals') == 1
                                and all(n == 1 for n in report['command_writer_counts'].values())
                                and all(n == 1 for n in report['arm_command_writer_counts'].values()))
        keys = sorted({key for row in rows for key in row})
        with (a.output/'timeseries.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        (a.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('gate_states', 'interlocks')}, indent=2), flush=True)
        raw.close()
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
        from amir_cp8r3_process import reap_partition
        (a.output/'process_cleanup.json').write_text(json.dumps(reap_partition(report['IGN_PARTITION'], report['ROS_DOMAIN_ID']), indent=2)+'\n')
        for log in logs:
            log.close()
        for robot, data in profiles.items():
            (a.run_directory/(robot+'_safety.json')).write_bytes(data)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
