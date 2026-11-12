#!/usr/bin/env python3
"""Isolated Fortress CP-8R-1 ungrasped leader trial with saved live evidence.

Ground truth belongs to this evaluator only. Control nodes are separate
processes and receive only own state plus an offline virtual-fixture calibration.
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
from amir_compliance_control.wrench import LocalModel, origin
from amir_cp8r0_geometry_audit import fixture_geometry, generate_fixture


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


def prepare_virtual_calibration(folder, output):
    # Source geometry defines the virtual object relative to the robot. No
    # runtime GT observation is passed to the production leader.
    scratch = output/'nominal_fixture'
    spawn = generate_fixture(scratch)
    geometry = fixture_geometry(scratch, spawn)
    value = geometry['robots']['leader']
    file = output/'virtual_grasp_calibration.json'
    certificate = {'robot': 'leader', 'grasp_frame': 'tcp_link', 'base_frame': 'base_footprint',
        'object_frame': 'geometric_center_z_up', 'scope': 'virtual_ungrasped_fixture',
        'calibration_verified': True,
        'urdf_sha256': hashlib.sha256((folder/'leader.urdf').read_bytes()).hexdigest(),
        'object_T_grasp': value['object_T_tcp_nominal'],
        'source': 'current source fixture, virtual object only; no physical grasp calibration claimed'}
    file.write_text(json.dumps(certificate, indent=2)+'\n')
    # Values are software test policies. The velocity caps are one fifth of
    # the prior passing independent x/y/yaw motion envelope, never robot ratings.
    measured = json.loads((folder/'leader_safety.json').read_text())['max_base_velocity']
    limits = [v*.2 for v in measured]
    policy = {'grasp_calibration_file': str(file.resolve()), 'rotation_duration_s': 160.,
        'rotation_max_yaw_rad': math.radians(5.), 'rotation_max_angular_speed': min(.002, limits[2]),
        'rotation_max_base_speed': min(limits[:2]), 'rotation_max_grasp_speed': min(limits[:2]),
        'rotation_max_angular_acceleration': .001, 'rotation_max_base_acceleration': .001,
        'rotation_yaw_gain': 1., 'rotation_yaw_tolerance_rad': .0001,
        'rotation_position_tolerance_m': .0001, 'rotation_terminal_timeout_s': 20.,
        'rotation_tracking_limit_m': .01, 'rotation_tracking_limit_rad': .01,
        'rotation_hold_s': 1., 'rotation_arm_position_limit_m': .0005,
        'rotation_arm_angle_limit_rad': .01, 'rotation_ungrasped_test': True}
    policy_file = output/'rotation_policy.yaml'
    policy_file.write_text(yaml.safe_dump({'/leader/leader_object_rotation': {'ros__parameters': policy}}))
    return policy_file


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--delta-deg', type=float, required=True, choices=[-5., 5.])
    p.add_argument('--condition', choices=['leader_ungrasped', 'synchronized_fixed'], default='leader_ungrasped')
    a = p.parse_args()
    a.output, a.run_directory = a.output.resolve(), a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    synchronized = a.condition == 'synchronized_fixed'
    report = {'passed': False, 'condition': a.condition, 'delta_deg': a.delta_deg,
              'argv': __import__('sys').argv, 'ROS_DISTRO': os.getenv('ROS_DISTRO'),
              'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION')}
    if report['ROS_DISTRO'] != 'humble' or not report['IGN_PARTITION'] or not report['ROS_DOMAIN_ID']:
        raise ValueError('explicit Humble, isolated ROS_DOMAIN_ID and IGN_PARTITION required')
    policy_file = prepare_virtual_calibration(a.run_directory, a.output) if not synchronized else None
    profiles = {r: (a.run_directory/(r+'_safety.json')).read_bytes() for r in ('leader', 'follower')}
    for robot, data in profiles.items():
        profile = json.loads(data)
        profile['require_grasp_for_base_motion'] = synchronized
        profile['ungrasped_base_test_only'] = not synchronized
        (a.run_directory/(robot+'_safety.json')).write_text(json.dumps(profile, indent=2)+'\n')
        (a.output/(robot+'_safety_used.json')).write_text(json.dumps(profile, indent=2)+'\n')
    code = Path(__file__).resolve().parents[1]/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    (a.output/'source_sha256.json').write_text(json.dumps({str(f): hashlib.sha256(f.read_bytes()).hexdigest()
        for f in [*code.glob('*.py'), Path(__file__), Path(__file__).with_name('amir_cp8r1_sync_reference.py')]}, indent=2)+'\n')
    processes, logs = [], []

    def start(name, command):
        log = (a.output/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    rclpy.init(args=[])
    node = rclpy.create_node('cp8r1_evaluator')
    latest, states, gates, guards, poses, outputs = {}, {}, {}, {}, {}, {}
    now, safety, ref, rows, phase = [0.], {}, {}, [], ['startup']
    grasp_states = {}
    models = {r: LocalModel((a.run_directory/(r+'.urdf')).read_text()) for r in ('leader', 'follower')}
    initial_object_grasps = {}
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
            lambda msg, r=robot: grasp_states.__setitem__(r, msg.data), 10))
    subs.append(node.create_subscription(Clock, '/clock', lambda msg: now.__setitem__(0,
        msg.clock.sec+msg.clock.nanosec*1e-9), qos_profile_sensor_data))
    subs.append(node.create_subscription(String, '/transport/safety_status', lambda msg: safety.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/leader/rotation/debug', lambda msg: ref.update(json.loads(msg.data)), 10))
    if synchronized:
        subs.append(node.create_subscription(String, '/experiment/sync_leader/debug', lambda msg: ref.update(json.loads(msg.data)), 10))

    def gt(msg):
        for t in msg.transforms:
            if t.child_frame_id in ('leader', 'follower', 'cooperative_payload'):
                poses[t.child_frame_id] = pose_matrix(t.transform.translation, t.transform.rotation)

    subs.append(node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', gt, qos_profile_sensor_data))
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    goal = node.create_publisher(Float64, '/leader/object_rotation_delta', 1)
    sync_goal = node.create_publisher(Float64, '/experiment/sync_start', 1)
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
                if synchronized and robot in poses and 'cooperative_payload' in poses:
                    local_grasp = sensor.copy()
                    local_grasp[:3, 3] = tcp
                    world_grasp = poses[robot]@local_grasp
                    if robot in initial_object_grasps:
                        desired = poses['cooperative_payload']@initial_object_grasps[robot]
                        row[robot+'_grasp_constraint_position_error_m'] = float(np.linalg.norm(world_grasp[:3, 3]-desired[:3, 3]))
        if 'center_odom_xyz' in ref:
            for i, k in enumerate('xyz'):
                row['center_odom_'+k] = ref['center_odom_xyz'][i]
        if 'cooperative_payload' in poses and synchronized:
            for i, k in enumerate('xyz'):
                row['object_center_GT_'+k] = float(poses['cooperative_payload'][i, 3])
            row['object_yaw_GT_rad'] = math.atan2(poses['cooperative_payload'][1, 0], poses['cooperative_payload'][0, 0])
            row['object_vertical_axis_dot_z'] = float(poses['cooperative_payload'][2, 1])
        rows.append(row)
        raw.write(json.dumps(row)+'\n')

    def wait(predicate, timeout=60., check_fault=False):
        deadline = time.monotonic()+timeout
        while not predicate():
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
        if not synchronized:
            remove('cooperative_payload')
            remove('grasp_support')
        start('safety', ['ros2', 'launch', 'amir_compliance_control', 'guarded_safety.launch.py',
                         'run_directory:='+str(a.run_directory)])
        if synchronized:
            for robot in ('leader', 'follower'):
                start(robot+'_evaluation_reference', ['python3', str(Path(__file__).with_name('amir_cp8r1_sync_reference.py')),
                    '--robot', robot, '--ros-args', '-p', 'use_sim_time:=true', '-p', 'actuator_ack_required:=true',
                    '-p', 'urdf_file:='+str(a.run_directory/(robot+'.urdf')),
                    '-p', 'schedule_file:='+str(a.output/'synchronized_schedule.json')])
        else:
            start('leader', ['ros2', 'launch', 'amir_compliance_control', 'object_rotation_leader.launch.py',
                             'run_directory:='+str(a.run_directory), 'rotation_policy_file:='+str(policy_file)])
            start('follower_hold', ['ros2', 'run', 'amir_compliance_control', 'follower_hold', '--ros-args',
                '-r', '__ns:=/follower', '-p', 'use_sim_time:=true', '-p', 'actuator_ack_required:=true',
                '-p', 'urdf_file:='+str(a.run_directory/'follower.urdf')])
        wait(lambda: safety.get('ready') and all(states.get(r, {}).get('local_ready') for r in ('leader', 'follower')), 45.)
        service('/leader/transport/prepare')
        service('/follower/transport/prepare')
        service('/transport/safety/arm')
        phase[0] = 'home'
        wait(lambda: all(states.get(r, {}).get('state') == 'READY' for r in ('leader', 'follower')), 20., True)
        if synchronized:
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
        phase[0] = 'baseline'
        settle(1.)
        origin = {r: poses[r].copy() for r in ('leader', 'follower')}
        object_start = poses.get('cooperative_payload', np.eye(4)).copy()
        if synchronized:
            # Explicit GT-based INITIAL schedule for this evaluation comparison
            # only. It is never loaded by ObjectRotationLeader or local follower.
            obj = object_start.copy()
            angle = math.atan2(obj[1, 0], obj[0, 0])
            from amir_compliance_control.wrench import rotation
            obj[:3, :3] = rotation([0., 0., 1.], angle)
            schedule = {'duration_s': 160., 'delta_rad': math.radians(a.delta_deg),
                        'scope': 'EVALUATION_ONLY_INITIAL_GT_SYNCHRONIZED_COMPARISON'}
            for robot in ('leader', 'follower'):
                own = latest[robot, 'encoder'].pose.pose
                odom_b = pose_matrix(own.position, own.orientation)
                world_odom = origin[robot]@inverse_transform(odom_b)
                local_obj = inverse_transform(world_odom)@obj
                local_yaw = math.atan2(local_obj[1, 0], local_obj[0, 0])
                local_obj[:3, :3] = rotation([0., 0., 1.], local_yaw)
                schedule[robot] = {'object0_odom': local_obj.tolist()}
                j = latest[robot, 'joints']
                mapping = dict(zip(j.name, j.position))
                tcp, sensor = models[robot].frames([mapping[n] for n in models[robot].names])
                sensor[:3, 3] = tcp
                initial_object_grasps[robot] = inverse_transform(object_start)@origin[robot]@sensor
                # Save a full, static metrology calibration for a subsequent
                # leader-only test. Runtime control receives only this constant
                # transform, never the evaluator's ongoing GT observations.
                object_grasp = inverse_transform(local_obj)@odom_b@sensor
                certificate = {'robot': robot, 'grasp_frame': 'tcp_link', 'base_frame': 'base_footprint',
                    'object_frame': 'geometric_center_z_up', 'scope': 'fixed_fixture_verified',
                    'calibration_verified': True, 'object_T_grasp': object_grasp.tolist(),
                    'urdf_sha256': hashlib.sha256((a.run_directory/(robot+'.urdf')).read_bytes()).hexdigest(),
                    'source': 'offline static simulator metrology after fixed attachment and support removal; evaluation only',
                    'snapshot_sim_s': now[0], 'odom_T_base_snapshot': odom_b.tolist(),
                    'base_T_grasp_snapshot': sensor.tolist(), 'object_pose_odom_snapshot': local_obj.tolist(),
                    'joint_state_stamp_s': j.header.stamp.sec+j.header.stamp.nanosec*1e-9,
                    'joint_positions_rad': [mapping[n] for n in models[robot].names]}
                from amir_compliance_control.object_center_rotation_trajectory import initialize_object_pose
                reconstructed = initialize_object_pose(odom_b, sensor, object_grasp)
                certificate['reconstruction_max_element_error'] = float(np.max(abs(reconstructed-local_obj)))
                (a.output/(robot+'_fixed_grasp_calibration.json')).write_text(json.dumps(certificate, indent=2)+'\n')
            (a.output/'synchronized_schedule.json').write_text(json.dumps(schedule, indent=2)+'\n')
            sync_goal.publish(Float64(data=now[0]+.5))
        else:
            goal.publish(Float64(data=math.radians(a.delta_deg)))
        phase[0] = 'rotation'
        wait(lambda: states.get('leader', {}).get('accepted_goals') == 1, 5., True)
        wait(lambda: states.get('leader', {}).get('completed_goals') == 1, 1000., True)
        phase[0] = 'hold'
        settle(2.)
        movement = {r: inverse_transform(origin[r])@poses[r] for r in origin}
        gt_angle = math.atan2(movement['leader'][1, 0], movement['leader'][0, 0])
        report['leader_GT_delta_yaw_rad'] = gt_angle
        report['leader_GT_delta_xyz_m'] = movement['leader'][:3, 3].tolist()
        report['follower_GT_delta_xyz_m'] = movement['follower'][:3, 3].tolist()
        report['leader_encoder_angle_error_rad'] = ref.get('yaw_error_rad')
        report['gt_angle_ratio'] = gt_angle/math.radians(a.delta_deg)
        report['motion_completed_in_own_odom'] = True
        report['passed'] = bool(.7 <= report['gt_angle_ratio'] <= 1.3 and
                               (synchronized or np.linalg.norm(movement['follower'][:3, 3]) < .001))
        if synchronized:
            object_delta = inverse_transform(object_start)@poses['cooperative_payload']
            # Object P has rolled axes: use world long-axis heading directly.
            object_yaw = math.atan2(poses['cooperative_payload'][1, 0], poses['cooperative_payload'][0, 0])
            object_yaw0 = math.atan2(object_start[1, 0], object_start[0, 0])
            object_angle = math.atan2(math.sin(object_yaw-object_yaw0), math.cos(object_yaw-object_yaw0))
            report['object_GT_delta_yaw_rad'] = object_angle
            report['object_center_drift_m'] = float(np.linalg.norm(object_delta[:3, 3]))
            report['evaluation_only_shared_reference'] = True
            report['passed'] = bool(report['passed'] and states.get('follower', {}).get('completed_goals') == 1
                and .7 <= object_angle/math.radians(a.delta_deg) <= 1.3 and report['object_center_drift_m'] < .001)
        if not report['passed']:
            report['error'] = 'encoder completion did not meet independent GT sign/angle or stationary-follower criterion'
    except Exception as exc:
        report['error'], report['error_phase'] = str(exc), phase[0]
    finally:
        try:
            service('/transport/safety/stop')
            phase[0] = 'safe_stop'
            settle(.3, False)
            report['both_outputs_zero_after_stop'] = all(r in outputs and max(abs(v) for v in outputs[r]) < 1e-12 for r in ('leader', 'follower'))
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
        leader_inputs = graph.get('/leader/leader_object_rotation', {}).get('subscriptions', [])
        report['leader_input_boundary_passed'] = (bool(leader_inputs) and not any(
            t.startswith(('/follower/', '/experiment/', '/world/')) or 'ground_truth' in t or '/tf' in t for t, _ in leader_inputs)) if not synchronized else (
                '/leader/evaluation_synchronized_reference' in graph and '/follower/evaluation_synchronized_reference' in graph
                and '/follower/follower_whole_body' not in graph and '/follower/follower_arm_base' not in graph)
        report['passed'] = bool(report['passed'] and report['leader_input_boundary_passed'] and report.get('both_outputs_zero_after_stop'))
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
        for log in logs:
            log.close()
        for robot, data in profiles.items():
            (a.run_directory/(robot+'_safety.json')).write_bytes(data)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
