#!/usr/bin/env python3
"""CP-6 isolated evaluator: local recentering signs, static grasp, and leader-only transport."""
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
from geometry_msgs.msg import Twist, TwistStamped, Wrench, WrenchStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity
from ros_gz_interfaces.msg import Entity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from amir_compliance_control.wrench import LocalModel, at_grasp
from amir_compliance_control.transport import straight_reference as reference
import xml.etree.ElementTree as ET


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--calibration-file', type=Path, required=True)
    p.add_argument('--fault', choices=['none', 'dropout', 'preprocessor', 'clock'], default='none')
    p.add_argument('--mode', choices=['signs_xy', 'signs_yaw', 'static', 'transport'], default='transport')
    p.add_argument('--distance', type=float, default=.0001)
    p.add_argument('--speed', type=float, default=.00005)
    a = p.parse_args()
    if not 0 < a.distance <= .001 or not 0 < a.speed <= .00005:
        raise ValueError('CP-6 trial envelope: positive distance <=1 mm and speed <=.05 mm/s')
    a.calibration_file = a.calibration_file.resolve()
    a.output = a.output.resolve()
    a.run_directory = a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'profiles_used').mkdir()
    for robot in ('leader','follower'):
        (a.output/'profiles_used'/(robot+'_safety.json')).write_bytes((a.run_directory/(robot+'_safety.json')).read_bytes())
    code_root=Path(__file__).resolve().parent.parent/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    (a.output/'control_source_hashes.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest()
        for f in code_root.glob('*.py')},indent=2))
    (a.output/'command.json').write_text(json.dumps({'argv': __import__('sys').argv,
        'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION'),
        'ROS_LOCALHOST_ONLY': os.getenv('ROS_LOCALHOST_ONLY'),
        'condition': 'CP6_LOCAL_ARM_BASE'}, indent=2))
    processes, logs, suspended = [], [], []
    def start(name, argv):
        file = (a.output/(name+'.log')).open('w')
        logs.append(file)
        process = subprocess.Popen(argv, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp6_fixture.launch.py', 'run_directory:='+str(a.run_directory)])
    rclpy.init(args=[])
    node = rclpy.create_node('cp6_local_arm_base_evaluator')
    robots = ('leader', 'follower')
    now, latest, poses, statuses, debug = [0.], {}, {}, {}, {}
    subscriptions, endpoint_states, adm_state, filter_state, base_state = [], {}, {}, [''], {}
    grasp_state = {}
    attach = {r: node.create_publisher(Empty, '/'+r+'/grasp/attach', 1) for r in robots}
    goal_pub = node.create_publisher(Vector3Stamped, '/leader/transport_goal', 1)
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    excitation = node.create_publisher(Wrench, '/experiment/follower/calibration_wrench', 1)
    applied, last_excitation = [0.]*6, [0.]
    actual_commands, actual_twists, requested, motion_origins = {}, {}, {}, {}
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
        actual_twists[robot] = [msg.linear.x, msg.linear.y, msg.angular.z]
        output_log.write(json.dumps({'wall_time': time.monotonic(), 'sim_time': now[0],
            'robot': robot, 'vx': msg.linear.x, 'vy': msg.linear.y, 'wz': msg.angular.z})+'\n')

    for robot in robots:
        subscriptions.append(node.create_subscription(String, '/'+robot+'/grasp/state',
            lambda m,r=robot: grasp_state.__setitem__(r,m.data), 10))
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
    subscriptions += [node.create_subscription(String, '/follower/recentering/debug', lambda m: base_state.update(json.loads(m.data)), 10),
        node.create_subscription(String, '/follower/admittance/debug', lambda m: adm_state.update(json.loads(m.data)), 10),
        node.create_subscription(String, '/follower/wrench_status', lambda m: filter_state.__setitem__(0, m.data), 10),
        node.create_subscription(Clock, '/clock', lambda m: now.__setitem__(0,
        m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', payload_pose, qos_profile_sensor_data),
        node.create_subscription(String, '/transport/safety_status', lambda m: statuses.update(json.loads(m.data)), 10)]
    control = node.create_client(ControlWorld, '/world/cooperative_transport/control')
    report = {'passed': False, 'condition': a.mode, 'fault_test': a.fault, 'distance_m':a.distance, 'speed_m_s':a.speed}

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
                row[robot+'_encoder_y'] = encoder.pose.pose.position.y
                quat = encoder.pose.pose.orientation
                row[robot+'_encoder_yaw'] = float(np.arctan2(2*(quat.w*quat.z+quat.x*quat.y),1-2*(quat.y**2+quat.z**2)))
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
            for i,k in enumerate(('x','y','yaw')):
                row['measured_error_'+k] = (base_state.get('measured_error_xy_yaw') or [0.,0.,0.])[i]
                row['low_error_'+k] = (base_state.get('low_error_xy_yaw') or [0.,0.,0.])[i]
                row['base_request_'+k] = (base_state.get('base_twist') or [0.,0.,0.])[i]
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

    def start_transport():
        phase[0]='baseline';sim_wait(1.)
        origin={k:poses[k].translation.x for k in robots+('cooperative_payload',)}
        phase[0]='motion'
        goal=Vector3Stamped()
        goal.header.stamp,goal.header.frame_id=Time(seconds=now[0]).to_msg(),'leader/base_footprint'
        goal.vector.x=a.distance
        goal_pub.publish(goal)
        (a.output/'goal.json').write_text(json.dumps({'topic':'/leader/transport_goal',
            'relative_x_m':a.distance,'stamp_s':now[0]},indent=2))
        wait(lambda:endpoint_states['leader']['accepted_goals']==1,5.,check_fault=True)
        return origin,reference(0.,a.distance,a.speed,.001)[2]

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
            if bool(profile.get('ungrasped_base_test_only')) != a.mode.startswith('signs'):
                raise RuntimeError('profile grasp policy does not match experiment mode')
            models[robot] = LocalModel(xml)
        start('safety', ['ros2', 'launch', 'amir_compliance_control', 'safety.launch.py', 'run_directory:='+str(a.run_directory)])
        bridge_args = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/cooperative_transport/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
        for robot in robots:
            bridge_args += [f'/{robot}/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                            f'/{robot}/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg']
        start('evaluation', bridge_args)
        wait(lambda: statuses.get('ready') and control.service_is_ready() and remover.service_is_ready()
             and all(r in grasp_state for r in robots), 45.)
        start('transport', ['ros2', 'launch', 'amir_compliance_control', 'arm_base.launch.py', 'ungrasped_test:='+str(a.mode.startswith('signs')).lower(),
            'leader_speed:='+str(a.speed), 'base_gains:='+('0.5,0.5,0.0' if a.mode=='signs_xy' else '0.0,0.0,1.0' if a.mode=='signs_yaw' else '0.5,0.5,1.0'),
            'run_directory:='+str(a.run_directory), 'calibration_file:='+str(a.calibration_file)])
        wait(lambda: all(endpoint_states.get(r, {}).get('local_ready') for r in robots), 20.)
        parameters=node.create_client(GetParameters,'/follower/follower_arm_base/get_parameters')
        wait(parameters.service_is_ready,10.)
        names=['virtual_mass','virtual_damping','virtual_stiffness','displacement_limit_m',
               'base_cutoff_hz','base_gains','base_velocity_limits','base_acceleration_limits',
               'preserve_orientation','straight_only','allow_ungrasped_test']
        future=parameters.call_async(GetParameters.Request(names=names));wait(future.done,10.)
        (a.output/'control_policy.json').write_text(json.dumps({k:parameter_value_to_python(v)
            for k,v in zip(names,future.result().values)},indent=2))
        node.destroy_client(parameters)
        for robot in robots:
            service('/'+robot+'/transport/prepare', retry_discovery=True)
        service('/transport/safety/arm', retry_discovery=True)
        wait(lambda: statuses.get('armed'), 10., check_fault=True)
        phase[0] = 'local_home'
        wait(lambda: all(endpoint_states[r]['state'] == 'READY' for r in robots), 15., check_fault=True)
        if not a.mode.startswith('signs'):
            phase[0] = 'pre_grasp'
            sim_wait(1.)
            baseline = np.mean([[r[robot+'_fz'] for robot in robots] for r in rows if r['phase']=='pre_grasp'],axis=0)
            for _ in range(15):
                for pub in attach.values():pub.publish(Empty())
                wall_wait(.05)
            wait(lambda: all(grasp_state.get(r)=='attached' for r in robots),10.,check_fault=True)
            req=DeleteEntity.Request();req.entity.name,req.entity.type='grasp_support',Entity.MODEL
            f=remover.call_async(req);wait(f.done,10.,check_fault=True)
            if not f.result().success:raise RuntimeError('support removal failed')
            phase[0]='static_hold';sim_wait(10.)
            hold=[r for r in rows if r['phase']=='static_hold']
            late=[r for r in hold if r['sim_time']>=hold[-1]['sim_time']-8.]
            weight=-sum(np.mean([[r[robot+'_fz'] for robot in robots] for r in late],axis=0)-baseline)
            world=ET.parse(a.run_directory/'world.sdf')
            mass=sum(float(v.text) for v in world.findall("world/model[@name='cooperative_payload']/link/inertial/mass"))
            gravity=abs(float(world.findtext('world/gravity').split()[2]))
            drift=[float(np.ptp([r['payload_'+k] for r in late])) for k in 'xyz']
            report['static_check']={'mass_kg':mass,'expected_weight_N':mass*gravity,'measured_weight_N':float(weight),'late_drift_xyz_m':drift}
            if abs(weight-mass*gravity)>.1 or max(drift)>.0001:raise RuntimeError('compliant grasp static support test failed')
        phase[0] = 'tare_settle'
        sim_wait(1.)
        service('/follower/tare_wrench', retry_discovery=True)
        wait(lambda: filter_state[0] == 'READY', 5., check_fault=True)
        sim_wait(.2)
        if a.mode != 'static':
            service('/follower/admittance/enable')
            wait(lambda: base_state.get('enabled'), 5., check_fault=True)
        if a.fault == 'none':
            if a.mode.startswith('signs'):
                phases = [('baseline',[0.,0.],1.)]
                axes = [('fx',0),('fy',1)] if a.mode=='signs_xy' else [('yaw',1)]
                for label,index in axes:
                    for direction in [1.,-1.]:
                        command=[0.,0.];command[index]=direction*.1
                        phases += [(label+str(direction),command,4.),('release_'+label+str(direction),[0.,0.],8.)]
                results=[]
                for name,command,duration in phases:
                    phase[0],applied[:2]=name,command;sim_wait(duration)
                    if name.startswith(('fx','fy','yaw')):
                        selected=[r for r in rows if r['phase']==name]
                        key='x' if name.startswith('fx') else 'y' if name.startswith('fy') else 'yaw'
                        direction=1. if name.endswith('1.0') and not name.endswith('-1.0') else -1.
                        mean=float(np.mean([r['base_request_'+key] for r in selected[-10:]]))
                        delta=selected[-1]['follower_encoder_'+key]-selected[0]['follower_encoder_'+key]
                        if key=='yaw':delta=float(np.arctan2(np.sin(delta),np.cos(delta)))
                        passed=direction*mean>1e-5 and direction*delta>1e-5
                        results.append({'phase':name,'mean_requested_component':mean,'encoder_delta':delta,'passed':bool(passed)})
                report['sign_tests']=results
                if not all(r['passed'] for r in results):raise RuntimeError('base recentering sign check failed')
            elif a.mode=='transport':
                origin,duration=start_transport()
                wait(lambda:endpoint_states['leader']['completed_goals']==1,duration+8.,check_fault=True)
                phase[0]='settle';sim_wait(6.)
                report['gt_distance_m']={k:poses[k].translation.x-origin[k] for k in origin}
                late=[r for r in rows if r['phase']=='settle' and r['sim_time']>=rows[-1]['sim_time']-1.]
                report['final_arm_offset_x_m']=float(max(abs(r['measured_error_x']) for r in late))
                report['final_follower_speed_m_s']=float(max(abs(r['follower_output_vx']) for r in late))
                if (min(report['gt_distance_m'].values())<.7*a.distance or max(report['gt_distance_m'].values())>1.3*a.distance
                        or report['final_arm_offset_x_m']>.0001 or report['final_follower_speed_m_s']>.00001):
                    raise RuntimeError('local follower transport or final settling check failed')
                if endpoint_states['follower']['accepted_goals']!=0:raise RuntimeError('follower received a goal')
        else:
            if a.mode=='transport':
                start_transport()
                sim_wait(3.)
                wait(lambda:all(abs(actual_commands.get(r,0.))>5e-6 for r in robots),5.,check_fault=True)
            else:
                phase[0], applied[:2] = 'force_before_fault', [.1, 0.]
                sim_wait(.4)
            phase[0] = 'fault_injected'
            report['twists_at_injection'] = actual_twists.copy()
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
            wait(lambda: all(r in actual_twists and max(abs(v) for v in actual_twists[r]) < 1e-12 for r in robots)
                 and all(not debug.get(r, {}).get('active', True) for r in robots), 2.)
            report['dual_zero_latency_wall_s'] = time.monotonic()-injection
            if report['dual_zero_latency_wall_s'] > .4:
                raise RuntimeError('dual stop exceeded experiment wall-time budget')
            if a.fault in ('dropout', 'preprocessor'):
                os.kill(suspended.pop(), signal.SIGCONT)
            if a.fault == 'clock':
                pause(False)
            wall_wait(.5)
            if any(max(abs(v) for v in actual_twists.get(r,[1.])) > 1e-12 for r in robots):
                raise RuntimeError('motion resumed automatically after fault recovery')
            if any(max(abs(latest[(r, 'encoder')].twist.twist.linear.x),
                       abs(latest[(r, 'encoder')].twist.twist.linear.y)) > 1e-6
                   or abs(latest[(r, 'encoder')].twist.twist.angular.z) > 1e-5 for r in robots):
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
        report['last_base_state']=base_state
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
        required = ('/follower/follower_arm_base', '/follower/wrench_preprocessor', '/follower/safety_gate')
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
        raise RuntimeError('CP-6 graph boundary or experiment validation failed')


if __name__ == '__main__':
    main()
