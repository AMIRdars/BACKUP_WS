#!/usr/bin/env python3
"""CP-13 native contact 1 m forward / 1 m return simulation evaluator.

Derived from the unchanged CP-9 acceptance harness. Preparation, safety
contracts and teardown use the same runtime components; only this separate
experiment sends two sequential, signed goals to the leader.

The production controller and all watchdog thresholds remain unchanged.
Calibration uses a separately exited scene; this scene has native contact only.
"""
import argparse
import csv
import fcntl
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
from geometry_msgs.msg import Twist, TwistStamped, Wrench, WrenchStamped, Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity
from ros_gz_interfaces.msg import Entity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from amir_compliance_control.wrench import LocalModel, at_grasp
from amir_compliance_control.transport import straight_reference as reference
import xml.etree.ElementTree as ET
from cp9_trial_progress import TrialProgress


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--metrology-directory', type=Path, required=True)
    p.add_argument('--fault', choices=['none','nonfinite'], default='none',
        help='Inject invalid local F/T only while moving backwards')
    p.add_argument('--distance', type=float, choices=[1.], default=1.)
    p.add_argument('--speed', type=float, choices=[.1], default=.1,
        help='Speed cap; existing .005 m/s2 reference makes each 1 m leg triangular')
    p.add_argument('--base-speed-evidence', type=Path, required=True)
    a = p.parse_args()
    a.mode, a.axis, a.control_mode = 'transport', 'forward', 'WHOLE_BODY'
    a.angle_rad, a.delay, a.yaw_profile = .0002, 0., 'filtered'
    a.settle_seconds, a.translation_profile = 12., 'speed_trial'
    a.experimental_speed_profile = a.grip_maintenance = a.display_scene = True
    a.metrology_directory = a.metrology_directory.resolve()
    a.calibration_file = a.metrology_directory/'calibration/calibration.json'
    if os.getenv('ROS_LOCALHOST_ONLY')!='1' or not os.getenv('IGN_PARTITION','').startswith('amir_cp9_') or not os.getenv('ROS_DOMAIN_ID'):
        raise ValueError('isolated CP-9 simulation required')
    a.output = a.output.resolve()
    a.run_directory = a.run_directory.resolve()
    scene_lock = (a.run_directory/'.cp9_scene.lock').open('a')
    fcntl.flock(scene_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    a.output.mkdir(parents=True, exist_ok=False)
    saved_profiles = {r: (a.run_directory/(r+'_safety.json')).read_bytes()
                      for r in ('leader','follower')} if a.experimental_speed_profile else {}
    code_root=Path(__file__).resolve().parent.parent/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    (a.output/'control_source_hashes.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest()
        for f in code_root.glob('*.py')},indent=2))
    trial_sources=[Path(__file__),Path(__file__).with_name('round_trip_evidence.py'),
        Path(__file__).with_name('round_trip_safety_gate.py'),
        Path(__file__).resolve().parent.parent/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip_safety.launch.py',
        Path(__file__).resolve().parent.parent/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip_scene.launch.py',
        Path(__file__).with_name('cp9_transport_preparation.py'),Path(__file__).with_name('amir_cp9_grasp_preparation.py'),Path(__file__).with_name('cp9_trial_progress.py')]
    if a.experimental_speed_profile:
        trial_sources.append(Path(__file__).with_name('cp10_speed_evidence.py'))
    if a.display_scene:
        trial_sources.append(Path(__file__).resolve().parent.parent/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp12_display_scene.launch.py')
    (a.output/'trial_source_hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in trial_sources},indent=2))
    (a.output/'source_snapshot').mkdir()
    for f in trial_sources:
        (a.output/'source_snapshot'/f.name).write_bytes(f.read_bytes())
    (a.output/'command.json').write_text(json.dumps({'argv': __import__('sys').argv,
        'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION'),
        'ROS_LOCALHOST_ONLY': os.getenv('ROS_LOCALHOST_ONLY'),
        'RMW_FASTRTPS_PUBLICATION_MODE': os.getenv('RMW_FASTRTPS_PUBLICATION_MODE'),
        'condition': 'CP13_ROUND_TRIP_NATIVE_GUARDED_LOCAL'}, indent=2))
    processes, logs, suspended = [], [], []
    def start(name, argv):
        file = (a.output/(name+'.log')).open('w')
        logs.append(file)
        process = subprocess.Popen(argv, stdout=file, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    # Keep the ROS context live while Ctrl+C runs the stop/child cleanup below.
    # The default rclpy SIGINT handler shuts it down before finally can use it.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('cp9_transport_evaluator')
    robots = ('leader', 'follower')
    now, latest, poses, statuses, debug = [0.], {}, {}, {}, {}
    subscriptions, endpoint_states, adm_state, filter_state, base_state = [], {}, {}, [''], {}
    grasp_state = {}
    prep, monitor, maintenance = {}, {}, {}
    goal_pub = node.create_publisher(PoseStamped, '/leader/transport_pose_goal', 1)
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    applied, last_excitation = [0.]*6, [0.]
    actual_commands, actual_twists, requested, motion_origins = {}, {}, {}, {}
    interlocks={};controller_states={};interlock_received={}
    for robot in robots:
        if a.grip_maintenance:
            subscriptions.append(node.create_subscription(String,'/'+robot+'/grasp/maintenance_debug',
                lambda m,r=robot: maintenance.__setitem__(r,json.loads(m.data)),10))
        subscriptions.append(node.create_subscription(String,'/'+robot+'/grasp/debug',lambda m,r=robot: prep.__setitem__(r,json.loads(m.data)),10))
        subscriptions.append(node.create_subscription(String,'/'+robot+'/grasp/monitor_debug',lambda m,r=robot: monitor.__setitem__(r,json.loads(m.data)),10))
        subscriptions.append(node.create_subscription(String,'/'+robot+'/interlock/audit',
            lambda m,r=robot:controller_states.__setitem__(r,json.loads(m.data).get('controllers',{})),10))
        subscriptions.append(node.create_subscription(String,'/'+robot+'/interlock/debug',
            lambda m,r=robot: interlock_event(r,m),10))
    phase, last_record = ['startup'], [0.]
    active_leg = [0]
    rows, raw_log = [], (a.output/'samples.jsonl').open('w', buffering=1)
    models = {}
    output_log = (a.output/'command_outputs.jsonl').open('w', buffering=1)
    interlock_log = (a.output/'interlock_events.jsonl').open('w', buffering=1)

    def interlock_event(robot,msg):
        value=json.loads(msg.data);interlocks[robot]=value
        interlock_received[robot]=time.monotonic()
        interlock_log.write(json.dumps({'robot':robot,'wall_time':time.monotonic(),'sim_time':now[0],**value})+'\n')

    def wrench_status(msg):
        if msg.data!=filter_state[0]:
            with (a.output/'wrench_status_events.jsonl').open('a') as log:log.write(json.dumps({'sim_time':now[0],'wall_time':time.monotonic(),'status':msg.data})+'\n')
        filter_state[0]=msg.data

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
        subscriptions.append(node.create_subscription(JointTrajectory,'/'+robot+'/arm_joint_cmd',
            lambda m,r=robot: output_log.write(json.dumps({'kind':'arm_request','robot':r,'wall_time':time.monotonic(),
                'receiver_sim':now[0],'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9})+'\n'),10))
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
        node.create_subscription(String, '/follower/wrench_status', wrench_status, 10),
        node.create_subscription(Clock, '/clock', lambda m: now.__setitem__(0,
        m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', payload_pose, qos_profile_sensor_data),
        node.create_subscription(String, '/transport/safety_status', lambda m: statuses.update(json.loads(m.data)), 10)]
    allocation_log=(a.output/'allocations.jsonl').open('w',buffering=1)
    subscriptions.append(node.create_subscription(String,'/follower/allocation/debug',lambda m:allocation_log.write(m.data+'\n'),10))
    control = node.create_client(ControlWorld, '/world/cooperative_transport/control')
    report = {'passed': False, 'condition': a.mode, 'fault_test': a.fault, 'distance_m':a.distance, 'speed_m_s':a.speed,'control_mode':a.control_mode,'axis':a.axis,'angle_rad':a.angle_rad,'yaw_profile':a.yaw_profile,'translation_profile':a.translation_profile,'settle_seconds':a.settle_seconds,'delay_s':a.delay,'native_contact_only':True,'hardware_motion_gate_open':False,'cooperative_contact_transport_verified':False}
    report['grip_maintenance_enabled'] = a.grip_maintenance
    report['legs'] = []
    report['round_trip_verified'] = False
    progress = TrialProgress(a.output, distance_m=a.distance)

    def step():
        rclpy.spin_once(node, timeout_sec=.001)
        progress.update(phase=phase[0], sim_time=now[0], preparation=prep,
            encoders={r: latest[r, 'encoder'].pose.pose.position.x
                      for r in robots if (r, 'encoder') in latest})
        wall = time.monotonic()
        if wall-last_record[0] >= .05 and all((r, k) in latest for r in robots for k in ('joints', 'wrench', 'encoder')):
            last_record[0] = wall
            row = {'sim_time': now[0], 'wall_time': wall, 'phase': phase[0], 'leg': active_leg[0], 'fault': statuses.get('fault', ''),
                   'applied_fx_N': applied[0], 'applied_fy_N': applied[1],
                   'applied_fz_N':applied[2],'applied_tx_Nm':applied[3],'applied_ty_Nm':applied[4],'applied_tz_Nm':applied[5], 'native_grasp_monitor':dict(monitor), 'grip_maintenance':dict(maintenance)}
            if adm_state.get('reference_xyz_m') is not None:
                for i, k in enumerate('xyz'): row['reference_'+k] = adm_state['reference_xyz_m'][i]
            if adm_state.get('displacement_xy_m') is not None:
                for i, k in enumerate('xy'): row['admittance_'+k] = adm_state['displacement_xy_m'][i]
            row['admittance_enabled'] = adm_state.get('enabled', False)
            row['saturated'] = any(adm_state.get('saturated_xy') or [False])
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
                row[robot+'_state'] = endpoint_states.get(robot, {}).get('state')
                for index,component in enumerate(('vx','vy','wz')):row[robot+'_output_'+component]=actual_twists.get(robot,[0.,0.,0.])[index]
                encoder = latest[(robot, 'encoder')]
                row[robot+'_encoder_stamp_s']=encoder.header.stamp.sec+encoder.header.stamp.nanosec*1e-9
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
                row['force_memory_velocity_'+k] = (base_state.get('force_memory_velocity_xy_yaw') or [0.,0.,0.])[i]
                row['stationary_hold_'+k] = (base_state.get('stationary_hold_axes') or [False,False,False])[i]
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

    def wait_sim(predicate, seconds):
        begin=now[0]
        wait(lambda:predicate() or now[0]-begin>seconds,max(30.,seconds*15),check_fault=True)
        if not predicate():raise RuntimeError('simulation-time completion budget exceeded')

    def service(name, retry_discovery=False):
        client = node.create_client(Trigger, name)
        wait(client.service_is_ready, 10.)
        deadline = time.monotonic()+5.
        while True:
            future = client.call_async(Trigger.Request())
            wait(future.done, 10.)
            result = future.result()
            startup_ack = (not statuses.get('armed') and not statuses.get('fault') and any(v in result.message
                for v in ('local actuator acknowledgement not ready','local actuator heartbeat missing',
                          'local clock/state heartbeat missing','local safety gate not ready')))
            if (result.success or not retry_discovery or time.monotonic() > deadline
                    or not (startup_ack or any(v in result.message for v in ('command writer ownership not confirmed',
                        'unverified', 'unverified preprocessor publisher', 'waiting for local sensors and gate', 'fresh stationary local samples required',
                        'retired preparation and explicit unarmed enable required')))):
                break
            wall_wait(.2)
        node.destroy_client(client)
        with (a.output/'service_results.jsonl').open('a') as log:
            log.write(json.dumps({'service': name, 'success': result.success, 'message': result.message,
                                  'wall_time': time.monotonic(), 'sim_time': now[0]})+'\n')
        if not result.success:
            raise RuntimeError(name+': '+result.message)
        return result

    def find_process(argument, namespace=None):
        candidates = []
        for path in Path('/proc').iterdir():
            if not path.name.isdigit():
                continue
            try:
                argv = (path/'cmdline').read_bytes().split(b'\0')
                env = (path/'environ').read_bytes().split(b'\0')
                if (any(argument in v for v in argv) and
                        ('ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID']).encode() in env and
                        (namespace is None or ('__ns:='+namespace).encode() in argv)):
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

    def snapshot_controllers():
        from controller_manager_msgs.srv import ListControllers
        actual_states={}
        for robot in robots:
            client=node.create_client(ListControllers,'/'+robot+'/controller_manager/list_controllers')
            try:
                wait(client.service_is_ready,3.)
                end=time.monotonic()+3.
                while time.monotonic()<end:
                    future=client.call_async(ListControllers.Request());wait(future.done,2.)
                    actual_states[robot]={c.name:c.state for c in future.result().controller}
                    if all(actual_states[robot].get(c)=='inactive' for c in ('arm_controller','mecanum_drive_controller')):break
                    wall_wait(.1)
            finally:
                node.destroy_client(client)
        return actual_states

    def gt_pose(key):
        t=poses[key];q=t.rotation
        return np.array([t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])

    def start_leg(index, signed_distance):
        phase[0]='baseline';sim_wait(1.)
        active_leg[0] = index
        # These snapshots are evaluation only; the signed goal is a constant.
        origin={k:gt_pose(k) for k in robots+('cooperative_payload',)}
        encoder_origin={r:latest[r,'encoder'].pose.pose.position.x for r in robots}
        if (statuses.get('fault') or not statuses.get('armed')
                or endpoint_states['leader']['state'] != 'READY'
                or endpoint_states['leader']['accepted_goals'] != index-1
                or endpoint_states['leader']['completed_goals'] != index-1
                or endpoint_states['follower']['accepted_goals'] != 0):
            raise RuntimeError('next leg requires armed safety, stopped leader and exact goal count')
        phase[0]='motion'
        goal=PoseStamped()
        goal.header.stamp=Time(seconds=float(now[0])).to_msg()
        goal.header.frame_id='leader/base_footprint'
        goal.pose.position.x=float(signed_distance)
        goal.pose.orientation.w=1.
        goal_pub.publish(goal)
        event={'leg':index,'topic':'/leader/transport_pose_goal',
            'relative_xy_yaw':[signed_distance,0.,0.],'stamp_s':now[0]}
        (a.output/f'goal_{index}.json').write_text(json.dumps(event,indent=2))
        print(f'[CP-13] leg {index}/2: leader relative x={signed_distance:+.3f} m',flush=True)
        wait(lambda:endpoint_states['leader']['accepted_goals']==index,5.,check_fault=True)
        return origin,encoder_origin,reference(0.,a.distance,a.speed,a.leader_acceleration)[2]

    def finish_leg(index, signed_distance, origin, encoder_origin, duration):
        wait_sim(lambda:endpoint_states['leader']['completed_goals']==index,
            duration+leader_policy['terminal_timeout_s']+5.)
        phase[0]='settle';sim_wait(a.settle_seconds)
        late=[r for r in rows if r['leg']==index and r['phase']=='settle'
            and r['sim_time']>=rows[-1]['sim_time']-1.]
        if len(late)<2:
            raise RuntimeError('insufficient terminal measurements')
        arm=float(max(math.hypot(r['measured_error_x'],r['measured_error_y']) for r in late))
        speed=float(max(np.linalg.norm([r['follower_output_vx'],r['follower_output_vy'],
            .1*r['follower_output_wz']]) for r in late))
        # Keep the original CP-9 terminal limits; GT is not used to release the next goal.
        if arm>.0001 or speed>.00001:
            raise RuntimeError('original arm/base settling limits not satisfied; no next leg')
        if (endpoint_states['follower']['accepted_goals']!=0
                or not all(monitor[r]['state']=='HOLDING' for r in robots)):
            raise RuntimeError('follower goal or native grasp boundary violated')
        from round_trip_evidence import leg_metrics
        moving=[r for r in rows if r['leg']==index and r['phase']=='motion']
        metric=leg_metrics(moving,origin,{k:gt_pose(k) for k in origin},signed_distance)
        metric.update(leg=index,final_arm_offset_xy_m=arm,final_follower_speed_m_s=speed,
            encoder_delta_x_m={r:latest[r,'encoder'].pose.pose.position.x-encoder_origin[r] for r in robots})
        report['legs'].append(metric)
        print(f'[CP-13] leg {index}/2 settled: arm={arm:.9f} m, base={speed:.9f} m/s',flush=True)

    def interrupt_once(signum, frame):
        # Terminal SIGINT and launch's forwarded SIGINT can both arrive.
        # Do not let a second delivery interrupt safety-stop/child cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt_once)
    try:
        scene_launch='compliance_round_trip_scene.launch.py'
        start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', scene_launch, 'run_directory:='+str(a.run_directory)])
        wait(lambda: now[0] > 0 and all((r, k) in latest for r in robots for k in ('joints', 'encoder', 'wrench')), 90.)
        world = ET.parse(a.run_directory/'world.sdf')
        plugins = [p.get('name','') for p in world.findall('.//plugin')]
        allowed = {'Physics','UserCommands','SceneBroadcaster','Contact','Sensors','ForceTorque','DartContactParameters'}
        if any(p.rsplit('::',1)[-1] not in allowed for p in plugins):
            raise RuntimeError('non-native world plugin present')
        report['native_world_plugins'] = plugins
        cert = json.loads(a.calibration_file.read_text())
        if cert['urdf_sha256'] != hashlib.sha256((a.run_directory/'follower.urdf').read_bytes()).hexdigest():
            raise RuntimeError('native robot differs from measured six-axis certificate')
        measured_hashes = json.loads((a.metrology_directory/'generated_hashes.json').read_text())
        for r in robots:
            for suffix in ('.urdf','_controllers.yaml','_base_controller.yaml'):
                f = a.run_directory/(r+suffix)
                if hashlib.sha256(f.read_bytes()).hexdigest() != measured_hashes[f.name]:
                    raise RuntimeError('metrology robot/controller bytes changed: '+f.name)
        motion_evidence = Path('audits/whole_body_compliance_20261019/cp1/motion_run3/motion.json')
        if a.experimental_speed_profile:
            from cp10_speed_evidence import combine_evidence
            combined = combine_evidence(a.base_speed_evidence, a.run_directory, motion_evidence)
            motion_evidence = a.output/'measured_base_envelope.json'
            motion_evidence.write_text(json.dumps(combined, indent=2))
        with (a.output/'profile_generation.log').open('w') as log:
            result = subprocess.run(['/usr/bin/python3','tools/amir_make_safety_profiles.py',
                '--probe',str(a.metrology_directory/'static_proof/runtime.json'),
                '--run-directory',str(a.run_directory),'--motion-evidence',
                str(motion_evidence)],stdout=log,stderr=subprocess.STDOUT)
        if result.returncode: raise RuntimeError('measured safety profile derivation failed')
        (a.output/'profiles_used').mkdir()
        for r in robots:
            (a.output/'profiles_used'/(r+'_safety.json')).write_bytes((a.run_directory/(r+'_safety.json')).read_bytes())
        report['generated_hashes'] = {f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in a.run_directory.iterdir() if f.is_file()}
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
        start('evaluation', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/cooperative_transport/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        wait(lambda: len(poses)==3 and control.service_is_ready() and remover.service_is_ready(),30.)
        from cp9_transport_preparation import prepare
        contacts_process = prepare(node=node, robots=robots, generated=a.run_directory, output=a.output,
            start=start, wait=wait, sim_wait=sim_wait, wall_wait=wall_wait, service=service,
            poses=poses, now=now, phase=phase, prep=prep, monitor=monitor, report=report,
            grip_maintenance=a.grip_maintenance)
        phase[0] = 'guarded_initialization'
        start('safety', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_round_trip_safety.launch.py', 'run_directory:='+str(a.run_directory)])
        wait(lambda: statuses.get('ready') and all(v['state']=='HOLDING' for v in monitor.values()),45.)
        start('transport', ['ros2','launch','amir_compliance_control','whole_body_transport.launch.py',
            'mode:='+a.control_mode,'leader_speed:='+str(a.speed),'allow_ungrasped_test:='+str(a.mode.startswith('signs')).lower(),'yaw_profile:='+a.yaw_profile,
            'leader_max_distance_m:='+str(max(.001, a.distance)),
            'translation_profile:='+a.translation_profile,
            'run_directory:='+str(a.run_directory),'calibration_file:='+str(a.calibration_file)])
        wait(lambda: all(endpoint_states.get(r, {}).get('local_ready') for r in robots), 20.)
        leader_parameters=node.create_client(GetParameters,'/leader/leader_planar_trajectory/get_parameters')
        wait(leader_parameters.service_is_ready,10.)
        leader_names=['max_distance_m','speed_m_s','acceleration_m_s2','reference_acceleration_m_s2','goal_tolerance_m','terminal_timeout_s']
        leader_future=leader_parameters.call_async(GetParameters.Request(names=leader_names))
        wait(leader_future.done,10.)
        leader_policy={k:parameter_value_to_python(v) for k,v in zip(leader_names,leader_future.result().values)}
        a.leader_acceleration=float(leader_policy['reference_acceleration_m_s2'] or leader_policy['acceleration_m_s2'])
        if not math.isfinite(a.leader_acceleration) or a.leader_acceleration<=0:
            raise RuntimeError('leader acceleration policy unavailable')
        (a.output/'leader_policy.json').write_text(json.dumps(leader_policy,indent=2))
        node.destroy_client(leader_parameters)
        follower_name='follower_whole_body' if a.control_mode=='WHOLE_BODY' else 'follower_arm_base'
        parameters=node.create_client(GetParameters,'/follower/'+follower_name+'/get_parameters')
        wait(parameters.service_is_ready,10.)
        names=['virtual_mass','virtual_damping','virtual_stiffness','displacement_limit_m',
               'base_cutoff_hz','base_gains','base_velocity_limits','base_acceleration_limits',
               'preserve_orientation','straight_only','allow_ungrasped_test','command_delay_s','monitor_pose','height_error_limit_m','rotation_error_limit_rad','actuator_ack_required']
        if a.control_mode=='WHOLE_BODY':
            names=[n for n in names if n not in ('preserve_orientation','straight_only','monitor_pose')]+['joint_acceleration_rad_s2','pose_gain_s_inv','arm_command_horizon_s','allocation_on_wrench','allocation_use_clock','virtual_yaw_inertia','virtual_yaw_damping','virtual_yaw_stiffness','yaw_displacement_limit_rad','allocation_base_filter_hz','base_integral_gains','base_force_acceleration_gains','base_force_filter_hz','base_stationary_hold','allocation_base_cost_velocity']
        future=parameters.call_async(GetParameters.Request(names=names));wait(future.done,10.)
        (a.output/'control_policy.json').write_text(json.dumps({k:parameter_value_to_python(v)
            for k,v in zip(names,future.result().values)},indent=2))
        node.destroy_client(parameters)
        if a.experimental_speed_profile or a.grip_maintenance:
            # All participants/entities are now created. Settle DDS discovery
            # while unarmed; retain the original active heartbeat watchdogs.
            wall_wait(2.)
        if a.grip_maintenance:
            # Preparation has retired and every runtime participant exists.
            # Enable the keeper only after unarmed discovery has settled.
            phase[0] = 'unarmed_readiness'
            stable_since = [None]
            def stable_unarmed():
                wall = time.monotonic()
                good = statuses.get('ready') and not statuses.get('fault') and all(
                    monitor.get(r,{}).get('healthy') and endpoint_states.get(r,{}).get('local_ready')
                    and interlocks.get(r,{}).get('ready') and not interlocks[r].get('fault')
                    and all(interlocks[r].get('ages_wall_s',{}).get(k,math.inf)
                        +wall-interlock_received.get(r,0.) < (.2 if k=='audit' else .1)
                        for k in ('joint','clock','base','relay','gate','audit')) for r in robots)
                stable_since[0] = wall if good and stable_since[0] is None else stable_since[0] if good else None
                return stable_since[0] is not None and wall-stable_since[0] >= 1.
            wait(stable_unarmed, 30.)
            (a.output/'prearm_readiness.json').write_text(json.dumps({'stable_window_wall_s':1.,
                'sim_time':now[0], 'interlocks':interlocks},indent=2))
            wait(lambda: len(monitor) == 2 and all(v['healthy'] for v in monitor.values()), 5.)
            for robot in robots:
                service('/'+robot+'/grasp/enable_maintenance', retry_discovery=True)
            wait(lambda: len(maintenance) == 2 and all(v['enabled'] for v in maintenance.values()), 5.)
        for robot in robots:
            service('/'+robot+'/transport/prepare', retry_discovery=True)
        service('/transport/safety/arm', retry_discovery=True)
        wait(lambda: statuses.get('armed'), 10., check_fault=True)
        phase[0] = 'local_home'
        wait(lambda: all(endpoint_states[r]['state'] == 'READY' for r in robots), 15., check_fault=True)
        phase[0] = 'remove_native_support'
        baseline = np.mean([[r[robot+'_fz'] for robot in robots] for r in rows if r['phase']=='pre_grasp'],axis=0)
        req=DeleteEntity.Request();req.entity.name,req.entity.type='grasp_support',Entity.MODEL
        f=remover.call_async(req);wait(f.done,10.,check_fault=True)
        if not f.result().success:raise RuntimeError('support removal failed')
        phase[0]='static_hold';sim_wait(10.)
        hold=[r for r in rows if r['phase']=='static_hold']
        late=[r for r in hold if r['sim_time']>=hold[-1]['sim_time']-8.]
        weight=-sum(np.mean([[r[robot+'_fz'] for robot in robots] for r in late],axis=0)-baseline)
        mass=sum(float(v.text) for v in world.findall("world/model[@name='cooperative_payload']/link/inertial/mass"))
        gravity=abs(float(world.findtext('world/gravity').split()[2]))
        drift=[float(np.ptp([r['payload_'+k] for r in late])) for k in 'xyz']
        report['static_check']={'mass_kg':mass,'expected_weight_N':mass*gravity,'measured_weight_N':float(weight),'late_drift_xyz_m':drift}
        if abs(weight-mass*gravity)>.1 or max(drift)>.0001 or not all(v['state']=='HOLDING' for v in monitor.values()):
            raise RuntimeError('native grasp static support test failed')
        phase[0] = 'tare_settle'
        sim_wait(1.)
        service('/follower/tare_wrench', retry_discovery=True)
        wait(lambda: filter_state[0] == 'READY', 5., check_fault=True)
        sim_wait(.2)
        if a.mode != 'static':
            service('/follower/admittance/enable')
            wait(lambda: base_state.get('enabled'), 5., check_fault=True)
        # Only this external caller receives the route. Follower APIs and
        # subscriptions remain untouched; no peer pose/TF/goal is transmitted.
        origin,encoder_origin,duration=start_leg(1,a.distance)
        round_origin={k:v.copy() for k,v in origin.items()}
        finish_leg(1,a.distance,origin,encoder_origin,duration)
        if a.fault == 'nonfinite':
            bad_pub=node.create_publisher(WrenchStamped,'/follower/finger_left_wrench',10)
            wait(lambda:bad_pub.get_subscription_count()>0,5.,check_fault=True)
            wall_wait(.2)
        origin,encoder_origin,duration=start_leg(2,-a.distance)
        if a.fault == 'none':
            finish_leg(2,-a.distance,origin,encoder_origin,duration)
            report['round_trip_net_gt_xy_yaw']={k:(gt_pose(k)-v).tolist() for k,v in round_origin.items()}
            report['final_arm_offset_xy_m']=report['legs'][-1]['final_arm_offset_xy_m']
            report['final_follower_speed_m_s']=report['legs'][-1]['final_follower_speed_m_s']
            report['leader_completed_goals_before_teardown']=endpoint_states['leader']['completed_goals']
            report['follower_accepted_goals_before_teardown']=endpoint_states['follower']['accepted_goals']
            # GT acceptance is evaluated after both goals, never as a reference.
            for leg in report['legs']:
                if any(not .7<=b['tracking_ratio']<=1.3 for b in leg['bodies'].values()):
                    raise RuntimeError('signed displacement failed independent GT evaluation')
            report['round_trip_verified']=True
            report['cooperative_contact_transport_verified']=True
        else:
            # The opposing follower's local x axis is reversed. Confirm each
            # encoder in its OWN command axes; only the leader's goal is -x.
            wait_sim(lambda:actual_commands.get('leader',0.)<-.01
                and latest['leader','encoder'].twist.twist.linear.x<-.01
                and all(abs(actual_commands.get(r,0.))>.01
                    and abs(latest[r,'encoder'].twist.twist.linear.x)>.01
                    and latest[r,'encoder'].twist.twist.linear.x*actual_commands[r]>0
                    for r in robots),duration/2+5.)
            sim_wait(1.)
            phase[0]='fault_injected'
            report['fault_leg']='return'
            report['twists_at_injection']={r:list(v) for r,v in actual_twists.items()}
            report['measured_encoder_vx_at_injection']={r:latest[r,'encoder'].twist.twist.linear.x for r in robots}
            report['goals_at_injection']=endpoint_states['leader']['accepted_goals']
            report['fault_injected']=True
            injection=time.monotonic()
            end=time.monotonic()+.4
            while time.monotonic()<end:
                bad=WrenchStamped();bad.wrench.force.x=float('nan')
                bad.header.frame_id='follower/left_inner_front_joint/finger_left_force_sensor'
                bad.header.stamp=Time(seconds=float(now[0])).to_msg()
                bad_pub.publish(bad);step()
                if statuses.get('fault'):break
            wait(lambda:all(r in actual_twists and max(abs(v) for v in actual_twists[r])<1e-12 for r in robots)
                and all(not interlocks.get(r,{}).get('active',True) for r in robots),2.)
            report['dual_zero_latency_wall_s']=time.monotonic()-injection
            report['stop_budget_wall_s']=.4
            if report['dual_zero_latency_wall_s']>.4:
                raise RuntimeError('return leg dual stop exceeded existing 400 ms budget')
            report['maintenance_at_dual_stop']={r:dict(v) for r,v in maintenance.items()}
            wall_wait(.5)
            report['maintenance_after_fault_recovery']={r:dict(v) for r,v in maintenance.items()}
            if (len(maintenance)!=2 or any(v['state']!='FAULT' for v in maintenance.values())
                    or any(maintenance[r]['sent_goals']!=report['maintenance_at_dual_stop'][r]['sent_goals'] for r in robots)):
                raise RuntimeError('grip maintenance resumed commands after return fault')
            if any(max(abs(v) for v in actual_twists.get(r,[1.]))>1e-12 for r in robots):
                raise RuntimeError('motion resumed automatically after return fault')
            wheel_names=('front_left_joint','front_right_joint','back_left_joint','back_right_joint')
            wheel_speed={r:max(abs(dict(zip(latest[r,'joints'].name,latest[r,'joints'].velocity))[n]) for n in wheel_names) for r in robots}
            report['post_fault_max_wheel_speed_rad_s']=wheel_speed
            if max(wheel_speed.values())>1e-4:
                raise RuntimeError('wheel joints did not confirm both bases stopped')
            if adm_state.get('enabled',True) or any(endpoint_states[r]['state']!='SAFE_STOP' for r in robots):
                raise RuntimeError('arm admittance or endpoint resumed after return fault')
            late=[row for row in rows if row['wall_time']>time.monotonic()-.2]
            q_span=[max(row[f'follower_q{i}_rad'] for row in late)-min(row[f'follower_q{i}_rad'] for row in late) for i in range(1,6)]
            report['post_fault_joint_span_rad']=q_span
            if max(q_span)>.00001:
                raise RuntimeError('arm did not hold position after return fault')
            report['goals_after_fault_recovery']=endpoint_states['leader']['accepted_goals']
            if report['goals_after_fault_recovery']!=2:
                raise RuntimeError('unexpected goal after fault')
            report['no_automatic_restart']=True
            report['post_fault_controller_states']=snapshot_controllers()
            if len(report['post_fault_controller_states'])!=2 or not all(v.get(c)=='inactive'
                    for v in report['post_fault_controller_states'].values() for c in ('arm_controller','mecanum_drive_controller')):
                raise RuntimeError('controllers did not deactivate before cleanup stop')
            (a.output/'post_fault_controller_snapshot.json').write_text(json.dumps({
                'wall_time':time.monotonic(),'sim_time':now[0],'before_teardown_stop':True,
                'states':report['post_fault_controller_states']},indent=2))
        report['passed'] = True
    except KeyboardInterrupt:
        report['passed'] = False
        report['cancelled'] = True
        report['error'] = 'interrupted by user; stopping owned simulation processes'
        report['error_phase'] = phase[0]
        print(f'[CP-9] 中断: {phase[0]}。停止・終了処理を待ってください。', flush=True)
    except Exception as exc:
        report['traceback']=__import__('traceback').format_exc()
        report['error'] = str(exc)
        report['error_phase']=phase[0]
        print(f'[CP-9] 停止: {phase[0]}: {exc}', flush=True)
        # An evaluator/route error must request the same shared latched stop,
        # even while all sensor watchdogs remain healthy.
        try:
            service('/transport/safety/stop')
        except Exception:
            pass
        if phase[0] in ('motion','settle'):
            try:
                wait(lambda: all(max(abs(v) for v in actual_twists.get(r,[1.]))<1e-12 for r in robots)
                     and all(not interlocks.get(r,{}).get('active',True) for r in robots),2.)
                wall_wait(.3)
                report['safe_stop_observed']=bool(statuses.get('fault')) and all(max(abs(v) for v in actual_twists.get(r,[1.]))<1e-12 for r in robots)
            except Exception:report['safe_stop_observed']=False
        else:report['safe_stop_observed']=False
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        report['last_preparation_debug']=dict(prep)
        report['last_endpoint_states']=dict(endpoint_states)
        report['last_grasp_monitor']=dict(monitor)
        report['last_grip_maintenance']=dict(maintenance)
        report['safety_before_teardown']=dict(statuses)
        report['monitor_before_teardown']=dict(monitor)
        applied[:] = [0.]*6
        for pid in suspended:
            (os.killpg if a.fault=='contacts' else os.kill)(pid, signal.SIGCONT)
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
        actual_states={}
        try:actual_states=snapshot_controllers()
        except Exception as error:report['controller_snapshot_error']=str(error)
        report['actual_final_controller_states']=actual_states
        report['both_arm_base_inactive']=len(actual_states)==2 and all(actual_states[r].get(c)=='inactive'
            for r in robots for c in ('arm_controller','mecanum_drive_controller'))
        if not report['both_arm_base_inactive']:report['passed']=False
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
        report['last_wrench_status']=filter_state[0]
        report['interlock_states']=interlocks
        report['controller_states']=controller_states
        report['gate_states']=debug
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
        required = ('/follower/'+('follower_whole_body' if a.control_mode=='WHOLE_BODY' else 'follower_arm_base'), '/follower/wrench_preprocessor', '/follower/safety_gate','/follower/actuator_interlock','/follower/controller_audit','/follower/native_grasp_monitor')
        if a.grip_maintenance:
            required += ('/follower/local_grip_maintenance',)
        follower_inputs = [v[0] for key, value in graph.items() if key in required for v in value['subscriptions']]
        report['follower_input_boundary_passed'] = all(n in graph for n in required) and not any(t.startswith('/leader/') or 'goal' in t or 'ground_truth' in t or t.endswith('/tf') or '/world/' in t for t in follower_inputs)
        if not report['follower_input_boundary_passed']:
            report['passed'] = False
        (a.output/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        raw_log.close()
        output_log.close()
        interlock_log.close()
        allocation_log.close()
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
        if saved_profiles:
            restored = {}
            for robot, contents in saved_profiles.items():
                path = a.run_directory/(robot+'_safety.json')
                path.write_bytes(contents)
                restored[robot] = path.read_bytes() == contents
            (a.output/'original_profiles_restored.json').write_text(json.dumps(restored, indent=2))
    if not report['passed']:
        raise RuntimeError('CP-9 native graph boundary or experiment validation failed; see result.json')


if __name__ == '__main__':
    main()
