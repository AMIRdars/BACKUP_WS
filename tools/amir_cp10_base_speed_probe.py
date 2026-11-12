#!/usr/bin/env python3
"""Simulation-only, ungrasped CP-1 base speed ladder, with independent GT evaluation.

No loaded transport permission or hardware rating is established by this test.
Creates an isolated scene, places stationary robots away from the supported
payload before testing, and only commands one base at a time.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
import yaml


ROBOTS = ('leader', 'follower')


def reference(t, speed, acceleration, hold):
    ramp = abs(speed)/acceleration
    magnitude = max(0., min(abs(speed), acceleration*t,
                            acceleration*(2*ramp+hold-t)))
    return math.copysign(magnitude, speed)


def body_delta(start, end):
    x, y = end[0]-start[0], end[1]-start[1]
    return [math.cos(start[2])*x+math.sin(start[2])*y,
            -math.sin(start[2])*x+math.cos(start[2])*y,
            math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2]))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speeds', type=float, nargs='+', default=[.01, .05, .1, .2, .5, 1., 1.05])
    parser.add_argument('--acceleration', type=float, default=.5)
    parser.add_argument('--hold', type=float, default=2.)
    args = parser.parse_args()
    if (os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID')
            or not os.environ.get('IGN_PARTITION', '').startswith('amir_cp10_base_')):
        parser.error('requires isolated ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=1, IGN_PARTITION=amir_cp10_base_*')
    if (not all(math.isfinite(x) and 0 < x <= 1.05 for x in args.speeds)
            or not math.isfinite(args.acceleration) or not 0 < args.acceleration <= .5
            or not math.isfinite(args.hold) or not 1 <= args.hold <= 3):
        parser.error('invalid numerical simulation experiment parameters')
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    generated = output/'generated'
    processes, logs, data, received, now, clock_wall = [], [], {}, {}, [0.], [time.monotonic()]
    results, controller_final = [], {}
    summary = {'scope': 'simulation-only ungrasped base motion; not loaded transport', 'passed': False,
               'acceleration_m_s2': args.acceleration, 'hold_s': args.hold,
               'ROS_DOMAIN_ID': os.environ['ROS_DOMAIN_ID'], 'IGN_PARTITION': os.environ['IGN_PARTITION']}
    def start(name, command):
        stream = (output/(name+'.log')).open('w')
        logs.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process
    def interrupt(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('cp10_ungrasped_base_speed_evaluator')
    pubs, lists, switches, subscriptions = {}, {}, {}, []
    for robot in ROBOTS:
        pubs[robot] = node.create_publisher(Twist, '/'+robot+'/rover_twist', 10)
        lists[robot] = node.create_client(ListControllers, '/'+robot+'/controller_manager/list_controllers')
        switches[robot] = node.create_client(SwitchController, '/'+robot+'/controller_manager/switch_controller')
        for key, topic, typ in [('joints', '/'+robot+'/joint_states', JointState),
                ('encoder', '/'+robot+'/mecanum_drive_controller/odometry', Odometry),
                ('gt', '/experiment/'+robot+'/ground_truth/odom', Odometry)]:
            def receive(msg, k=(robot, key)):
                data[k], received[k] = msg, time.monotonic()
            subscriptions.append(node.create_subscription(typ, topic, receive, qos_profile_sensor_data))
    def clock(msg):
        now[0], clock_wall[0] = msg.clock.sec+msg.clock.nanosec*1e-9, time.monotonic()
    subscriptions.append(node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data))
    csv_stream = (output/'samples.csv').open('w')
    writer = csv.writer(csv_stream)
    writer.writerow(['case', 'sim_time', 'active_robot', 'request_m_s', 'leader_gt_x', 'leader_gt_y',
                     'follower_gt_x', 'follower_gt_y', 'leader_encoder_vx', 'follower_encoder_vx',
                     'leader_wheel_max_rad_s', 'follower_wheel_max_rad_s'])
    wheel_names, wheel_limits = {}, {}

    def zero():
        for pub in pubs.values():
            pub.publish(Twist())

    def wait(predicate, timeout=60.):
        deadline = time.monotonic()+timeout
        while not predicate():
            zero()
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() > deadline:
                raise RuntimeError('ROS data/service wait timed out')

    def pose(robot, source='gt'):
        p = data[robot, source].pose.pose
        q = p.orientation
        return [p.position.x, p.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    def wheel_max(robot):
        msg = data[robot, 'joints']
        velocities = dict(zip(msg.name, msg.velocity))
        actual = [velocities[n] for n in wheel_names[robot]]
        if not all(math.isfinite(v) and abs(v) <= wheel_limits[robot][n]*1.01
                   for n, v in zip(wheel_names[robot], actual)):
            raise RuntimeError(robot+': nonfinite or over-limit actual wheel velocity')
        return max(map(abs, actual))

    def health():
        wall = time.monotonic()
        if wall-clock_wall[0] > 1.:
            raise RuntimeError('clock watchdog')
        if any(wall-received.get((r,k),0) > 1. for r in ROBOTS for k in ('gt','encoder','joints')):
            raise RuntimeError('feedback watchdog')
        if any(p.poll() is not None for _, p in processes):
            raise RuntimeError('owned scene/bridge process exited')
        for r in ROBOTS:
            wheel_max(r)
            odom = data[r, 'encoder']
            values = pose(r)+[odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                              odom.twist.twist.angular.z]
            if not all(map(math.isfinite, values)):
                raise RuntimeError('nonfinite pose/encoder')

    def interval(seconds, active=None, command=lambda t: 0., case='settle', sample=None):
        begin, wall = now[0], time.monotonic()
        while now[0]-begin < seconds:
            health()
            t = now[0]-begin
            speed = command(t)
            for r, pub in pubs.items():
                msg = Twist()
                msg.linear.x = speed if r == active else 0.
                pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.005)
            row = [case, now[0], active, speed, *pose('leader')[:2], *pose('follower')[:2],
                   *[data[r,'encoder'].twist.twist.linear.x for r in ROBOTS],
                   *[wheel_max(r) for r in ROBOTS]]
            writer.writerow(row)
            if sample is not None:
                sample(t)
            if time.monotonic()-wall > max(30., seconds*30):
                raise RuntimeError('simulation progress timeout')

    def save():
        (output/'motion.json').write_text(json.dumps(results, indent=2))
        (output/'result.json').write_text(json.dumps(summary, indent=2))

    try:
        start('scene', ['ros2','launch','cooperative_transport_bringup','compliance_cp1.launch.py',
                        'headless:=true','run_directory:='+str(generated)])
        start('evaluation', ['ros2','launch','cooperative_transport_bringup','compliance_evaluation.launch.py'])
        start('placement_bridge', ['ros2','run','ros_gz_bridge','parameter_bridge',
              '/world/cooperative_transport/set_pose@ros_gz_interfaces/srv/SetEntityPose'])
        wait(lambda: now[0] > 0 and all((r,k) in data for r in ROBOTS for k in ('gt','encoder','joints')))
        for robot in ROBOTS:
            wait(lists[robot].service_is_ready)
            deadline = time.monotonic()+90
            while True:
                f = lists[robot].call_async(ListControllers.Request()); wait(f.done)
                if len([c for c in f.result().controller if c.state == 'active']) == 4:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('four active controllers required')
            root = ET.parse(generated/(robot+'.urdf')).getroot()
            plugins = [e.text for e in root.findall('ros2_control/hardware/plugin')]
            if not plugins or set(plugins) != {'gz_ros2_control/GazeboSimSystem'}:
                raise RuntimeError('simulation hardware plugin must be confirmed')
            config = yaml.safe_load((generated/(robot+'_base_controller.yaml')).read_text())
            params = config['/**/mecanum_drive_controller']['ros__parameters']
            radius = float(params['kinematics']['wheels_radius'])
            wheel_names[robot] = [params[k] for k in ('front_left_wheel_command_joint_name',
                'front_right_wheel_command_joint_name','rear_left_wheel_command_joint_name','rear_right_wheel_command_joint_name')]
            wheel_limits[robot] = {}
            for name in wheel_names[robot]:
                limit = float(root.find(f"joint[@name='{name}']/limit").get('velocity'))
                interface = root.find(f"ros2_control/joint[@name='{name}']/command_interface[@name='velocity']")
                limit = min(limit, float(interface.find("param[@name='max']").text),
                            -float(interface.find("param[@name='min']").text))
                wheel_limits[robot][name] = limit
                if max(args.speeds)/radius > limit:
                    raise RuntimeError('requested straight speed exceeds actual model wheel bound')
            summary[robot+'_model'] = {'wheel_radius_m': radius, 'wheel_limits_rad_s': wheel_limits[robot],
                                       'hardware_plugins': plugins}
        placer = node.create_client(SetEntityPose, '/world/cooperative_transport/set_pose')
        wait(placer.service_is_ready)
        for robot, x in zip(ROBOTS, (-10.,10.)):
            req = SetEntityPose.Request()
            req.entity.name, req.entity.type = robot, Entity.MODEL
            req.pose = data[robot,'gt'].pose.pose
            req.pose.position.x = x
            f = placer.call_async(req); wait(f.done)
            if not f.result().success:
                raise RuntimeError('stationary initial placement rejected')
        node.destroy_client(placer)
        interval(2.)
        summary['initial_placement'] = {r:pose(r) for r in ROBOTS}
        for speed in args.speeds:
            for robot in ROBOTS:
                peer = next(r for r in ROBOTS if r != robot)
                for sign in (-1,1):
                    signed, ramp = speed*sign, speed/args.acceleration
                    duration = 2*ramp+args.hold
                    initial = {r:{s:pose(r,s) for s in ('gt','encoder')} for r in ROBOTS}
                    samples, wheels = [], []
                    def sample(t):
                        if ramp+.3 <= t <= ramp+args.hold-.2:
                            stamp = data[robot,'gt'].header.stamp
                            samples.append([stamp.sec+stamp.nanosec*1e-9,*pose(robot)])
                        wheels.append(wheel_max(robot))
                    case = f'{robot}_{signed:g}'
                    print('START '+case, flush=True)
                    interval(duration, robot, lambda t: reference(t,signed,args.acceleration,args.hold),case,sample)
                    interval(1., case=case+'_stop')
                    delta = {r:{s:body_delta(initial[r][s],pose(r,s)) for s in ('gt','encoder')} for r in ROBOTS}
                    measured = body_delta(samples[0][1:],samples[-1][1:])[0]/(samples[-1][0]-samples[0][0])
                    stop_speed = max(math.hypot(data[r,'encoder'].twist.twist.linear.x,
                                              data[r,'encoder'].twist.twist.linear.y) for r in ROBOTS)
                    stopped_wheel = max(wheel_max(r) for r in ROBOTS)
                    expected = signed*(ramp+args.hold)
                    passed = (abs(measured/signed-1) < .1
                        and abs(delta[robot]['gt'][0]/expected-1) < .1
                        and math.hypot(*delta[peer]['gt'][:2]) < .003
                        and abs(delta[peer]['gt'][2]) < .01
                        and abs(delta[robot]['gt'][1]) < .01 and abs(delta[robot]['gt'][2]) < .01
                        and stop_speed < .001 and stopped_wheel < .02)
                    results.append({'robot':robot, 'test':'base_x', 'speed':signed, 'duration_s':duration,
                        'steady_gt_speed_m_s':measured, 'expected_distance_m':expected,
                        'movement_body':delta, 'max_wheel_speed_rad_s':max(wheels),
                        'stopped_encoder_speed_m_s':stop_speed, 'stopped_wheel_speed_rad_s':stopped_wheel,
                        'passed':passed})
                    save()
                    print(json.dumps(results[-1]), flush=True)
                    if not passed:
                        raise RuntimeError(case+': speed/path/independence/stop failed')
        summary['passed'] = True
    except BaseException as exc:
        summary['error'] = type(exc).__name__+': '+str(exc)
        print(summary['error'], flush=True)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        zero()
        try:
            if all(r in wheel_names for r in ROBOTS):
                interval(.5)
            for robot in ROBOTS:
                if switches[robot].service_is_ready():
                    req = SwitchController.Request(deactivate_controllers=['arm_controller','mecanum_drive_controller'],
                                                   strictness=SwitchController.Request.BEST_EFFORT)
                    f = switches[robot].call_async(req); wait(f.done, 10.)
                    state = lists[robot].call_async(ListControllers.Request()); wait(state.done, 10.)
                    controller_final[robot] = {c.name:c.state for c in state.result().controller}
            summary['final_controllers'] = controller_final
            if not all(controller_final.get(r,{}).get(n) == 'inactive' for r in ROBOTS
                       for n in ('arm_controller','mecanum_drive_controller')):
                raise RuntimeError('stop/deactivation not confirmed')
        except BaseException as exc:
            summary['passed'], summary['cleanup_error'] = False, repr(exc)
        node.destroy_node()
        rclpy.shutdown()
        for name, process in reversed(processes):
            for sig, timeout in ((signal.SIGINT,12),(signal.SIGTERM,5),(signal.SIGKILL,5)):
                try:
                    os.killpg(process.pid,sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                # Ensure descendants that outlive their launch process also terminate.
                try:
                    os.killpg(process.pid,signal.SIGTERM)
                except ProcessLookupError:
                    pass
                break
        summary['owned_process_roots'] = {name:{'pid':p.pid,'returncode':p.poll()} for name,p in processes}
        csv_stream.close()
        for stream in logs:
            stream.close()
        save()
    print(json.dumps(summary,indent=2),flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
