#!/usr/bin/env python3
"""Independent CP-1 motion verification; GT is used only by this evaluator.

Requires the supported, ungrasped CP-1 scene. Does not implement transport.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--finish-file', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    log = (args.output/'evaluation_launch.log').open('w')
    evaluation = subprocess.Popen(['ros2', 'launch', 'cooperative_transport_bringup',
        'compliance_evaluation.launch.py'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = rclpy.create_node('cp1_independent_motion_verifier')
    robots = ('leader', 'follower')
    data, now = {}, [0.]
    pubs, arms, grippers, clients = {}, {}, {}, {}
    subscriptions = []
    for robot in robots:
        pubs[robot] = node.create_publisher(Twist, f'/{robot}/rover_twist', 10)
        arms[robot] = ActionClient(node, FollowJointTrajectory,
                                   f'/{robot}/arm_controller/follow_joint_trajectory')
        grippers[robot] = ActionClient(node, GripperCommand, f'/{robot}/gripper_controller/gripper_cmd')
        clients[robot] = node.create_client(ListControllers, f'/{robot}/controller_manager/list_controllers')
        for key, topic, typ in [
            ('joints', f'/{robot}/joint_states', JointState),
            ('encoder', f'/{robot}/mecanum_drive_controller/odometry', Odometry),
            ('gt', f'/experiment/{robot}/ground_truth/odom', Odometry)]:
            subscriptions.append(node.create_subscription(typ, topic,
                lambda msg, k=(robot, key): data.__setitem__(k, msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Clock, '/clock',
        lambda msg: now.__setitem__(0, msg.clock.sec+msg.clock.nanosec*1e-9), qos_profile_sensor_data))
    results = []

    def wait(predicate, timeout=45.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('ROS data/action wait timed out')
            rclpy.spin_once(node, timeout_sec=.02)

    def interval(seconds, active=None, command=None):
        start, wall = now[0], time.monotonic()
        while now[0]-start < seconds:
            if time.monotonic()-wall > max(20., seconds*15):
                raise RuntimeError('simulation clock stalled')
            for robot in robots:
                pubs[robot].publish(command if robot == active else Twist())
            rclpy.spin_once(node, timeout_sec=.02)

    def pose(robot, source):
        p = data[(robot, source)].pose.pose
        q = p.orientation
        return [p.position.x, p.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    def delta(start, end):
        x, y = end[0]-start[0], end[1]-start[1]
        return [math.cos(start[2])*x+math.sin(start[2])*y,
                -math.sin(start[2])*x+math.cos(start[2])*y,
                math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2]))]

    def joints(robot):
        msg = data[(robot, 'joints')]
        return dict(zip(msg.name, msg.position))

    def action(client, goal):
        wait(lambda: client.server_is_ready())
        f = client.send_goal_async(goal)
        wait(f.done)
        handle = f.result()
        if not handle.accepted:
            raise RuntimeError('action goal rejected')
        f = handle.get_result_async()
        wait(f.done)
        if f.result().status != 4:
            raise RuntimeError(f'action failed: {f.result()}')
        return f.result().result

    def save():
        (args.output/'motion.json').write_text(json.dumps(results, indent=2))

    try:
        wait(lambda: all((r, k) in data for r in robots for k in ('joints', 'encoder', 'gt')))
        for robot, client in clients.items():
            wait(client.service_is_ready)
            deadline = time.monotonic()+45.
            while True:
                f = client.call_async(ListControllers.Request())
                wait(f.done)
                if len([c for c in f.result().controller if c.state == 'active']) == 4:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(f'{robot}: four active controllers required')
                interval(1.)
        interval(1.)
        for robot in robots:
            peer = next(r for r in robots if r != robot)
            for axis, speed in [('x', .01), ('y', .01), ('yaw', .02)]:
                initial = {r: {s: pose(r, s) for s in ('encoder', 'gt')} for r in robots}
                cmd = Twist()
                if axis == 'yaw':
                    cmd.angular.z = speed
                else:
                    setattr(cmd.linear, axis, speed)
                interval(1.5, robot, cmd)
                interval(.5)
                movement = {r: {s: delta(initial[r][s], pose(r, s))
                               for s in ('encoder', 'gt')} for r in robots}
                index = ('x', 'y', 'yaw').index(axis)
                stopped = data[(robot, 'encoder')].twist.twist
                passed = (movement[robot]['encoder'][index] > speed*.5 and
                          movement[robot]['gt'][index] > speed*.5 and
                          math.hypot(*movement[peer]['gt'][:2]) < .003 and
                          abs(movement[peer]['gt'][2]) < .01 and
                          math.hypot(stopped.linear.x, stopped.linear.y) < .001 and
                          abs(stopped.angular.z) < .001)
                results.append({'robot': robot, 'test': 'base_'+axis,
                    'speed': speed, 'duration_s': 1.5, 'movement_body': movement, 'passed': passed})
                save()
                if not passed:
                    raise RuntimeError(f'{robot} base {axis}: sign/independence/stop failed')
            for name in [f'Joint_{i}' for i in range(1, 6)]:
                baseline = joints(robot)
                peer_before = joints(peer)
                for offset in (.01, 0.):
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory.joint_names = [f'Joint_{i}' for i in range(1, 6)]
                    target = [baseline[j]+(offset if j == name else 0.)
                              for j in goal.trajectory.joint_names]
                    point = JointTrajectoryPoint(positions=target)
                    point.time_from_start.sec = 2
                    goal.trajectory.points = [point]
                    response = action(arms[robot], goal)
                    interval(.3)
                    actual = joints(robot)
                    error = max(abs(actual[j]-v) for j, v in zip(goal.trajectory.joint_names, target))
                    peer_change = max(abs(joints(peer)[j]-peer_before[j]) for j in goal.trajectory.joint_names)
                    measured = actual[name]-baseline[name]
                    passed = (response.error_code == 0 and error < .015 and peer_change < .003 and
                              (offset == 0. or measured > .005))
                    results.append({'robot': robot, 'test': name, 'offset_rad': offset,
                        'measured_delta_rad': measured, 'max_target_error_rad': error,
                        'peer_max_change_rad': peer_change, 'passed': passed})
                    save()
                    if not passed:
                        raise RuntimeError(f'{robot} {name}: tracking/independence failed')
            for position in (-.9, -1.):
                goal = GripperCommand.Goal()
                goal.command.position, goal.command.max_effort = position, .8
                response = action(grippers[robot], goal)
                interval(.3)
                actual = joints(robot)['Gripper']
                passed = abs(actual-position) < .01
                results.append({'robot': robot, 'test': 'gripper', 'target_rad': position,
                                'actual_rad': actual, 'passed': passed})
                save()
                if not passed:
                    raise RuntimeError(f'{robot}: gripper position failed')
        print(f'PASS: {len(results)} independent motion checks', flush=True)
    except Exception as exc:
        results.append({'passed': False, 'error': str(exc)})
        raise
    finally:
        for publisher in pubs.values():
            publisher.publish(Twist())
        save()
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(evaluation.pid, signal.SIGINT)
        try:
            evaluation.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(evaluation.pid, signal.SIGKILL)
            evaluation.wait()
        log.close()
        args.finish_file.touch()


if __name__ == '__main__':
    main()
