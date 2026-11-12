"""Physical ROS/Gazebo BT handshake trial, with no arm/base command publisher."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-directory', required=True)
    args = parser.parse_args()
    out = Path(args.output_directory).resolve()
    out.mkdir(parents=True, exist_ok=True)
    xml = out / 'duration_one_second.xml'
    xml.write_text('<root main_tree_to_execute="Trial"><BehaviorTree ID="Trial">'
                   '<FollowerCompliantCarry robot_namespace="amir2" duration_s="1" '
                   'status_timeout_s="0.5"/></BehaviorTree></root>\n')
    rclpy.init()
    node = Node('bt_trial_supervisor', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = ['UNKNOWN']
    events = []
    def status(message):
        state[0] = message.data
        events.append({'sim_time': node.get_clock().now().nanoseconds * 1e-9,
                       'state': message.data})
    node.create_subscription(String, '/amir2/follower/status', status, 10)
    active = node.create_publisher(Bool, '/cooperative_transport/active', 1)
    calibrate = node.create_client(Trigger, '/amir2/follower/calibrate')
    enable = node.create_client(SetBool, '/amir2/follower/enable')
    def until(predicate, seconds=30.):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('BT trial timed out: ' + state[0])
            rclpy.spin_once(node, timeout_sec=.02)
    process = None
    result = {'success': False, 'scope': 'stationary one-second enable/disable handshake'}
    log = (out / 'bt_runner.log').open('w')
    try:
        until(lambda: state[0] == 'DISABLED' and calibrate.service_is_ready())
        active.publish(Bool(data=True))
        deadline = time.monotonic() + 30.
        while True:
            future = calibrate.call_async(Trigger.Request())
            until(future.done)
            response = future.result()
            if response.success:
                break
            if (response.message != 'audited controllers not ready; retry calibration after discovery'
                    or time.monotonic() > deadline):
                raise RuntimeError(response.message)
            rclpy.spin_once(node, timeout_sec=.1)
        until(lambda: state[0] in ('READY', 'FAULT'))
        if state[0] != 'READY':
            raise RuntimeError('calibration failed')
        events.clear()
        process = subprocess.Popen(
            ['ros2', 'run', 'coop_transport_bt', 'bt_runner', '--ros-args',
             '-p', 'use_sim_time:=true', '-p', 'bt_xml_path:=' + str(xml)],
            stdout=log, stderr=subprocess.STDOUT)
        until(lambda: process.poll() is not None)
        result['bt_returncode'] = process.returncode
        if process.returncode != 0:
            raise RuntimeError('BT exited with code ' + str(process.returncode))
        until(lambda: state[0] == 'DISABLED', seconds=3.)
        states = [event['state'] for event in events]
        if process.returncode != 0 or 'FOLLOWING' not in states or states[-1] != 'DISABLED':
            raise RuntimeError('BT did not complete the observed enable/disable sequence')
        result['success'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['failure'] = str(exc) or 'interrupted'
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if rclpy.ok() and enable.service_is_ready():
            future = enable.call_async(SetBool.Request(data=False))
            try:
                until(future.done, seconds=3.)
            except RuntimeError:
                pass
        log.close()
        result['states'] = sorted(set(event['state'] for event in events))
        result['final_state'] = state[0]
        (out / 'status.json').write_text(json.dumps(events, indent=2))
        (out / 'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
