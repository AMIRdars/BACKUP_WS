"""Pause one verified experiment F/T bridge and observe the real FAULT latch.

Run only against a stationary initialized Gazebo test scene. The named bridge is
always resumed in finally; the controller fault is deliberately not reset.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from amir_follower_control.experiment_support import StationaryWindow


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bridge-pid', type=int, required=True)
    parser.add_argument('--output-directory', required=True)
    args = parser.parse_args()
    proc = Path('/proc') / str(args.bridge_pid)
    command = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
    if not all(item in command for item in ('parameter_bridge', '/amir2/ft_sensor@',
                                            '__node:=wrist_ft_bridge', '__ns:=/amir2')):
        raise RuntimeError('PID does not identify the audited amir2 wrist bridge')
    environment = dict(item.split(b'=', 1) for item in (proc / 'environ').read_bytes().split(b'\0') if b'=' in item)
    if environment.get(b'ROS_DOMAIN_ID') != os.environ.get('ROS_DOMAIN_ID', '0').encode():
        raise RuntimeError('bridge ROS domain differs from the test process')
    out = Path(args.output_directory)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('wrench_dropout_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = ['UNKNOWN']
    events = []
    base = []
    wrench_stamps = []
    samples = {}
    def now():
        return node.get_clock().now().nanoseconds * 1e-9
    def status(message):
        state[0] = message.data
        events.append({'sim_time': now(), 'state': message.data})
    def wrench(message):
        stamp = message.header.stamp
        wrench_stamps.append(stamp.sec + stamp.nanosec * 1e-9)
    node.create_subscription(String, '/amir2/follower/status', status, 10)
    node.create_subscription(WrenchStamped, '/amir2/ft_sensor', wrench, qos_profile_sensor_data)
    node.create_subscription(JointState, '/amir2/joint_states',
                             lambda m: samples.update(joints=m), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/amir2/mecanum_drive_controller/odometry',
                             lambda m: samples.update(odom=m), qos_profile_sensor_data)
    node.create_subscription(Twist, '/amir2/rover_twist', lambda m: base.append(
        {'sim_time': now(), 'twist': [m.linear.x, m.linear.y, m.angular.z]}), 10)
    calibrate = node.create_client(Trigger, '/amir2/follower/calibrate')
    enable = node.create_client(SetBool, '/amir2/follower/enable')
    def until(predicate, seconds=30.):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('dropout trial timeout: ' + state[0])
            rclpy.spin_once(node, timeout_sec=.01)
    def service(client, request):
        until(client.service_is_ready)
        future = client.call_async(request)
        until(future.done)
        if not future.result().success:
            raise RuntimeError(future.result().message)
    stopped = False
    result = {'success': False, 'bridge_pid': args.bridge_pid, 'bridge_command': command,
              'scope': 'stationary F/T bridge loss; not a Leader communication delay'}
    try:
        until(lambda: state[0] == 'DISABLED' and calibrate.service_is_ready())
        stationary = StationaryWindow()
        until(lambda: stationary.ready(now(), samples.get('joints'), samples.get('odom')))
        service(calibrate, Trigger.Request())
        until(lambda: state[0] in ('READY', 'FAULT'))
        if state[0] != 'READY':
            raise RuntimeError('calibration failed')
        service(enable, SetBool.Request(data=True))
        until(lambda: state[0] in ('FOLLOWING', 'FAULT'))
        if state[0] != 'FOLLOWING':
            raise RuntimeError('could not enable stationary follower')
        result['bridge_stop_sim_time'] = now()
        os.kill(args.bridge_pid, signal.SIGSTOP)
        stopped = True
        until(lambda: state[0] == 'FAULT', seconds=5.)
        result['fault_observed_sim_time'] = now()
        result['stop_to_observed_fault_s'] = now() - result['bridge_stop_sim_time']
        result['last_wrench_stamp_at_fault'] = max(wrench_stamps)
        os.kill(args.bridge_pid, signal.SIGCONT)
        stopped = False
        restored = now()
        until(lambda: now() - restored > .2, seconds=5.)
        if state[0] != 'FAULT' or result['stop_to_observed_fault_s'] > .1:
            raise RuntimeError('fault was late or did not remain latched after bridge recovery')
        after = [sample for sample in base if sample['sim_time'] >= result['bridge_stop_sim_time']]
        if not after or any(abs(value) > 1e-9 for sample in after for value in sample['twist']):
            raise RuntimeError('expected stationary zero base command during the fault')
        result['success'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['failure'] = str(exc) or 'interrupted'
    finally:
        if stopped:
            os.kill(args.bridge_pid, signal.SIGCONT)
        if rclpy.ok() and enable.service_is_ready():
            try:
                service(enable, SetBool.Request(data=False))
            except RuntimeError:
                pass
        result['final_state'] = state[0]
        (out / 'status.json').write_text(json.dumps(events, indent=2))
        (out / 'base_commands.json').write_text(json.dumps(base, indent=2))
        (out / 'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
