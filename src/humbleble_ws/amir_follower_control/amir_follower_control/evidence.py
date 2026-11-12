"""Read-only local sensor evidence, including frame/stamp diagnostics."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', default='amir2')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--output', required=True)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node('local_sensor_evidence', namespace=options.robot,
                parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    path = Path(options.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    counts = {}
    with path.open('w') as log:
        def receive(topic, msg):
            counts[topic] = counts.get(topic, 0)+1
            log.write(json.dumps({'topic':topic, 'wall_elapsed':time.monotonic()-start,
                                  'sim_receipt':node.get_clock().now().nanoseconds*1e-9,
                                  'message':message_to_ordereddict(msg)})+'\n')
        for topic, kind in [('ft_sensor',WrenchStamped), ('joint_states',JointState),
                            ('mecanum_drive_controller/odometry',Odometry)]:
            node.create_subscription(kind, topic, lambda m, t=topic: receive(t,m),
                                     qos_profile_sensor_data)
        while time.monotonic()-start < options.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
    print(json.dumps({'counts':counts, 'elapsed_wall_s':time.monotonic()-start}))
    node.destroy_node()
    rclpy.shutdown()
