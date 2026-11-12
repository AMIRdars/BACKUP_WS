"""Replay the measured fixed-grasp calibration through the production leader.

Snapshot unit test only: no executor, no simulator subscription, no actuation.
Run after the positive synchronized metrology trial has produced its certificate.
"""
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from amir_compliance_control.object_rotation_leader import ObjectRotationLeader


def test_measured_full_fixed_calibration_initializes_center_without_gt_input():
    folder = Path(__file__).resolve().parents[1]/'audits/object_center_rotation_20261103/cp8r1'
    certificate_file = folder/'sync_plus5_01/leader_fixed_grasp_calibration.json'
    certificate = json.loads(certificate_file.read_text())
    import yaml
    policy = yaml.safe_load((folder/'leader_minus5_01/rotation_policy.yaml').read_text())[
        '/leader/leader_object_rotation']['ros__parameters']
    policy.update(grasp_calibration_file=str(certificate_file), rotation_ungrasped_test=False)
    rclpy.init(args=[])
    node = ObjectRotationLeader(namespace='/leader', parameter_overrides=[
        Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(folder/'generated/leader.urdf')),
        *[Parameter(k, value=v) for k, v in policy.items()]])
    try:
        node.timer.cancel()
        base = np.array(certificate['odom_T_base_snapshot'])
        node.encoder = Odometry()
        node.encoder.header.frame_id, node.encoder.child_frame_id = 'leader/odom', 'leader/base_footprint'
        pose = node.encoder.pose.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, base[:3, 3])
        yaw = math.atan2(base[1, 0], base[0, 0])
        pose.orientation.z, pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        node.joints = JointState(name=list(node.model.names), position=certificate['joint_positions_rad'])
        node.sim = 10.
        node.encoder.header.stamp = node.joints.header.stamp = Time(seconds=node.sim).to_msg()
        node.clock_wall = node.encoder_wall = node.joints_wall = node.gate_wall = node.permit_wall = node.grasp_wall = time.monotonic()
        node.gate = {'robot': 'leader', 'ready': True, 'fault': ''}
        node.prepared = node.homed = node.permit = node.grasp = True
        node.emergency, node.state = False, 'READY'
        node.rotation_goal(Float64(data=math.radians(5.)))
        assert node.accepted == 1, node.last_rejection
        initial = node.rotation_session[1].sample(0.)
        np.testing.assert_allclose(initial.object_pose, certificate['object_pose_odom_snapshot'], atol=1e-14)
        assert not any(t.topic_name.startswith(('/world/', '/experiment/', '/follower/')) for t in node.subscriptions)
    finally:
        node.destroy_node()
        rclpy.shutdown()
