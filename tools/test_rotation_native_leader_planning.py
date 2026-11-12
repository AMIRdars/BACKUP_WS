"""Native passive leader preserves actual contact tilt and gates excessive tilt."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import yaml

from amir_compliance_control.object_rotation_leader import ObjectRotationLeader
from amir_compliance_control.object_center_rotation_trajectory import initialize_object_pose
from amir_compliance_control.wrench import LocalModel, rotation


ROOT = Path(__file__).resolve().parents[1]
MEASURED = ROOT/'audits/rotation_native_gate0_20261105/native_gate0_01'


def test_actual_native_static_calibration_can_start_passive_goal_without_forcing_level():
    policy = yaml.safe_load((MEASURED/'rotation_policy.yaml').read_text())['/leader/leader_object_rotation']['ros__parameters']
    urdf = MEASURED/'fixture_snapshot/leader.urdf'
    with (MEASURED/'samples.jsonl').open() as stream:
        row = next(json.loads(line) for line in stream if json.loads(line)['phase'] == 'native_static_hold')
    q = [row[f'leader_q{i}_rad'] for i in range(1, 6)]
    model = LocalModel(urdf.read_text())
    rclpy.init(args=[])
    node = None
    try:
        node = ObjectRotationLeader(namespace='/leader', parameter_overrides=[
            Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(urdf)),
            *[Parameter(k, value=v) for k, v in policy.items()]])
        node.timer.cancel()
        node.encoder = Odometry()
        node.encoder.header.frame_id, node.encoder.child_frame_id = 'leader/odom', 'leader/base_footprint'
        node.encoder.pose.pose.orientation.w = 1.
        node.joints = JointState(name=list(model.names), position=q)
        node.sim = 10.
        node.encoder.header.stamp = node.joints.header.stamp = Time(seconds=node.sim).to_msg()
        node.gate = dict(robot='leader', ready=True, fault='', armed=True, armed_sim_s=0.)
        node.prepared = node.homed = node.permit = node.grasp = True
        node.emergency = False
        node.state = 'READY'
        wall = time.monotonic()
        for key in ('clock_wall', 'encoder_wall', 'joints_wall', 'gate_wall', 'permit_wall', 'grasp_wall'):
            setattr(node, key, wall)
        base, local = node.base_pose(), node.grasp_pose()
        reconstructed = initialize_object_pose(base, local, node.object_T_grasp)
        actual_tilt = math.acos(float(np.clip(reconstructed[2, 2], -1., 1.)))
        assert 1e-9 < actual_tilt < policy['rotation_arm_angle_limit_rad']
        node.rotation_goal(Float64(data=math.radians(.005)))
        assert node.accepted == 1, node.last_rejection
        _, path, _, initial_grasp = node.rotation_session
        reference = path.sample(0.)
        np.testing.assert_allclose(reference.object_pose[:3, 3], reconstructed[:3, 3], atol=1e-15)
        np.testing.assert_allclose(reference.object_pose[:3, 2], [0., 0., 1.], atol=1e-15)
        np.testing.assert_allclose(initial_grasp, local, atol=1e-15)
        np.testing.assert_allclose(reference.grasp_pose, base@local, atol=1e-15)
        assert not np.allclose(reference.grasp_pose[:3, 2], [0., 0., 1.])
        assert not any(s.topic_name.startswith(('/experiment/', '/world/', '/follower/')) for s in node.subscriptions)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('angle', [.010001, .02, -.02])
def test_native_tilt_outside_existing_pose_envelope_still_refuses(angle):
    node = SimpleNamespace(calibration_scope='native_contact_verified', policy={'rotation_arm_angle_limit_rad': .01})
    obj = np.eye(4)
    obj[:3, :3] = rotation([1., 0., 0.], angle)
    with pytest.raises(ValueError, match='tilt'):
        ObjectRotationLeader.planning_object_pose(node, obj)


@pytest.mark.parametrize('scope', ['fixed_fixture_verified', 'virtual_ungrasped_fixture'])
def test_non_native_planning_contract_is_unchanged(scope):
    node = SimpleNamespace(calibration_scope=scope)
    obj = np.eye(4)
    obj[:3, :3] = rotation([1., 0., 0.], .001)
    assert ObjectRotationLeader.planning_object_pose(node, obj) is obj
