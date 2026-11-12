"""CP-8R-1 exclusive modes, input boundaries, replay, stops and rigid tracking."""
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from amir_compliance_control.object_center_rotation_trajectory import inverse_transform
from amir_compliance_control.object_rotation_leader import ObjectRotationLeader, minimum_duration
from amir_compliance_control.wrench import LocalModel, origin


ROOT = Path(__file__).resolve().parents[4]
URDF = ROOT/'audits/object_center_rotation_20261103/cp8r0/generated/leader.urdf'


def rotation_policy(calibration):
    return {'grasp_calibration_file': str(calibration), 'rotation_duration_s': 160.,
            'rotation_max_yaw_rad': math.radians(5.), 'rotation_max_angular_speed': .002,
            'rotation_max_base_speed': .002, 'rotation_max_grasp_speed': .002,
            'rotation_max_angular_acceleration': .001, 'rotation_max_base_acceleration': .001,
            'rotation_yaw_gain': 1., 'rotation_yaw_tolerance_rad': .0001,
            'rotation_position_tolerance_m': .0001, 'rotation_terminal_timeout_s': 20.,
            'rotation_tracking_limit_m': .01, 'rotation_tracking_limit_rad': .01,
            'rotation_hold_s': 1., 'rotation_arm_position_limit_m': .0005,
            'rotation_arm_angle_limit_rad': .01, 'rotation_ungrasped_test': True}


@pytest.fixture
def leader(tmp_path):
    xml = URDF.read_text()
    root, model = ET.fromstring(xml), LocalModel(xml)
    q = [float(root.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text)
         for n in model.names]
    _, sensor = model.frames(q)
    bg = sensor @ origin(root.find("joint[@name='tcp_fixed']/origin"))
    obj = np.eye(4)
    obj[:3, 3] = [1.075, 0., bg[2, 3]]
    file = tmp_path/'calibration.json'
    file.write_text(json.dumps({'robot': 'leader', 'grasp_frame': 'tcp_link',
        'object_frame': 'geometric_center_z_up', 'base_frame': 'base_footprint',
        'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(), 'calibration_verified': True,
        'scope': 'virtual_ungrasped_fixture', 'object_T_grasp': (inverse_transform(obj)@bg).tolist()}))
    rclpy.init(args=[])
    node = ObjectRotationLeader(namespace='/leader', parameter_overrides=[
        Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(URDF)),
        *[Parameter(k, value=v) for k, v in rotation_policy(file).items()]])
    node.timer.cancel()
    node.encoder = Odometry()
    node.encoder.header.frame_id, node.encoder.child_frame_id = 'leader/odom', 'leader/base_footprint'
    node.encoder.pose.pose.orientation.w = 1.
    node.joints = JointState(name=list(model.names), position=q)
    node.gate = {'robot': 'leader', 'ready': True, 'fault': '', 'armed': True, 'armed_sim_s': 0.}
    node.sim = 10.
    node.prepared = node.homed = node.permit = True
    node.emergency = False
    node.state = 'READY'
    refresh(node)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def refresh(node):
    node.encoder.header.stamp = node.joints.header.stamp = Time(seconds=node.sim).to_msg()
    node.clock_wall = node.encoder_wall = node.joints_wall = node.gate_wall = node.permit_wall = time.monotonic()


@pytest.mark.parametrize('delta', [math.radians(5), -math.radians(5)])
def test_ideal_motion_tracks_full_circle_reference_and_completion_requires_hold(leader, delta):
    leader.rotation_goal(Float64(data=delta))
    assert leader.accepted == 1 and leader.state == 'ROTATE_OBJECT_CENTER'
    start, path, duration, _ = leader.rotation_session
    reference = path.sample(0.)
    np.testing.assert_allclose(reference.object_pose[:3, 3], [1.075, 0., reference.object_pose[2, 3]], atol=1e-14)
    for t in np.arange(0., duration+1.1, .02):
        leader.sim = start+float(t)
        leader.last_tick_sim = leader.sim-.02
        ref = path.sample(t)
        p = leader.encoder.pose.pose
        p.position.x, p.position.y = map(float, ref.base_pose[:2, 3])
        p.orientation.z, p.orientation.w = math.sin(ref.base_yaw_unwrapped/2), math.cos(ref.base_yaw_unwrapped/2)
        refresh(leader)
        command = leader.base_request(0.)
        assert np.linalg.norm(command[:2]) <= .002+1e-14
        assert abs(command[2]) <= .002+1e-14
        if t < duration:
            assert leader.completed == 0
    assert leader.completed == 1 and leader.rotation_session is None and leader.state == 'READY'


def test_goal_modes_exclude_each_other_and_duplicates_never_reinitialize(leader):
    leader.rotation_goal(Float64(data=.01))
    session = leader.rotation_session
    leader.rotation_goal(Float64(data=.02))
    leader.goal(PoseStamped())
    assert leader.accepted == 1 and leader.rejected == 2 and leader.rotation_session is session
    leader.rotation_session = None
    leader.state = 'READY'
    leader.rotation_goal(Float64(data=.01))
    assert leader.accepted == 1 and 'duplicate' in leader.last_rejection
    assert leader.rearm_same_delta(Trigger.Request(), Trigger.Response()).success
    leader.rotation_goal(Float64(data=.01))
    assert leader.accepted == 2


@pytest.mark.parametrize('delta', [0., .2, math.nan, math.inf])
def test_bad_delta_is_rejected_without_motion(leader, delta):
    leader.rotation_goal(Float64(data=delta))
    assert leader.accepted == 0 and leader.rotation_session is None
    assert not leader.base_request(0.).any()


def test_straight_goal_keeps_existing_api_and_blocks_rotation(leader):
    leader.grasp, leader.grasp_wall = True, time.monotonic()
    goal = PoseStamped()
    goal.header.stamp, goal.header.frame_id = Time(seconds=leader.sim).to_msg(), 'leader/base_footprint'
    goal.pose.orientation.w = 1.
    goal.pose.position.x = .0001
    leader.goal(goal)
    assert leader.accepted == 1 and leader.path_goal is not None
    leader.rotation_goal(Float64(data=.01))
    assert leader.accepted == 1 and leader.rotation_session is None


@pytest.mark.parametrize('fault', ['stop', 'tracking', 'arm_pose', 'time_jump'])
def test_fault_discards_latch_and_produces_zero_without_replay(leader, fault):
    leader.rotation_goal(Float64(data=.01))
    leader.last_tick_sim = leader.sim-.02
    if fault == 'stop':
        leader.trip('explicit shared stop')
    elif fault == 'tracking':
        leader.encoder.pose.pose.position.x = .1
    elif fault == 'arm_pose':
        leader.joints.position[0] += .01
    else:
        leader.last_tick_sim = leader.sim-.1
    assert not leader.base_request(0.).any()
    assert leader.latched and leader.rotation_session is None and leader.state == 'SAFE_STOP'
    leader.encoder.pose.pose.position.x = 0.
    leader.rotation_goal(Float64(data=-.01))
    assert leader.accepted == 1 and not leader.base_request(0.).any()


def test_unready_or_moving_start_and_timestamp_skew_are_rejected(leader):
    leader.encoder.twist.twist.angular.z = .001
    leader.rotation_goal(Float64(data=.01))
    assert 'stationary' in leader.last_rejection
    leader.encoder.twist.twist.angular.z = 0.
    leader.joints.header.stamp = Time(seconds=leader.sim-.03).to_msg()
    leader.rotation_goal(Float64(data=.01))
    assert 'skew' in leader.last_rejection


def test_virtual_calibration_cannot_drive_an_attached_payload(leader):
    leader.grasp, leader.grasp_wall = True, time.monotonic()
    leader.rotation_goal(Float64(data=.01))
    assert leader.accepted == 0 and 'attached payload' in leader.last_rejection


def test_rotation_has_only_own_state_goals_and_boolean_safety_inputs(leader):
    inputs = {s.topic_name for s in leader.subscriptions}
    assert '/leader/object_rotation_delta' in inputs
    assert not any('/follower/' in t or '/experiment/' in t or '/world/' in t or '/tf' in t for t in inputs)
    assert sum(p.topic_name == '/leader/base_cmd' for p in leader.publishers) == 1


@pytest.mark.parametrize('sign', [-1., 1.])
def test_duration_bound_accounts_for_each_robot_radius_and_centripetal_acceleration(sign):
    angle = sign*math.pi/2
    t = minimum_duration(angle, 1.075, .6, angular_speed=.002,
        base_speed=.001, grasp_speed=.001, angular_acceleration=.001, base_acceleration=.001)
    assert 1.875*abs(angle)/t <= .002
    assert 1.075*1.875*abs(angle)/t <= .001+1e-15
    assert .6*1.875*abs(angle)/t <= .001
    assert 1.075*(10*math.sqrt(3)/3*abs(angle)+(1.875*angle)**2)/t**2 <= .001
