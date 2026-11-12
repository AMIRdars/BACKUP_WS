import hashlib
import json
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.parameter import Parameter
from rclpy.time import Time

from amir_compliance_control.wrench import LocalModel, at_grasp, interpolate, rotation
from amir_compliance_control.wrench_preprocessor import WrenchPreprocessor


@pytest.fixture
def model_file(tmp_path):
    # Synthetic chain exercises the pipeline; this is not physical calibration.
    joints = []
    parent = 'base_footprint'
    for i in range(1, 6):
        child = 'gripper_base_1' if i == 5 else f'link{i}'
        joints.append(f'<joint name="Joint_{i}" type="revolute"><parent link="{parent}"/>'
            f'<child link="{child}"/><origin xyz=".1 0 0"/><axis xyz="0 0 1"/>'
            '<limit lower="-2" upper="2" velocity="1" effort="4"/></joint>')
        parent = child
    xml = '<robot>'+''.join(joints)+('<joint name="tcp_fixed" type="fixed">'
        '<parent link="gripper_base_1"/><child link="tcp_link"/>'
        '<origin xyz=".08 0 0"/></joint></robot>')
    path = tmp_path/'robot.urdf'
    path.write_text(xml)
    cert = tmp_path/'certificate.json'
    cert.write_text(json.dumps({'sensor_calibration_passed': True, 'robot': 'follower',
        'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(), 'sign': 1.,
        'header_frame': 'follower/Joint_5/wrist_ft_sensor',
        'verified_axes': ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']}))
    return path, cert


def test_moment_is_shifted_to_grasp_not_mislabeled_at_base():
    sensor = np.eye(4)
    sensor[:3, :3] = rotation([0, 0, 1], np.pi/2)
    sensor[:3, 3] = [1, 0, 0]
    result = at_grasp([1, 0, 0, 0, 0, 0], sensor, [0, 0, 0], 1.)
    np.testing.assert_allclose(result, [0, 1, 0, 0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(at_grasp([1, 0, 0, 0, 0, 0], sensor, [0, 0, 0], -1.), -result)


def test_no_extrapolation_or_large_time_gap():
    history = [(1., np.zeros(5)), (1.01, np.ones(5))]
    np.testing.assert_allclose(interpolate(history, 1.005, .02), .5, atol=1e-12)
    with pytest.raises(ValueError):
        interpolate(history, 1.02, .02)
    with pytest.raises(ValueError):
        interpolate(history, 1.005, .001)


def test_local_fk_limits_and_nonfinite_inputs(model_file):
    model = LocalModel(model_file[0].read_text())
    grasp, sensor = model.frames(np.zeros(5))
    np.testing.assert_allclose(grasp, [.58, 0, 0])
    np.testing.assert_allclose(sensor[:3, 3], [.5, 0, 0])
    with pytest.raises(ValueError):
        model.frames([3, 0, 0, 0, 0])
    with pytest.raises(ValueError):
        at_grasp([np.nan]*6, sensor, grasp, 1.)


def test_precomputed_urdf_origins_preserve_fk_and_its_numeric_derivative(model_file,monkeypatch):
    from amir_compliance_control import wrench
    model=LocalModel(model_file[0].read_text());q=np.array([.13,-.24,.31,-.27,.18])
    # Independently rebuild the original serial transform using the URDF.
    t=np.eye(4);index=0;sensor=None
    for j in model.chain:
        t=t@wrench.origin(j.find('origin'))
        if j.get('type')!='fixed':
            r=np.eye(4);r[:3,:3]=rotation(j.find('axis').get('xyz').split(),q[index]);t=t@r;index+=1
        if j.find('child').get('link')=='gripper_base_1':sensor=t.copy()
    # No origin parser may run on the FK/Jacobian hot path after construction.
    def unexpected(*args):raise AssertionError('constant URDF origin reparsed')
    monkeypatch.setattr(wrench,'origin',unexpected)
    p,s=model.frames(q);np.testing.assert_array_equal(p,t[:3,3]);np.testing.assert_array_equal(s,sensor)
    h=1e-6
    numerical=np.column_stack([(model.frames(q+np.eye(5)[i]*h)[0]-model.frames(q-np.eye(5)[i]*h)[0])/(2*h) for i in range(5)])
    np.testing.assert_allclose(model.jacobian(q)[:3],numerical,atol=1e-10)


@pytest.fixture
def node(model_file, monkeypatch):
    rclpy.init(args=[])
    instance = WrenchPreprocessor(namespace='/follower', parameter_overrides=[
        Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(model_file[0])),
        Parameter('calibration_file', value=str(model_file[1])), Parameter('bias_samples', value=2)])
    class SimClock:
        def now(self):
            return Time(seconds=1.)
    monkeypatch.setattr(instance, 'get_clock', lambda: SimClock())
    instance.history.extend([(.98, np.zeros(5)), (1., np.zeros(5))])
    instance.joint_stamp = instance.odom_stamp = 1.
    instance.last_joint_wall = instance.last_odom_wall = time.monotonic()
    instance.stationary = instance.joint_stationary = True
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def sample(seconds=.99, value=1.):
    msg = WrenchStamped()
    msg.header.stamp = Time(seconds=seconds).to_msg()
    msg.header.frame_id = 'follower/Joint_5/wrist_ft_sensor'
    msg.wrench.force.y = value
    return msg


def test_simulation_wall_watchdog_requires_advancing_clock_even_for_slow_frames(node,monkeypatch):
    node.source_watchdog=True
    node.last_wall=node.last_joint_wall=node.last_odom_wall=0.
    wall=time.monotonic()
    assert not node.wall_input_expired(wall)
    assert node.wall_input_expired(wall+.51)
    class AdvancedClock:
        def now(self):return Time(seconds=1.004)
    monkeypatch.setattr(node,'get_clock',lambda:AdvancedClock())
    assert not node.wall_input_expired(wall+.52)


def test_inputs_are_only_local_and_wrench_frame_has_tcp_origin(node, monkeypatch):
    assert {s.topic_name for s in node.subscriptions} == {
        '/clock', '/follower/ft_sensor', '/follower/joint_states',
        '/follower/mecanum_drive_controller/odometry'}
    output, transforms = [], []
    monkeypatch.setattr(node.raw_pub, 'publish', output.append)
    monkeypatch.setattr(node.tf_pub, 'publish', transforms.append)
    node.wrench(sample())
    node.tick()
    assert node.state == 'UNTARED'
    assert output[0].header.frame_id == 'follower/grasp_aligned'
    assert output[0].wrench.torque.z == pytest.approx(-.08)
    assert transforms[0].transforms[0].transform.translation.x == pytest.approx(.58)


def test_bad_data_and_missing_sensor_invalidate_bias(node, monkeypatch):
    filtered = []
    monkeypatch.setattr(node.filtered_pub, 'publish', filtered.append)
    node.bias, node.filtered = np.zeros(6), np.zeros(6)
    node.wrench(sample(value=float('nan')))
    node.tick()
    assert node.bias is None and not filtered and node.state.startswith('INVALID')
    node.bias, node.filtered = np.zeros(6), np.zeros(6)
    node.last_stamp, node.last_wall = .9, time.monotonic()
    node.tick()
    assert node.bias is None and not filtered


def test_reordered_sensor_cannot_enter_tare_window(node):
    node.wrench(sample())
    node.tick()
    node.wrench(sample(.98))
    assert node.state.startswith('INVALID') and not node.tare_window


def test_opt_in_measurement_processing_publishes_without_waiting_for_wall_timer(node,monkeypatch):
    node.process_on_measurement=True
    output=[];monkeypatch.setattr(node.raw_pub,'publish',output.append)
    node.wrench(sample())
    assert len(output)==1 and not node.pending
    assert output[0].header.stamp==sample().header.stamp
    node.bias,node.filtered=np.zeros(6),np.zeros(6)
    node.last_joint_wall-=1.
    node.wall_watchdog()
    assert node.bias is None and node.state.startswith('INVALID')


@pytest.mark.parametrize('bad_input', ['certificate', 'remap'])
def test_forbidden_calibration_or_input_remap_is_refused(model_file, bad_input):
    rclpy.init(args=[])
    cert = json.loads(model_file[1].read_text())
    arguments = []
    if bad_input == 'certificate':
        cert['robot'] = 'leader'
        model_file[1].write_text(json.dumps(cert))
        expected = 'certificate'
    else:
        arguments = ['--ros-args', '-r', 'ft_sensor:=/follower/odom']
        expected = 'boundary'
    try:
        with pytest.raises(ValueError, match=expected):
            WrenchPreprocessor(namespace='/follower', cli_args=arguments, parameter_overrides=[
                Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(model_file[0])),
                Parameter('calibration_file', value=str(model_file[1]))])
    finally:
        rclpy.shutdown()
