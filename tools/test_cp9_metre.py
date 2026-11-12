"""Metre goal wiring and simulation tuning bounds, without starting Gazebo."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters
from amir_compliance_control.transport import straight_reference
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.whole_body import WholeBodyAllocator, whole_jacobian
from amir_compliance_control.planar_leader import PlanarLeader
from nav_msgs.msg import Odometry


ROOT = Path(__file__).resolve().parent.parent
GENERATED = ROOT / 'audits/whole_body_compliance_20261019/cp9_transport/generated'


def load(relative):
    spec = importlib.util.spec_from_file_location(Path(relative).stem, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_public_launch_defaults_are_one_metre_and_faster_speed():
    module = load('src/humbleble_ws/cooperative_transport_bringup/launch/compliance_contact_transport.launch.py')
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert float(context.launch_configurations['forward_distance_m']) == 1.
    assert float(context.launch_configurations['translation_velocity_m_s']) == .005


def policies(profile):
    module = load('src/humbleble_ws/amir_compliance_control/launch/whole_body_transport.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'run_directory': str(GENERATED), 'calibration_file': 'test-unused.json',
        'mode': 'WHOLE_BODY', 'yaw_profile': 'filtered', 'leader_speed': '.005',
        'leader_max_distance_m': '1.0', 'allow_ungrasped_test': 'false',
        'translation_profile': profile,
    })
    nodes = module.setup(context)
    assert len(nodes) == 3  # leader, local follower, local wrench processor
    return [dict(evaluate_parameters(context, n._Node__parameters)[0]) for n in nodes]


def test_metre_envelope_only_goes_to_leader_and_does_not_override_safety_conditions():
    leader, follower, _ = policies('metre_trial')
    assert leader['max_distance_m'] == 1. and leader['speed_m_s'] == .005
    assert leader['terminal_timeout_s'] == 10.
    assert 'max_distance_m' not in follower and 'speed_m_s' not in follower
    assert follower['allow_ungrasped_test'] is False and follower['actuator_ack_required'] is True
    profile = __import__('json').loads((GENERATED / 'follower_safety.json').read_text())
    assert np.all(np.array(follower['base_velocity_limits']) <= profile['max_base_velocity'])
    assert not {'displacement_limit_m', 'height_error_limit_m', 'rotation_error_limit_rad',
                'timeout_s', 'joint_request_speed_rad_s'} & follower.keys()


def test_original_micro_transport_keeps_its_original_control_defaults():
    leader, follower, _ = policies('baseline')
    assert leader['terminal_timeout_s'] == 3. and leader['acceleration_m_s2'] == .001
    assert follower['allocation_base_filter_hz'] == .2
    assert 'base_gains' not in follower and 'base_velocity_limits' not in follower


def test_one_metre_reference_reaches_metres_with_finite_duration_and_bounds():
    acceleration = policies('metre_trial')[0]['reference_acceleration_m_s2']
    _, _, duration = straight_reference(0., 1., .005, acceleration)
    assert duration == pytest.approx(220.)
    ramp = .005 / acceleration
    times = np.unique(np.r_[np.linspace(0., duration, 20001), ramp, duration - ramp])
    velocities = np.array([straight_reference(t, 1., .005, acceleration)[1] for t in times])
    integral = np.sum(.5 * (velocities[1:] + velocities[:-1]) * np.diff(times))
    assert integral == pytest.approx(1., abs=1e-8)
    assert max(abs(velocities)) <= .005
    assert max(abs(np.diff(velocities) / np.diff(times))) <= acceleration + 1e-12
    assert straight_reference(duration + 1., 1., .005, acceleration)[:2] == (1., 0.)


def test_metre_allocator_retains_arm_first_authority_and_all_bounds():
    _, follower, _ = policies('metre_trial')
    xml = (GENERATED / 'follower.urdf').read_text()
    model = LocalModel(xml)
    tree = ET.fromstring(xml)
    q = np.array([float(tree.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text)
                  for n in model.names])
    allocator = WholeBodyAllocator(model, base_speed=follower['base_velocity_limits'],
        base_acceleration=follower['base_acceleration_limits'],
        base_filter_hz=follower['allocation_base_filter_hz'])
    desired = np.array([.005, 0., 0., 0., 0., 0.])
    velocity = allocator.step(q, q, desired, np.zeros(3), 0., .004)
    assert not velocity[5:].any() and velocity[:5].any()
    for _ in range(400):
        previous = velocity.copy()
        velocity = allocator.step(q, q, desired, [.005, 0., 0.], 1., .004)
        assert np.all(abs(velocity) <= allocator.scale + 1e-12)
        assert np.all(abs(velocity - previous) <= allocator.acceleration * .004 + 1e-12)
        q += velocity[:5] * .004
        assert np.all(q > model.lower + .01) and np.all(q < model.upper - .01)
    assert (whole_jacobian(model, q) @ velocity)[0] > .004


@pytest.mark.parametrize(('timeout', 'tripped'), [(3., True), (10., False)])
def test_terminal_deadline_retains_default_and_bounds_extended_settling(timeout, tripped):
    policy = {'speed_m_s': .005, 'acceleration_m_s2': .00025,
              'reference_acceleration_m_s2': 0.,
              'yaw_metric_m': .1, 'goal_tolerance_m': .000002,
              'position_gain_s_inv': 1., 'terminal_timeout_s': timeout}
    duration = straight_reference(0., 1., .005, .00025)[2]
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.
    odom.pose.pose.position.x = 1. - .00001
    faults = []
    node = SimpleNamespace(state='TRANSPORT', encoder=odom, sim=duration + 4.,
        path_goal=(0., np.array([1., 0., 0.]), 1., np.zeros(3)),
        planar_velocity=np.zeros(3), last_tick_sim=duration + 4. - .004,
        p=policy.__getitem__, trip=faults.append)
    PlanarLeader.base_request(node, None)
    assert bool(faults) is tripped
    if not tripped:
        node.sim = duration + timeout + .1
        PlanarLeader.base_request(node, None)
        assert faults == ['leader local planar tracking timeout']


@pytest.mark.parametrize('tracking_headroom', [False, True])
def test_long_ramp_needs_tracking_acceleration_headroom_with_delayed_encoder(tracking_headroom):
    policy = policies('metre_trial')[0]
    policy.update(yaw_metric_m=.1, goal_tolerance_m=.000002, position_gain_s_inv=1.)
    if not tracking_headroom:
        policy['acceleration_m_s2'] = policy['reference_acceleration_m_s2']
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.
    faults = []
    node = SimpleNamespace(state='TRANSPORT', encoder=odom, sim=0.,
        path_goal=(0., np.array([1., 0., 0.]), 1., np.zeros(3)),
        planar_velocity=np.zeros(3), last_tick_sim=0., completed=0,
        p=policy.__getitem__, trip=faults.append)
    # Deliberately delay the synthetic plant/encoder by three 20 ms ticks.
    # Run the production tracking method through the whole metre trajectory.
    pending, positions, commands = [0.] * 3, [], []
    for i in range(12000):
        node.sim = i * .02
        velocity = PlanarLeader.base_request(node, None)[0]
        node.last_tick_sim = node.sim
        pending.append(velocity)
        odom.pose.pose.position.x += pending.pop(0) * .02
        positions.append(odom.pose.pose.position.x)
        commands.append(velocity)
        if node.completed or faults:
            break
    if tracking_headroom:
        assert node.completed == 1 and not faults
        assert abs(positions[-1] - 1.) <= .000002
        assert max(positions) - 1. < .0001
        assert max(abs(np.array(commands))) <= .005
        assert max(abs(np.diff(commands))) <= policy['acceleration_m_s2'] * .02 + 1e-12
    else:
        assert not node.completed and faults == ['leader local planar tracking timeout']
