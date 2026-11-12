"""Exercise the real bounded update and stop path without any actuators."""
import json
import math
import time
from types import SimpleNamespace

import pytest
from std_msgs.msg import Bool, String
from amir_compliance_control.grip_maintenance import GripMaintenance, closing_target
from amir_compliance_control.native_grasp import GraspLatch, NativeGraspMonitor


def test_correction_uses_actual_position_and_existing_small_positive_step():
    assert closing_target(-.65, [16., 19.], [True, True], (-1., .26)) == pytest.approx(-.6498)
    assert closing_target(-.65, [18., 21.], [True, True], (-1., .26)) is None
    # Repeated stalled measurements give the same bounded goal, no wind-up.
    assert closing_target(-.65, [16., 19.], [True, True], (-1., .26)) == pytest.approx(-.6498)


@pytest.mark.parametrize('q,forces,contacts', [
    (-.65,[math.nan,19.],[True,True]),(-.65,[-.1,19.],[True,True]),
    (-.65,[16.,26.1],[True,True]),(-.65,[16.,23.],[True,True]),
    (-.65,[16.,19.],[True]),(.2599,[16.,19.],[True,True]),
    (math.inf,[16.,19.],[True,True]),(-1.1,[16.,19.],[True,True]),
])
def test_missing_hold_or_force_or_joint_margin_never_generates_regrasp(q, forces, contacts):
    with pytest.raises(ValueError):
        closing_target(q, forces, contacts, (-1., .26))


class ImmediateAck:
    def __init__(self, accepted):
        self.accepted = accepted

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return SimpleNamespace(accepted=self.accepted)


def node_fixture():
    node = GripMaintenance.__new__(GripMaintenance)
    node.state, node.reason = 'READY', ''
    node.hold_safe_force_imbalance=False
    node.fault_context, node.sample_wall = {}, time.monotonic()
    node.permitted, node.ever_permitted = True, True
    node.permit_wall = time.monotonic()
    node.last_sim, node.clock_wall = 10., time.monotonic()
    node.now = lambda: 10.
    node.fresh = lambda: True
    node.last_command_sim = -math.inf
    node.pending, node.pending_wall = None, 0.
    node.sent, node.accepted, node.command = 0, 0, None
    node.initial_hold = None
    node.low_force_since = 9.6
    node.sample = {'positions': {'Gripper': -.65}, 'own_forces_N': {'left': 16., 'right': 19.},
                   'own_contacts': {'left': True, 'right': True}}
    node.limits = (-1., .26)
    node.source_hash = 'test-source'
    node.stops, node.debug, node.goals = [], [], []
    node.stop_pub = SimpleNamespace(publish=node.stops.append)
    node.debug_pub = SimpleNamespace(publish=node.debug.append)
    def send(goal):
        node.goals.append(goal)
        return ImmediateAck(True)
    node.client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send)
    return node


def test_actual_tick_sends_one_bounded_goal_and_uses_simulation_time_rate_limit():
    node = node_fixture()
    node.tick()
    assert node.sent == node.accepted == 1
    assert node.goals[0].command.position == pytest.approx(-.6498)


    node.tick()
    assert node.sent == 1
    node.now = lambda: 10.501
    node.tick()
    assert node.sent == 1
    node.sample['positions']['Gripper'] = -.6498
    node.tick()
    assert node.sent == 1
    node.now = lambda: 10.802
    node.tick()
    assert node.sent == 2
    assert node.goals[1].command.position == pytest.approx(-.6496)


def test_opt_in_imbalance_withholds_closing_but_keeps_absolute_force_stop():
    node=node_fixture();node.hold_safe_force_imbalance=True;node.command=-.65
    node.sample['own_forces_N']={'left':14.991105040163074,'right':23.255224080021293}
    node.tick()
    assert node.state=='ACTIVE' and not node.stops and not node.goals
    assert node.command==-.65
    node.sample['own_forces_N']['right']=26.1
    node.tick()
    assert node.state=='FAULT' and node.stops and not node.goals


def test_passive_gripper_excursion_cannot_lower_or_wind_up_the_held_goal():
    node = node_fixture()
    node.tick()
    node.now = lambda: 10.6
    node.sample['positions']['Gripper'] = -.651
    node.tick()
    assert node.sent == 1 and node.command == pytest.approx(-.6498)
    # A passive excursion past the target also cannot cause an opening goal.
    node.sample['positions']['Gripper'] = -.649
    node.tick()
    assert node.sent == 1


def test_brief_low_force_does_not_nudge_the_base_during_settling():
    node = node_fixture()
    node.low_force_since = None
    node.tick()
    assert not node.goals
    node.now = lambda: 10.084
    node.sample['own_forces_N']['left'] = 17.1
    node.tick()
    assert node.low_force_since is None and not node.goals
    node.now = lambda: 10.2
    node.sample['own_forces_N']['left'] = 16.
    node.tick()
    node.now = lambda: 10.49
    node.tick()
    assert not node.goals
    node.now = lambda: 10.501
    node.tick()
    assert len(node.goals) == 1


@pytest.mark.parametrize('fault', ['permit', 'stop', 'stale', 'overload', 'ack_timeout', 'server'])
def test_real_tick_latches_and_never_restarts_or_opens_after_recovery(fault):
    node = node_fixture()
    if fault == 'permit': node.permit(Bool(data=False))
    if fault == 'stop': node.stop(Bool(data=True))
    if fault == 'stale': node.fresh = lambda: False
    if fault == 'overload': node.sample['own_forces_N']['right'] = 27.
    if fault == 'contact': node.sample['own_contacts']['left'] = False
    if fault == 'ack_timeout': node.pending, node.pending_wall = object(), time.monotonic()-1.
    if fault == 'server': node.client.server_is_ready = lambda: False
    node.tick()
    assert node.state == 'FAULT' and node.stops and not node.goals
    node.fresh = lambda: True
    node.sample['own_forces_N']['right'] = 19.
    node.sample['own_contacts']['left'] = True
    node.permit(Bool(data=True))
    node.tick()
    assert node.state == 'FAULT' and not node.goals


@pytest.mark.parametrize('transient', ['force', 'contact'])
def test_loss_confirmation_never_commands_regrasp_and_parent_fault_still_latches(transient):
    node = node_fixture()
    if transient == 'force': node.sample['own_forces_N']['left'] = .3
    else: node.sample['own_contacts']['left'] = False
    node.tick()
    assert node.state == 'ACTIVE' and not node.goals
    node.expected_monitor_hash = 'own-monitor'
    node.monitor(String(data=json.dumps({'state':'FAULT','healthy':False,'sim_time':10.,
                                        'source_sha256':'own-monitor'})))
    node.tick()
    assert node.state == 'FAULT' and not node.goals
    node.permit(Bool(data=True))
    node.tick()
    assert node.state == 'FAULT' and not node.goals


def test_action_rejection_uses_shared_stop_and_retains_the_last_bounded_goal():
    node = node_fixture()
    node.client.send_goal_async = lambda goal: ImmediateAck(False)
    node.tick()
    assert node.state == 'FAULT' and node.stops and node.command == pytest.approx(-.6498)
    node.tick()
    assert node.sent == 1


def test_no_gripper_correction_during_initial_disarm():
    node = node_fixture()
    node.permitted, node.ever_permitted = False, False
    node.stop(Bool(data=True))
    node.tick()
    assert node.state == 'READY' and not node.goals


def test_independent_monitor_latches_maintenance_failure_after_first_permit():
    latch = GraspLatch()
    assert latch.takeover(True, True)
    probe = SimpleNamespace(latch=latch, trip=latch.trip)
    msg = String(data=json.dumps({'enabled':False, 'state':'FAULT'}))
    NativeGraspMonitor.maintenance(probe, msg)
    assert latch.state == 'HOLDING'
    latch.permit(True)
    NativeGraspMonitor.maintenance(probe, msg)
    assert latch.state == 'FAULT'


@pytest.mark.parametrize('bad', ['none', 'previous_permit', 'preparation_writer', 'wrong_monitor', 'late_monitor', 'fault', 'low_force','no_contact'])
def test_explicit_enable_requires_retirement_and_no_previous_motion_permit(bad):
    node = node_fixture()
    node.robot, node.state = 'follower', 'DISABLED'
    node.permitted, node.ever_permitted = False, False
    node.sample['ever_permitted'] = False
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='native_grasp_monitor', node_namespace='/follower')]
    node.get_node_names_and_namespaces = lambda: []
    if bad == 'previous_permit': node.ever_permitted = True
    if bad == 'preparation_writer': node.get_node_names_and_namespaces = lambda: [('cp9_contact_preparation','/follower')]
    if bad == 'wrong_monitor': node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(node_name='native_grasp_monitor',node_namespace='/leader')]
    if bad == 'late_monitor': node.sample['ever_permitted'] = True
    if bad == 'fault': node.state = 'FAULT'
    if bad == 'low_force': node.sample['own_forces_N']['left'] = .3
    if bad == 'no_contact': node.sample['own_contacts']['left'] = False
    response = node.enable(None, SimpleNamespace())
    assert response.success == (bad == 'none')
    assert not node.goals
    if bad == 'none':
        assert node.command == node.initial_hold == -.65
        # The first correction also cannot open relative to the handoff.
        node.permitted, node.ever_permitted = True, True
        node.sample['positions']['Gripper'] = -.651
        node.tick()
        assert not node.goals and node.command == -.65


@pytest.mark.parametrize('remap', [None,
    'grasp/monitor_debug:=/leader/grasp/monitor_debug',
    'gripper_controller/gripper_cmd:=/leader/gripper_controller/gripper_cmd',
    'gripper_controller/gripper_cmd/_action/send_goal:=/leader/gripper_controller/gripper_cmd/_action/send_goal',
    '/transport/safety_permit:=/leader/velocity', '/clock:=/leader/tf'])
def test_real_ros_node_rejects_input_and_action_remaps_before_creating_actuator_client(remap):
    from pathlib import Path
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    root = Path(__file__).resolve().parents[4]
    context = Context()
    rclpy.init(context=context, args=[])
    node = None
    try:
        kwargs = dict(context=context, namespace='/follower',
            parameter_overrides=[Parameter('use_sim_time', value=True),
                Parameter('urdf_file', value=str(root/'audits/whole_body_compliance_20261019/cp9_transport/generated/follower.urdf'))],
            cli_args=['--ros-args', '-r', remap] if remap else [])
        if remap:
            with pytest.raises(ValueError, match='boundary remapped'):
                GripMaintenance(**kwargs)
        else:
            node = GripMaintenance(**kwargs)
            assert {s.topic_name for s in node.subscriptions} == {
                '/clock','/follower/grasp/monitor_debug','/transport/safety_permit','/transport/emergency_stop'}
    finally:
        if node is not None: node.destroy_node()
        context.shutdown()
