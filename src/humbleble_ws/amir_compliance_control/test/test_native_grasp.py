"""Safety transition and physical joint boundary regressions for native handoff."""
import math
import pytest
from amir_compliance_control.native_grasp import GraspLatch, NativeGraspMonitor, measured_joints_ok, released_arm_writer
from geometry_msgs.msg import WrenchStamped
from types import SimpleNamespace


def test_initial_disarm_and_fault_after_first_permit_are_distinct():
    latch = GraspLatch()
    assert latch.takeover(True, True)
    latch.stop(True)
    assert latch.state == 'HOLDING'
    latch.permit(True)
    latch.stop(True)
    assert latch.state == 'FAULT'
    latch.stop(False)
    latch.permit(True)
    assert not latch.takeover(True, True)
    assert latch.state == 'FAULT'


@pytest.mark.parametrize('healthy,exclusive', [(False,True),(True,False),(False,False)])
def test_handoff_does_not_bypass_missing_sensor_or_duplicate_writer(healthy, exclusive):
    latch = GraspLatch()
    assert not latch.takeover(healthy, exclusive)
    assert latch.state == 'WAITING'


def test_local_fault_before_transport_arm_latches_and_cannot_retakeover():
    latch = GraspLatch(); latch.takeover(True, True)
    latch.trip('overload')
    latch.trip('secondary stop')
    assert latch.reason == 'overload'
    assert not latch.takeover(True, True)


def test_initial_sensor_discovery_does_not_create_motion_or_reset_a_fault():
    latch=GraspLatch()
    assert not latch.takeover(False,True)
    latch.sensor_freshness(False)
    assert latch.state=='WAITING'
    assert latch.takeover(True,True)
    latch.sensor_freshness(False)
    assert latch.state=='HOLDING' and not latch.ever
    latch.trip('nonfinite sensor during startup')
    latch.sensor_freshness(True)
    assert latch.state=='FAULT'


def test_outage_after_first_permit_latches_even_after_input_and_permit_return():
    latch=GraspLatch();latch.takeover(True,True);latch.permit(True)
    latch.sensor_freshness(False)
    assert latch.state=='FAULT'
    latch.sensor_freshness(True);latch.permit(True)
    assert latch.state=='FAULT' and not latch.takeover(True,True)


@pytest.mark.parametrize('writers,good', [([],True),([('actuator_interlock','/follower')],True),
    ([('cp9_local_grasp','/follower')],False),([('actuator_interlock','/leader')],False),
    ([('actuator_interlock','/follower'),('cp9_local_grasp','/follower')],False)])
def test_only_actual_local_interlock_can_remain(writers,good):
    assert released_arm_writer(writers,'follower') == good


@pytest.mark.parametrize('bad', ['none','missing','nonfinite','master_limit','mimic_limit','tracking','arm_limit'])
def test_own_model_limits_and_measured_mimic_relation(bad):
    limits={'Joint_1':(-1.,1.),'Gripper':(-1.,.2)}
    mimics={'finger':(-1.,.2,-1.,-.7)}
    positions={'Joint_1':0.,'Gripper':-.5,'finger':-.2}
    if bad=='missing':del positions['finger']
    if bad=='nonfinite':positions['finger']=math.nan
    if bad=='master_limit':positions['Gripper']=2.
    if bad=='mimic_limit':positions['finger']=-2.
    if bad=='tracking':positions['finger']=0.
    if bad=='arm_limit':positions['Joint_1']=1.001
    assert measured_joints_ok(positions,limits,mimics) == (bad=='none')


@pytest.mark.parametrize('bad',['nan','torque_nan','overload','wrong_frame','repeated_stamp'])
def test_actual_wrench_callback_latches_bad_data_without_motion_publisher(bad):
    state=GraspLatch();state.takeover(True,True)
    probe=SimpleNamespace(robot='follower',forces={},data={},trip=state.trip)
    probe.receive=lambda k,s: NativeGraspMonitor.receive(probe,k,s)
    msg=WrenchStamped();msg.header.stamp.sec=10
    msg.header.frame_id='follower/left_inner_front_joint/finger_left_force_sensor'
    msg.wrench.force.x=20.
    if bad=='nan':msg.wrench.force.x=math.nan
    if bad=='torque_nan':msg.wrench.torque.z=math.nan
    if bad=='overload':msg.wrench.force.x=27.
    if bad=='wrong_frame':msg.header.frame_id='leader/left_inner_front_joint/finger_left_force_sensor'
    if bad=='repeated_stamp':probe.data['left']=(10.,0.)
    NativeGraspMonitor.force(probe,'left',msg)
    assert state.state=='FAULT'


@pytest.mark.parametrize('bad',['inactive','wrong_robot','malformed'])
def test_actual_gripper_audit_callback_trips_after_permit(bad):
    from std_msgs.msg import String
    import json
    state=GraspLatch();state.takeover(True,True);state.permit(True)
    probe=SimpleNamespace(robot='follower',latch=state,trip=state.trip)
    value={'robot':'leader' if bad=='wrong_robot' else 'follower','controllers':{'gripper_controller':'inactive'}}
    msg=String(data='invalid' if bad=='malformed' else json.dumps(value))
    NativeGraspMonitor.audit(probe,msg)
    assert state.state=='FAULT'
