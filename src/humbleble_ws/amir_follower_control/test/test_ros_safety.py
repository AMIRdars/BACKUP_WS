"""Exercise the real ROS node's gates and resolved input boundary."""
from pathlib import Path

import pytest
import rclpy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from amir_follower_control.core import allowed_subscriptions
from amir_follower_control.node import FollowerNode


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-r','__ns:=/amir2'])
    from ament_index_python.packages import get_package_share_directory
    import xacro
    source=Path(get_package_share_directory('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
    xml=xacro.process_file(str(source),mappings={'namespace':'amir2'}).toxml()
    # Parameters passed exactly as a launch would, before node initialization.
    from unittest.mock import patch
    from rclpy.node import Node
    base_init=Node.__init__
    def init_with_overrides(self,*args,**kwargs):
        kwargs['parameter_overrides']=[Parameter('use_sim_time',value=True),
                                       Parameter('robot_description',value=xml)]
        base_init(self,*args,**kwargs)
    with patch.object(Node,'__init__',init_with_overrides):
        instance=FollowerNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_runtime_subscriptions_are_only_local(node):
    actual={s.topic_name for s in node.subscriptions}
    assert actual == allowed_subscriptions('amir2')
    assert '/amir2/odom' not in actual


def test_missing_physical_calibration_cannot_enable(node):
    result=node._calibrate(Trigger.Request(),Trigger.Response())
    assert not result.success
    assert 'certificate' in result.message
    result=node._enable(SetBool.Request(data=True),SetBool.Response())
    assert not result.success and not node.owned


@pytest.mark.parametrize('state', ['READY', 'CALIBRATING', 'FAULT'])
def test_disable_cancels_readiness_but_preserves_fault_latch(node, state):
    node.state = state
    result = node._enable(SetBool.Request(data=False), SetBool.Response())
    assert result.success and not node.owned and not node.enabled
    assert node.state == ('FAULT' if state == 'FAULT' else 'DISABLED')


def test_controller_discovery_is_a_non_driving_calibration_precondition(node):
    node.certificate={}
    node.sign=1.
    result=node._calibrate(Trigger.Request(),Trigger.Response())
    assert not result.success and 'after discovery' in result.message
    assert node.state=='DISABLED' and not node.owned


def test_duplicate_controller_with_same_name_cannot_take_ownership(node,monkeypatch):
    from types import SimpleNamespace
    node.state='READY'
    node.controllers_ok=True
    writer=SimpleNamespace(node_name=node.get_name(),node_namespace=node.get_namespace())
    monkeypatch.setattr(node,'get_publishers_info_by_topic',lambda topic:[writer,writer])
    result=node._enable(SetBool.Request(data=True),SetBool.Response())
    assert not result.success and 'conflict' in result.message and not node.owned


def test_base_calibration_cannot_bypass_four_axis_integration_gate(node,monkeypatch):
    node.state='READY'
    node.certificate={'base_axes_verified':True,'phase2_axes_passed':['fx']}
    original=node.p
    monkeypatch.setattr(node,'p',lambda name:'C' if name=='mode' else original(name))
    result=node._enable(SetBool.Request(data=True),SetBool.Response())
    assert not result.success and 'four-axis' in result.message and not node.owned


def test_remapping_to_truth_is_rejected(node, monkeypatch):
    real=node.resolve_topic_name
    monkeypatch.setattr(node,'resolve_topic_name',
                        lambda name: '/amir2/odom' if name.endswith('odometry') else real(name))
    with pytest.raises(ValueError,match='whitelist'):
        node._audit_subscriptions()


def test_missing_data_fault_latches_and_holds_gripper(node):
    node.state='FOLLOWING'
    node.enabled=True
    node.last_tick=node.get_clock().now().nanoseconds*1e-9-.01
    node._tick()
    assert node.state=='FAULT' and not node.enabled
    assert 'missing local state' in node.fault
    # The process never creates a gripper command publisher.
    assert not any('gripper' in p.topic_name for p in node.publishers)


def test_disabled_clock_jump_does_not_take_command_ownership(node):
    node.last_tick=-100
    node._tick()
    assert node.state=='DISABLED' and not node.owned


def test_disabled_node_reports_status_without_acquiring_motion(node,monkeypatch):
    states=[]
    monkeypatch.setattr(node.status_pub,'publish',lambda msg:states.append(msg.data))
    node.last_tick=-.01
    node._tick()
    assert states==['DISABLED'] and not node.owned


def test_pause_does_not_integrate_or_publish(node):
    node.state='FOLLOWING'
    node.owned=True
    node.last_tick=node.get_clock().now().nanoseconds*1e-9
    before=node.adm.x.copy()
    node._tick()
    assert (node.adm.x==before).all()
    assert node.state=='FOLLOWING'


def prepare_aligned_state(node, monkeypatch, raw_fx=0.):
    class TestClock:
        def now(self):
            return Time(seconds=1.)
    monkeypatch.setattr(node,'get_clock',lambda: TestClock())
    q=np.array([0,1.4,-1.6,.2,1.570796])
    for stamp,offset in [(980000000,0.),(1000000000,.002)]:
        m=JointState(name=list(node.arm.names),position=(q+offset).tolist(),velocity=[0.]*5)
        m.header.stamp=Time(nanoseconds=stamp).to_msg()
        node._receive('joints',m)
    wrench=WrenchStamped()
    wrench.header.stamp=Time(nanoseconds=990000000).to_msg()
    wrench.wrench.force.x=raw_fx
    node._receive('wrench',wrench)
    odom=Odometry()
    odom.header.stamp=Time(seconds=1.).to_msg()
    node._receive('odom',odom)
    node.sign=1.
    node.grasp=True
    node.grasp_seen=1.
    node.controllers_ok=True
    node.controller_checked=1.
    node.last_tick=.99
    node.state='CALIBRATING'
    return q+.001


def test_real_node_uses_interpolated_sensor_transform(node, monkeypatch):
    q_at_sensor=prepare_aligned_state(node,monkeypatch)
    import amir_follower_control.node as module
    real=module.transform_wrench
    transforms=[]
    def observe(raw,ts,pg,sign):
        transforms.append(ts.copy())
        return real(raw,ts,pg,sign)
    monkeypatch.setattr(module,'transform_wrench',observe)
    node._tick()
    assert node.state=='CALIBRATING'
    assert node.observer.count==1
    np.testing.assert_allclose(transforms[0],node.arm.evaluate(q_at_sensor)[1],atol=1e-12)


def test_raw_overload_cannot_be_hidden_in_hold_bias(node, monkeypatch):
    prepare_aligned_state(node,monkeypatch,raw_fx=1000.)
    node._tick()
    assert node.state=='FAULT' and 'raw six-axis' in node.fault
    assert node.observer.count==0 and not node.owned


def test_fault_holds_fresh_measurement_instead_of_old_target(node, monkeypatch):
    prepare_aligned_state(node,monkeypatch)
    node.target=np.array([0.,1.4,-1.6,.2,1.570796])
    node.owned=True
    commands=[]
    monkeypatch.setattr(node,'_arm_command',lambda q,dq,h:commands.append((q.copy(),dq.copy())))
    node._trip('test overload')
    np.testing.assert_allclose(commands[0][0],node.samples['joints'].position)
    np.testing.assert_array_equal(commands[0][1],np.zeros(5))
    assert node.state=='FAULT' and not node.owned


def test_stale_actual_grasp_heartbeat_prevents_calibration(node, monkeypatch):
    prepare_aligned_state(node,monkeypatch)
    node.grasp_seen=.5
    node._tick()
    assert node.state=='FAULT' and 'heartbeat' in node.fault
    assert node.observer.count==0


def test_measured_wrist_axis_torque_respects_urdf_effort(node, monkeypatch):
    prepare_aligned_state(node,monkeypatch)
    node.samples['wrench'].wrench.torque.y=4.1
    node._tick()
    assert node.state=='FAULT' and 'URDF effort' in node.fault
    assert node.observer.count==0


def test_wrist_load_cannot_overload_upstream_joint_before_biasing(node,monkeypatch):
    prepare_aligned_state(node,monkeypatch,raw_fx=49.)
    node._tick()
    assert node.state=='FAULT' and 'quasistatic model load' in node.fault
    assert 'Joint_2' in node.fault
    assert node.observer.count==0 and not node.owned
