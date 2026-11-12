"""Aligned own data may bridge arrival skew, never a new faulty measurement."""
import hashlib,json,time
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from amir_compliance_control.rotation_observer import observer_type,ProjectedYaw,qualify_passive
from test_local_yaw import URDF,HOME
from test_rotation_integration import passive_fixture


@pytest.fixture
def aligned(tmp_path):
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    cert=qualify_passive(passive_fixture(),g,force_limit_N=.5,torque_limit_Nm=.05,evidence_sha256={'synthetic':'unit'})
    cert.update(robot='follower',urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest())
    file=tmp_path/'cert.json';file.write_text(json.dumps(cert))
    rclpy.init();node=None
    try:
        node=observer_type()(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(URDF)),Parameter('qualification_file',value=str(file)),
            Parameter('yaw_heading_frame',value='grasp_heading'),Parameter('orientation_envelope_rad',value=.01),
            Parameter('joint_margin_rad',value=.01),Parameter('align_own_source_history',value=True)])
        q=HOME.copy();q[4]=.3
        def send(topic,t,position=None):
            if topic=='joint_states':msg=JointState(name=list(g.model.names),position=(q if position is None else position).tolist(),velocity=[0.]*5)
            elif topic=='wrench_at_grasp_raw':msg=WrenchStamped()
            else:msg=Odometry()
            msg.header.stamp=Time(seconds=t).to_msg();node.receive(topic,msg);return msg
        node.get_clock().set_ros_time_override(Time(seconds=10.))
        for topic in node.source_history:send(topic,10.)
        node.receive('grasp/state',String(data='attached'))
        assert node.latch(Trigger.Request(),Trigger.Response()).success
        node.armed=True
        for topic in ['joint_states','mecanum_drive_controller/odometry']:
            for t in [10.008,10.012,10.02,10.04]:send(topic,t)
        send('wrench_at_grasp_raw',10.01)
        node.get_clock().set_ros_time_override(Time(seconds=10.04))
        assert node.observation()['yaw_valid']
        yield node,send,q
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()


def test_actual_history_aligns_30ms_arrival_skew_within_original_20ms_source_bound(aligned):
    node,_,_=aligned;o=node.observation()
    assert o['yaw_valid'] and o['joint_stamp']==pytest.approx(10.04)
    assert o['health_source_stamps']['joint_states']==pytest.approx(10.008)
    assert o['source_skew_s']<=.02 and o['latest_source_stamps']['joint_states']==pytest.approx(10.04)
    assert o['continuity_joint_stamp']==pytest.approx(10.04)


@pytest.mark.parametrize('fault',['old','future','new_joint_jump','new_wrench_nan','receipt'])
def test_history_does_not_rescue_bad_latest_input(aligned,fault):
    node,send,q=aligned
    if fault=='old':node.get_clock().set_ros_time_override(Time(seconds=10.061))
    elif fault=='future':send('joint_states',10.1)
    elif fault=='new_joint_jump':
        q=q.copy();q[0]+=.2;send('joint_states',10.04,q)
    elif fault=='new_wrench_nan':
        msg=send('wrench_at_grasp_raw',10.04);msg.wrench.force.x=float('nan')
    else:node.receipt['wrench_at_grasp_raw']=time.monotonic()-.2
    assert not node.observation()['yaw_valid']


def test_no_matching_actual_history_refuses_instead_of_synthesizing_a_sample(aligned):
    node,_,_=aligned
    for key in ['joint_states','mecanum_drive_controller/odometry']:
        node.source_history[key].clear();node.source_history[key].append(node.latest[key])
    assert not node.observation()['yaw_valid']


def test_reference_latch_checks_latest_arm_motion_even_when_aligned_sample_is_static(aligned):
    node,send,_=aligned;node.armed=False
    msg=send('joint_states',10.04);msg.velocity[0]=.01
    assert not node.latch(Trigger.Request(),Trigger.Response()).success


def test_outside_region_still_reports_latched_reference_without_authorizing_motion(aligned):
    node,send,q=aligned;node.armed=False;node.previous=None
    q=q.copy();q[0]=.03
    for key in node.source_history:
        node.source_history[key].clear()
        send(key,10.04,q if key=='joint_states' else None)
    out=node.observation()
    assert not out['yaw_valid'] and 'outside measured' in out['valid_reason']
    assert 'psi_home_B' in out and out['reference_sequence']>0


def test_actual_raw_arrival_publishes_without_waiting_for_periodic_tick(aligned):
    from types import SimpleNamespace
    node,send,_=aligned;node.publish_on_raw=True;published=[]
    node.pub=SimpleNamespace(publish=lambda m:published.append(json.loads(m.data)))
    send('wrench_at_grasp_raw',10.04)
    assert len(published)==1 and published[-1]['yaw_valid']
    node.get_clock().set_ros_time_override(Time(seconds=10.1))
    send('wrench_at_grasp_raw',10.1)
    assert len(published)==2 and not published[-1]['yaw_valid']


@pytest.mark.parametrize('fault',['receipt','source','grasp','wrench'])
def test_latest_snapshot_keeps_a_transient_online_fault_until_verified_latch(aligned,fault):
    from types import SimpleNamespace
    node,send,_=aligned;published=[]
    node.pub=SimpleNamespace(publish=lambda m:published.append(json.loads(m.data)))
    node.tick();assert published[-1]['yaw_valid']
    if fault=='receipt':node.receipt['wrench_at_grasp_raw']=time.monotonic()-.2
    elif fault=='source':node.get_clock().set_ros_time_override(Time(seconds=10.2))
    elif fault=='grasp':node.receive('grasp/state',String(data='detached'))
    else:node.latest['wrench_at_grasp_raw'].wrench.force.x=float('nan')
    node.tick();assert not published[-1]['yaw_valid']
    node.get_clock().set_ros_time_override(Time(seconds=10.04))
    node.receive('grasp/state',String(data='attached'));send('wrench_at_grasp_raw',10.04)
    node.tick();assert not published[-1]['yaw_valid']
    assert 'latched online' in published[-1]['valid_reason']
    assert not node.latch(Trigger.Request(),Trigger.Response()).success
    node.armed=False
    assert node.latch(Trigger.Request(),Trigger.Response()).success
    node.tick();assert published[-1]['yaw_valid']


def test_snapshot_uses_depth_one_without_changing_reliable_raw_event_stream(aligned):
    node,_,_=aligned
    assert node.pub.qos_profile.depth==1
    raw=next(s for s in node.subscriptions if s.topic_name=='/follower/wrench_at_grasp_raw')
    assert raw.qos_profile.depth==10


def test_fk_yaw_uses_latest_joint_and_never_the_older_health_tuple(aligned):
    node,send,q=aligned;node.previous=None;q=q.copy();q[0]=1e-5
    send('joint_states',10.04,q)
    out=node.observation()
    assert out['yaw_valid'] and out['joint_stamp']==pytest.approx(10.04)
    assert out['e_yaw_rad']==pytest.approx(1e-5,abs=1e-12)
    assert out['health_source_stamps']['joint_states']<out['joint_stamp']
