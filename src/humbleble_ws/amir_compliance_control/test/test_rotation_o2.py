"""O2 rate saturation, rigid point geometry and loaded-reference interlock."""
import json
import hashlib
from types import SimpleNamespace
import time
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from amir_compliance_control.rotation_compensation_probe import ProbeRecentering,pulse_rate,CompensationProbe
from amir_compliance_control.rotation_observer import observer_type
from amir_compliance_control.rotation_observer import ProjectedYaw,qualify_passive
from amir_compliance_control.rotation_follow import RotationFollower,own_geometry
from test_local_yaw import HOME,URDF
from test_rotation_integration import passive_fixture


@pytest.mark.parametrize('sign',[-1.,1.])
def test_o2_saturated_rate_keeps_absolute_grasp_point_stationary(sign):
    p=np.array([.47,.012,.66])
    base=ProbeRecentering(lambda:(sign*1.,'test'),mode='XY_YAW_COMP',gains=[0.,0.,0.],
        velocity=[.1,.1,.002],acceleration=[.1,.1,.001],cutoff=.2)
    previous=np.zeros(3)
    for _ in range(20):
        command=base.step([0.,0.,0.],p,.02,yaw_valid=True)
        assert np.all(abs(command-previous)<=base.amax*.02+1e-15)
        np.testing.assert_allclose(np.r_[command[:2],0.]+np.cross([0.,0.,command[2]],p),0.,atol=1e-18)
        assert abs(command[2])<1.
        previous=command
    with pytest.raises(ValueError,match='qualified'):base.step([0.,0.,0.],p,.02,yaw_valid=False)


def test_probe_pulse_has_two_signed_lobes_and_no_residual_angle():
    times=np.linspace(0.,62.,6201);rates=np.array([pulse_rate(t,3e-6,30.,2.)[0] for t in times])
    assert np.max(rates)>0. and np.min(rates)<0.
    assert abs(np.trapz(rates,times))<1e-18
    assert pulse_rate(-1.,3e-6,30.,2.)==(0.,'waiting')
    assert pulse_rate(31.,3e-6,30.,2.)==(0.,'hold')
    assert pulse_rate(63.,3e-6,30.,2.)==(0.,'complete')
    assert not CompensationProbe.may_authorize_rotation(None)


@pytest.mark.parametrize('extra_ns',[0,1])
def test_exact_twenty_millisecond_joint_skew_boundary(extra_ns):
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    measured=g.measure(HOME)
    msg=JointState(name=list(g.model.names),position=HOME.tolist())
    msg.header.stamp=Time(nanoseconds=10_040_000_000+extra_ns).to_msg()
    observed=dict(measured,frame_id='follower/base_link',urdf_sha256='unit',yaw_valid=True,joint_stamp=10.02,
        e_yaw_rad=0.,psi_home_B=measured['psi_ee_B'],p_home_footprint=measured['p_ee_footprint'])
    node=SimpleNamespace(observation=observed,observation_wall=time.monotonic(),p=lambda key:.15,
        sim=10.04,urdf_hash='unit',joints=msg,model=g.model,yaw_geometry=g,core=None,observation_reference=None)
    if extra_ns:
        with pytest.raises(ValueError,match='timestamp skew'):RotationFollower.require_yaw(node)
    else:assert RotationFollower.require_yaw(node)['yaw_valid']


def test_own_joint_history_aligns_observer_without_weakening_source_watchdog():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    m=g.measure(HOME)
    latest=JointState(name=list(g.model.names),position=HOME.tolist());latest.header.stamp=Time(seconds=10.06).to_msg()
    old=JointState(name=list(g.model.names),position=HOME.tolist());old.header.stamp=Time(seconds=10.02).to_msg()
    o=dict(m,frame_id='follower/base_link',urdf_sha256='unit',yaw_valid=True,joint_stamp=10.02,
           e_yaw_rad=0.,psi_home_B=m['psi_ee_B'],p_home_footprint=m['p_ee_footprint'])
    node=SimpleNamespace(observation=o,observation_wall=time.monotonic(),p=lambda key:.15,
        sim=10.06,urdf_hash='unit',joints=latest,yaw_joint_history=[old],model=g.model,yaw_geometry=g,
        core=None,observation_reference=None)
    assert RotationFollower.require_yaw(node)['yaw_valid'] and node.yaw_alignment_ns==0
    node.sim=10.071
    with pytest.raises(ValueError,match='source timestamp watchdog'):RotationFollower.require_yaw(node)


def test_last_wrist_axis_is_analytically_heading_and_tcp_invariant():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    assert g.yaw_invariant_joint_indices==[4]
    initial=g.measure(HOME)
    for delta in np.linspace(-.01,.01,21):
        q=HOME.copy();q[4]+=delta;m=g.measure(q)
        np.testing.assert_allclose(m['p_ee_B'],initial['p_ee_B'],atol=1e-15)
        assert abs(m['psi_ee_B']-initial['psi_ee_B'])<1e-15
    legacy=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01)
    assert legacy.yaw_invariant_joint_indices==[]


def test_own_fk_cache_reuses_identical_sample_but_checks_changed_data(monkeypatch):
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    original=g.measure;calls=[]
    def measured(q,**kwargs):calls.append(q.copy());return original(q,**kwargs)
    monkeypatch.setattr(g,'measure',measured)
    node=SimpleNamespace(model=g.model,yaw_geometry=g)
    msg=JointState(name=list(g.model.names),position=HOME.tolist());msg.header.stamp=Time(seconds=10.).to_msg()
    first=own_geometry(node,msg)
    assert own_geometry(node,msg) is first and len(calls)==1
    msg.position[0]+=.001
    assert own_geometry(node,msg)['psi_ee_B']!=first['psi_ee_B'] and len(calls)==2
    msg.position[0]=float('nan')
    with pytest.raises(ValueError):own_geometry(node,msg)


def test_invariant_wrist_keeps_pose_guard_and_legacy_certificate_region(tmp_path):
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    rows=passive_fixture()
    for row in rows:row['q_rad'][4]=HOME[4]
    cert=qualify_passive(rows,g,force_limit_N=.5,torque_limit_Nm=.05,evidence_sha256={'synthetic':'unit'})
    cert.update(robot='follower',urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest())
    file=tmp_path/'certificate.json';file.write_text(json.dumps(cert))
    rclpy.init();node=None
    try:
        node=observer_type()(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(URDF)),Parameter('qualification_file',value=str(file)),
            Parameter('yaw_heading_frame',value='grasp_heading'),Parameter('orientation_envelope_rad',value=.01),
            Parameter('joint_margin_rad',value=.01)])
        def feed(t,delta):
            node.get_clock().set_ros_time_override(Time(seconds=t));q=HOME.copy();q[4]+=delta
            joint=JointState(name=list(g.model.names),position=q.tolist(),velocity=[0.]*5)
            odom=Odometry();w=WrenchStamped()
            for msg in (joint,odom,w):msg.header.stamp=Time(seconds=t).to_msg()
            for key,msg in [('joint_states',joint),('mecanum_drive_controller/odometry',odom),
                ('wrench_at_grasp_raw',w),('grasp/state',String(data='attached'))]:node.receive(key,msg)
            return node.observation()
        feed(10.,0.);assert node.latch(Trigger.Request(),Trigger.Response()).success
        assert feed(10.02,.005)['yaw_valid']
        node.cert.pop('yaw_invariant_joint_indices')
        assert 'qualification joint region' in feed(10.04,.005)['valid_reason']
        node.cert['yaw_invariant_joint_indices']=[4]
        assert 'pose envelope' in feed(10.06,.02)['valid_reason']
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('fault',[None,'active','moving','stale','wrong_writer','not_ready'])
def test_loaded_reference_requires_verified_inactive_stationary_controller(fault):
    rclpy.init();node=None
    try:
        node=observer_type()(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(URDF)),Parameter('yaw_heading_frame',value='grasp_heading'),
            Parameter('orientation_envelope_rad',value=.01),Parameter('joint_margin_rad',value=.01),
            Parameter('allow_loaded_reference_latch',value=True)])
        node.get_clock().set_ros_time_override(Time(seconds=10.))
        joints=JointState(name=list(node.geometry.model.names),position=HOME.tolist(),velocity=[0.]*5)
        odom=Odometry();wrench=WrenchStamped()
        for m in (joints,odom,wrench):m.header.stamp=Time(seconds=10.).to_msg()
        for key,m in [('joint_states',joints),('mecanum_drive_controller/odometry',odom),
            ('wrench_at_grasp_raw',wrench),('grasp/state',String(data='attached'))]:node.receive(key,m)
        debug=dict(sim_time=10.,enabled=False,state='READY',base_twist=[0.,0.,0.])
        if fault=='active':debug['enabled']=True
        if fault=='moving':debug['base_twist'][2]=.001
        if fault=='not_ready':debug['state']='TRANSPORT'
        node.receive('rotation_follow/debug',String(data=json.dumps(debug)))
        if fault=='stale':node.receipt['rotation_follow/debug']-=.2
        node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(
            node_name='other' if fault=='wrong_writer' else 'follower_yaw_whole_body',node_namespace='/follower')]
        node.armed=True
        response=node.latch(Trigger.Request(),Trigger.Response())
        assert response.success is (fault is None)
        if response.success:
            assert node.observation()['reference_sequence']==1
            debug['enabled']=True;node.receive('rotation_follow/debug',String(data=json.dumps(debug)))
            reference=node.reference.copy()
            assert not node.latch(Trigger.Request(),Trigger.Response()).success
            assert node.reference==reference
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()


def test_yaw_freshness_uses_current_own_ros_clock_between_endpoint_ticks():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    m=g.measure(HOME);msg=JointState(name=list(g.model.names),position=HOME.tolist())
    msg.header.stamp=Time(seconds=10.032).to_msg()
    o=dict(m,frame_id='follower/base_link',urdf_sha256='unit',yaw_valid=True,joint_stamp=10.032,
        e_yaw_rad=0.,psi_home_B=m['psi_ee_B'],p_home_footprint=m['p_ee_footprint'])
    now=[10.04]
    node=SimpleNamespace(observation=o,observation_wall=time.monotonic(),p=lambda k:.15,sim=10.,
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=now[0])),urdf_hash='unit',joints=msg,
        model=g.model,yaw_geometry=g,core=None,observation_reference=None)
    assert RotationFollower.require_yaw(node)['yaw_valid']
    now[0]=10.083
    with pytest.raises(ValueError,match='timestamp watchdog'):RotationFollower.require_yaw(node)
