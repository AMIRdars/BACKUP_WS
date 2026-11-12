"""Independent chapter 23 geometry, ablation, coupling and shared-stop tests."""
import hashlib
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
import time
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rclpy.time import Time
from geometry_msgs.msg import WrenchStamped

from amir_compliance_control.rotation_observer import ProjectedYaw,read_qualification,wrap_pi,qualify_passive,observer_type,yaw_noise_std
from amir_compliance_control.rotation_recentering import base_reference,RotationRecentering,whole_body_task
from amir_compliance_control.rotation_follow import RotationFollower
from amir_compliance_control.whole_body import WholeBodyAllocator,whole_jacobian
from amir_compliance_control.wrench import rotation
from test_local_yaw import LocalYawRig,URDF,HOME


def geometry():
    return ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01)


def test_actual_home_projected_yaw_is_not_a_robust_measurement():
    g=geometry();m=g.measure(HOME)
    assert m['projection_norm']<1e-6 and not m['geometry_valid']
    q1,q2=HOME.copy(),HOME.copy();q1[4]-=1e-6;q2[4]+=1e-6
    difference=abs(wrap_pi(g.measure(q1)['psi_ee_B']-g.measure(q2)['psi_ee_B']))
    assert difference>3.14


@pytest.mark.parametrize('axis',range(5))
def test_projected_yaw_derivative_matches_independent_finite_difference(axis):
    g=geometry();q=HOME.copy();q[4]=.3
    m=g.measure(q);assert m['geometry_valid']
    plus,minus=q.copy(),q.copy();plus[axis]+=1e-7;minus[axis]-=1e-7
    derivative=wrap_pi(g.measure(plus)['psi_ee_B']-g.measure(minus)['psi_ee_B'])/2e-7
    assert m['yaw_jacobian'][axis]==pytest.approx(derivative,abs=2e-8)


def test_base_link_to_existing_control_frame_is_from_the_urdf():
    g=geometry();m=g.measure(HOME)
    point=g.footprint_T_base@np.r_[m['p_ee_B'],1.]
    np.testing.assert_allclose(point[:3],m['p_ee_footprint'],atol=1e-15)
    assert abs(g.footprint_T_base[0,1])>.999


def test_explicit_grasp_heading_preserves_position_and_removes_home_chart_singularity():
    old=geometry()
    new=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,
                     heading_frame='grasp_heading')
    measured=new.measure(HOME)
    assert measured['geometry_valid'] and measured['projection_norm']>.999999
    np.testing.assert_allclose(measured['p_ee_B'],old.measure(HOME)['p_ee_B'],atol=0.)
    np.testing.assert_allclose(new.tcp_T_heading[:3,:3].T@new.tcp_T_heading[:3,:3],np.eye(3))
    assert np.linalg.det(new.tcp_T_heading[:3,:3])==pytest.approx(1.)
    for joint in range(5):
        a,b=HOME.copy(),HOME.copy();a[joint]+=1e-7;b[joint]-=1e-7
        derivative=wrap_pi(new.measure(a)['psi_ee_B']-new.measure(b)['psi_ee_B'])/2e-7
        assert measured['yaw_jacobian'][joint]==pytest.approx(derivative,abs=2e-8)


def test_heading_frame_and_pose_policy_are_bound_to_passive_evidence(tmp_path):
    old=geometry()
    cert=qualify_passive(passive_fixture(),old,force_limit_N=.5,torque_limit_Nm=.05,
                         evidence_sha256={'synthetic':'unit'})
    cert.update(robot='follower',urdf_sha256='bound')
    file=tmp_path/'certificate.json';file.write_text(json.dumps(cert))
    assert read_qualification(file,'bound',old)
    new=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    with pytest.raises(ValueError):read_qualification(file,'bound',new)
    changed=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.02,joint_margin_rad=.01)
    with pytest.raises(ValueError,match='frame/envelope'):read_qualification(file,'bound',changed)


def test_approach_frame_is_never_guessed_for_a_different_tcp_transform():
    root=ET.fromstring(URDF.read_text())
    root.find("joint[@name='tcp_fixed']/origin").set('xyz','.01 .0795 0')
    with pytest.raises(ValueError,match='positive-Y'):
        ProjectedYaw(ET.tostring(root,encoding='unicode'),orientation_envelope_rad=.01,
                     joint_margin_rad=.01,heading_frame='grasp_heading')


def test_real_observer_latches_only_disarmed_and_wall_watchdog_survives_paused_clock(tmp_path):
    from nav_msgs.msg import Odometry
    from rclpy.clock import ClockType
    rclpy.init()
    node=None
    try:
        node=observer_type()(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(URDF)),Parameter('yaw_heading_frame',value='grasp_heading'),
            Parameter('orientation_envelope_rad',value=.01),Parameter('joint_margin_rad',value=.01)])
        assert {timer.clock.clock_type for timer in node.timers}=={ClockType.ROS_TIME,ClockType.STEADY_TIME}
        node.get_clock().set_ros_time_override(Time(seconds=10.))
        joints=JointState(name=list(node.geometry.model.names),position=HOME.tolist(),velocity=[0.]*5)
        odom=Odometry();wrench=WrenchStamped()
        for msg in (joints,odom,wrench):msg.header.stamp=Time(seconds=10.).to_msg()
        for key,msg in [('joint_states',joints),('mecanum_drive_controller/odometry',odom),
                        ('wrench_at_grasp_raw',wrench),('grasp/state',String(data='attached'))]:node.receive(key,msg)
        assert node.latch(Trigger.Request(),Trigger.Response()).success
        out=node.observation();assert out['geometry_valid'] and not out['yaw_valid']
        assert 'qualification missing' in out['valid_reason']
        node.armed=True
        response=node.latch(Trigger.Request(),Trigger.Response())
        assert not response.success and 'disarm' in response.message
        published=[];node.pub=SimpleNamespace(publish=lambda msg:published.append(json.loads(msg.data)))
        for key in node.receipt:node.receipt[key]-=.2
        node.watchdog()
        assert published and not published[-1]['yaw_valid'] and 'watchdog' in published[-1]['valid_reason']
        assert node.get_clock().now().nanoseconds==10_000_000_000
        for key in node.receipt:node.receipt[key]=time.monotonic()
        wrench.wrench.force.x=float('nan');node.tick()
        assert not published[-1]['yaw_valid'] and 'nonfinite' in published[-1]['valid_reason']
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()


def test_noise_monitor_distinguishes_measured_slow_motion_from_unexplained_noise():
    times=np.arange(32)*.02;rate=1e-5;noise=np.sin(np.arange(32))*1e-9
    # Cross pi to exercise unwrapping while the measured joint velocity
    # predicts a real slow yaw. No reference/command enters this estimate.
    angles=np.array([wrap_pi(math.pi-2e-6+rate*t+n) for t,n in zip(times,noise)])
    window=np.column_stack((times,angles,np.full(32,rate)))
    assert yaw_noise_std(window)==pytest.approx(float(np.std(noise,ddof=1)),abs=1e-15)
    window[:,1]+=np.sin(np.arange(32))*1e-6
    assert yaw_noise_std(window)>1e-7


@pytest.mark.parametrize('mode',['XY_YAW','XY_YAW_COMP'])
def test_invalid_yaw_refuses_both_yaw_modes(mode):
    with pytest.raises(ValueError,match='observable'):
        base_reference([.4,.2,0.],[.4,.2,0.],.1,0.,mode=mode,yaw_valid=False,
            gains=[.7,.7,.4],velocity=[.03,.03,.05])


@pytest.mark.parametrize('sign',[-1.,1.])
def test_document_example_and_rigid_point_velocity_sign(sign):
    p=np.array([.4,.2,0.])
    u,c,e=base_reference(p,p,sign*.1,0.,mode='XY_YAW_COMP',yaw_valid=True,
                         gains=[.7,.7,.4],velocity=[.03,.03,.05])
    np.testing.assert_allclose(u,sign*np.array([.008,-.016,.04]),atol=1e-15)
    np.testing.assert_allclose(np.r_[u[:2],0.]+np.cross([0.,0.,u[2]],p),0.,atol=1e-15)
    assert np.linalg.norm(e[:2])==0. and np.linalg.norm(c)>0.


@pytest.mark.parametrize('mode',['XY_ONLY','XY_YAW','XY_YAW_COMP','WHOLE_BODY'])
def test_limiter_compensates_only_c_using_post_acceleration_yaw(mode):
    c=RotationRecentering(mode=mode,gains=[0.,0.,10.],velocity=[.01,.01,.02],
                          acceleration=[1.,1.,.01],cutoff=100.)
    p=np.array([.4,.2,.6]);previous=np.zeros(3)
    for _ in range(100):
        u=c.step([0.,0.,1.],p,.01,yaw_valid=True)
        assert np.all(abs(u-previous)<=c.amax*.01+1e-14)
        assert np.all(abs(u)<=c.vmax+1e-14)
        expected=u[2]*np.array([p[1],-p[0]]) if mode=='XY_YAW_COMP' else np.zeros(2)
        np.testing.assert_allclose(c.compensation,expected)
        previous=u
    if mode=='XY_ONLY':assert u[2]==0.


def test_whole_body_static_grasp_task_does_not_double_count_base_rotation():
    g=geometry();q=HOME.copy();q[4]=.3
    alloc=WholeBodyAllocator(g.model,base_speed=[.002]*3,base_acceleration=[1.]*3,
                             arm_acceleration=1.,base_regularization=.00003)
    preferred=np.array([0.,0.,.001])
    task=whole_body_task(np.zeros(6),np.zeros(6),np.zeros(6),2.)
    for _ in range(30):u=alloc.step(q,q,task,preferred,1.,.01)
    assert u[7]>0.
    np.testing.assert_allclose(whole_jacobian(g.model,q)@u,0.,atol=1e-8)
    h=1e-4;p0=g.model.frames(q)[0];p1=g.model.frames(q+h*u[:5])[0]
    world_next=np.r_[h*u[5:7],0.]+rotation([0.,0.,1.],h*u[7])@p1
    np.testing.assert_allclose(world_next,p0,atol=1e-10)


def test_qualification_is_never_inferred_from_a_yaw_command(tmp_path):
    assert read_qualification('', 'unused') is None
    p=tmp_path/'certificate.json'
    p.write_text(json.dumps(dict(robot='follower',urdf_sha256='a',measurement='projected_tcp_x_axis_base_link',
        passive_positive_negative_passed=False,evidence_sha256={'raw':'b'})))
    with pytest.raises(ValueError,match='passive'):read_qualification(p,'a')


def compatible_rig_model_and_observer(rig):
    root=ET.fromstring(rig.file.read_text())
    root.find("joint[@name='tcp']").set('name','tcp_fixed')
    j=ET.SubElement(root,'joint',name='body_frame',type='fixed')
    ET.SubElement(j,'parent',link='base_footprint');ET.SubElement(j,'child',link='base_link')
    rig.file.write_text(ET.tostring(root,encoding='unicode'))
    observer=rclpy.create_node('rotation_observer',namespace='/follower')
    publisher=observer.create_publisher(String,'rotation/observation',10)
    rig.nodes.append(observer);rig.executor.add_node(observer)
    return observer,publisher


def test_actual_home_rejects_even_a_claimed_positive_observer_gate():
    msg=JointState(name=list(geometry().model.names),position=HOME.tolist())
    msg.header.stamp=Time(seconds=10.).to_msg()
    node=SimpleNamespace(observation_wall=time.monotonic(),p=lambda key:.15,sim=10.,urdf_hash='bound',
        observation=dict(frame_id='follower/base_link',urdf_sha256='bound',yaw_valid=True,joint_stamp=10.),
        joints=msg,model=geometry().model,yaw_geometry=geometry())
    with pytest.raises(ValueError,match='geometry'):RotationFollower.require_yaw(node)


def test_controller_rejects_a_valid_flag_for_a_different_heading_chart():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    msg=JointState(name=list(g.model.names),position=HOME.tolist())
    msg.header.stamp=Time(seconds=10.).to_msg()
    node=SimpleNamespace(observation_wall=time.monotonic(),p=lambda key:.15,sim=10.,urdf_hash='bound',
        observation=dict(frame_id='follower/base_link',urdf_sha256='bound',yaw_valid=True,joint_stamp=10.,
                         yaw_heading_frame='tcp_link'),joints=msg,model=g.model,yaw_geometry=g)
    with pytest.raises(ValueError,match='frame mismatch'):RotationFollower.require_yaw(node)


def test_horizontal_grasp_heading_uses_measured_joint_yaw_without_command_input():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    reference=g.measure(HOME)['psi_ee_B']
    for angle in (-.05,.05):
        q=HOME.copy();q[0]+=angle
        m=g.measure(q)
        assert m['geometry_valid']
        assert wrap_pi(m['psi_ee_B']-reference)==pytest.approx(angle,abs=1e-12)
    # Wrist roll about the approach direction does not masquerade as yaw.
    q=HOME.copy();q[4]+=.1
    assert wrap_pi(g.measure(q)['psi_ee_B']-reference)==pytest.approx(0.,abs=1e-12)


def test_passive_arm_torque_compliance_keeps_base_zero_and_cannot_authorize_rotation(tmp_path,monkeypatch):
    def measured_torque(rig):
        rig.status_pub.publish(String(data='READY'))
        if rig.sim==rig.last_sent:return
        msg=WrenchStamped();msg.header.stamp=Time(seconds=rig.sim).to_msg()
        msg.header.frame_id='follower/grasp_aligned';msg.wrench.torque.z=.001
        rig.raw_pub.publish(msg);rig.filtered_pub.publish(msg);rig.last_sent=rig.sim
    monkeypatch.setattr(LocalYawRig,'send_wrench',measured_torque)
    rig=LocalYawRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        compatible_rig_model_and_observer(rig)
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('rotation_follow_mode',value='XY_ONLY'),
            Parameter('base_gains',value=[0.,0.,0.]),Parameter('passive_yaw_calibration',value=True),
            Parameter('arm_yaw_admittance_enabled',value=True),Parameter('allow_ungrasped_test',value=True),
            Parameter('wrench_force_limit_N',value=.5),Parameter('wrench_torque_limit_Nm',value=.1)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node);rig.start()
        assert node.enable(Trigger.Request(),Trigger.Response()).success
        # Even a claimed successful observer cannot authorize the production
        # leader while this explicitly stationary calibration is enabled.
        node.require_yaw=lambda:{}
        ready=[];node.ready_pub=SimpleNamespace(publish=lambda m:ready.append(m.data))
        rig.pump(.25)
        assert node.arm_task.last['internal_yaw_target_rad']>0.
        assert node.base_twist.tolist()==[0.,0.,0.] and ready and not any(ready)
    finally:rig.close()


@pytest.mark.parametrize('mode',['XY_ONLY','XY_YAW','XY_YAW_COMP','WHOLE_BODY'])
def test_unqualified_yaw_refuses_enable_without_command_or_peer_dependency(tmp_path,mode):
    rig=LocalYawRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        compatible_rig_model_and_observer(rig)
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('rotation_follow_mode',value=mode),
            Parameter('rotation_observation_required',value=True),
            Parameter('wrench_force_limit_N',value=.5),Parameter('wrench_torque_limit_Nm',value=.1)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node)
        rig.start()
        node.observation_wall=time.monotonic()
        node.observation=dict(frame_id='follower/base_link',urdf_sha256=node.urdf_hash,
                              yaw_valid=False,joint_stamp=rig.sim,valid_reason='passive qualification missing')
        response=node.enable(Trigger.Request(),Trigger.Response())
        assert not response.success and node.core is None
        assert node.base_twist.tolist()==[0.,0.,0.]
        assert sum(p.topic_name=='/follower/base_cmd' for p in node.publishers)==1
        assert not any(s.topic_name.startswith(('/leader/','/world/','/experiment/')) or '/tf' in s.topic_name for s in node.subscriptions)
    finally:rig.close()


@pytest.mark.parametrize('mode',['XY_ONLY','XY_YAW','XY_YAW_COMP'])
def test_online_loss_uses_existing_both_robot_stop_and_hold_path(tmp_path,mode):
    rig=LocalYawRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        compatible_rig_model_and_observer(rig)
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('rotation_follow_mode',value=mode),
            Parameter('rotation_observation_required',value=True),
            Parameter('wrench_force_limit_N',value=.5),Parameter('wrench_torque_limit_Nm',value=.1)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node);rig.start()
        # A local control interval has started; then the observer expires.
        # The real home cannot qualify, so no fake valid geometry is injected.
        node.core=SimpleNamespace()
        node.observation_wall=0.
        node.update_local();rig.pump(.3)
        assert node.latched and node.core is None and rig.ends['leader'].latched
        assert rig.stop_calls>0 and rig.output['leader']==rig.output['follower']==0.
        rig.pump(.1);assert node.latched
    finally:rig.close()


@pytest.mark.parametrize('payload',[{'yaw_valid':False,'valid_reason':'transient invalid'},None])
def test_received_invalid_snapshot_stops_both_before_next_control_tick(tmp_path,payload):
    rig=LocalYawRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        compatible_rig_model_and_observer(rig)
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('rotation_follow_mode',value='XY_YAW_COMP'),
            Parameter('rotation_observation_required',value=True),
            Parameter('wrench_force_limit_N',value=.5),Parameter('wrench_torque_limit_Nm',value=.1)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node);rig.start()
        sub=next(s for s in node.subscriptions if s.topic_name=='/follower/rotation/observation')
        assert sub.qos_profile.depth==1
        node.core=SimpleNamespace()
        node.observe(String(data=json.dumps(payload)))
        assert node.latched and node.core is None
        node.observe(String(data=json.dumps({'yaw_valid':True})))
        assert node.latched and node.core is None
        rig.pump(.3)
        assert rig.ends['leader'].latched and rig.stop_calls>0
        assert rig.output['leader']==rig.output['follower']==0.
    finally:rig.close()


@pytest.mark.parametrize('mode',['XY_ONLY','XY_YAW','XY_YAW_COMP','WHOLE_BODY'])
def test_synthetic_qualified_ros_fixture_runs_one_selected_allocator(tmp_path,mode):
    rig=LocalYawRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        observer,publisher=compatible_rig_model_and_observer(rig)
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('rotation_follow_mode',value=mode),
            Parameter('rotation_observation_required',value=True),
            Parameter('wrench_force_limit_N',value=.5),Parameter('wrench_torque_limit_Nm',value=.1)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node)
        measured=node.yaw_geometry.measure(np.zeros(5));assert measured['geometry_valid']
        def emit():
            msg=dict(measured,frame_id='follower/base_link',urdf_sha256=node.urdf_hash,yaw_valid=True,
                joint_stamp=rig.sim,p_home_footprint=measured['p_ee_footprint'],psi_home_B=measured['psi_ee_B'],
                e_yaw_rad=0.,valid_reason='synthetic qualified unit fixture')
            publisher.publish(String(data=json.dumps(msg)))
        observer.create_timer(.005,emit)
        rig.force=[0.,0.];rig.start()
        response=node.enable(Trigger.Request(),Trigger.Response());assert response.success,response.message
        rig.force=[.001,0.];rig.pump(.15)
        assert not node.latched and node.core is not None
        assert node.arm_task.previous.shape==((8,) if mode=='WHOLE_BODY' else (5,))
        assert len([p for p in node.publishers if p.topic_name=='/follower/base_cmd'])==1
        assert node.arm_task.last['internal_yaw_target_rad']==0.
        if mode=='WHOLE_BODY':assert node.arm_task.last['explicit_compensation_xy']==[0.,0.]
        node.observation['p_home_footprint']=list(node.observation['p_home_footprint'])
        node.observation['p_home_footprint'][0]+=.001
        node.update_local();rig.pump(.1)
        assert node.latched and rig.ends['leader'].latched
    finally:rig.close()


def passive_fixture():
    rows=[];q=HOME.copy();q[4]=.3
    for i in range(32):
        pos=q.copy();pos[0]=1e-6*math.sin(i)
        rows.append(dict(phase='static',timestamp=1+i*.01,q_rad=pos.tolist(),qdot_rad_s=[0.]*5,
                         wrench_tcp=[0.,0.,2.45,0.,0.,0.],base_twist=[0.]*3,grasp_attached=True))
    for phase,start,delta,duration,count in [('positive',0.,.01,1.,100),('negative',.01,-.02,2.,200)]:
        for i in range(1,count+1):
            u=i/count;pos=q.copy();pos[0]=start+delta*(10*u**3-15*u**4+6*u**5)
            velocity=[delta/duration*30*u*u*(1-u)**2,0.,0.,0.,0.]
            rows.append(dict(phase=phase,timestamp=rows[-1]['timestamp']+.01,q_rad=pos.tolist(),qdot_rad_s=velocity,
                wrench_tcp=[0.,0.,2.45,0.,0.,0.],base_twist=[0.]*3,grasp_attached=True))
    return rows


def test_passive_certificate_requires_two_measured_signs_and_binds_region(tmp_path):
    cert=qualify_passive(passive_fixture(),geometry(),force_limit_N=.5,torque_limit_Nm=.05,evidence_sha256={'synthetic':'unit'})
    assert cert['minimum_signal_to_noise']>6
    assert cert['qualified_q_lower_rad'][0]<-.009 and cert['qualified_q_upper_rad'][0]>.009
    cert.update(robot='follower',urdf_sha256='bound')
    file=tmp_path/'qualified.json';file.write_text(json.dumps(cert))
    assert read_qualification(file,'bound')['passive_positive_negative_passed']


@pytest.mark.parametrize('fault',['flat','wrong_sign','force','base_moves','contact','velocity','time'])
def test_bad_passive_data_cannot_issue_a_qualification(fault):
    rows=passive_fixture()
    for r in rows:
        if r['phase']=='negative' and fault in ('flat','wrong_sign'):
            r['q_rad'][0]=0. if fault=='flat' else abs(r['q_rad'][0])
        if r['phase']=='positive' and fault=='force':r['wrench_tcp'][0]=.6
        if fault=='velocity' and r['phase']!='static':r['qdot_rad_s']=[0.]*5
    if fault=='base_moves':rows[-1]['base_twist'][2]=.001
    if fault=='contact':rows[-1]['grasp_attached']=False
    if fault=='time':rows[-1]['timestamp']+=1.
    with pytest.raises(ValueError):qualify_passive(rows,geometry(),force_limit_N=.5,torque_limit_Nm=.05,evidence_sha256={'synthetic':'unit'})


def test_integrated_launch_has_one_follower_writer_and_no_independent_recentering():
    folder=Path(__file__).resolve().parents[1]/'launch'
    tree=ast.parse((folder/'rotation_follow_integrated.launch.py').read_text())
    executables=[k.value.value for n in ast.walk(tree) if isinstance(n,ast.Call)
                 and isinstance(n.func,ast.Name) and n.func.id=='Node'
                 for k in n.keywords if k.arg=='executable' and isinstance(k.value,ast.Constant)]
    assert sorted(executables)==['follower_rotation_integrated','rotation_observer','wrench_preprocessor']


def test_previous_rotation_launches_route_through_the_gate():
    folder=Path(__file__).resolve().parents[1]/'launch'
    for name in ('yaw_arm_base.launch.py','yaw_whole_body.launch.py'):
        source=(folder/name).read_text()
        assert 'rotation_follow_integrated.launch.py' in source
        assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Node' for n in ast.walk(ast.parse(source)))
    tree=ast.parse((folder/'object_rotation_leader.launch.py').read_text())
    executables=[k.value.value for n in ast.walk(tree) if isinstance(n,ast.Call)
                 for k in n.keywords if k.arg=='executable' and isinstance(k.value,ast.Constant)]
    assert executables==['leader_grasp_priority']


@pytest.mark.parametrize('mode',['XY_ONLY','XY_YAW','XY_YAW_COMP','WHOLE_BODY'])
def test_arm_torque_target_remains_distinct_from_measured_yaw(tmp_path,monkeypatch,mode):
    def measured_torque(rig):
        rig.status_pub.publish(String(data='READY'))
        if rig.sim==rig.last_sent:return
        msg=WrenchStamped();msg.header.stamp=Time(seconds=rig.sim).to_msg()
        msg.header.frame_id='follower/grasp_aligned';msg.wrench.torque.z=.001
        rig.raw_pub.publish(msg);rig.filtered_pub.publish(msg);rig.last_sent=rig.sim
    monkeypatch.setattr(LocalYawRig,'send_wrench',measured_torque)
    rig=LocalYawRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        observer,publisher=compatible_rig_model_and_observer(rig)
        allocation=([Parameter('allocation_task_compliance_budget',value=True),
            Parameter('allocation_base_regularization',value=.00003)] if mode=='WHOLE_BODY' else [])
        node=RotationFollower(namespace='/follower',parameter_overrides=[*allocation,Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('rotation_follow_mode',value=mode),
            Parameter('rotation_observation_required',value=True),Parameter('arm_yaw_admittance_enabled',value=True),
            Parameter('base_gains',value=[.5,.5,2.]),Parameter('allow_ungrasped_test',value=True),
            Parameter('wrench_force_limit_N',value=.5),Parameter('wrench_torque_limit_Nm',value=.1)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node)
        measured=node.yaw_geometry.measure(np.zeros(5))
        def emit():
            publisher.publish(String(data=json.dumps(dict(measured,frame_id='follower/base_link',
                urdf_sha256=node.urdf_hash,yaw_valid=True,joint_stamp=rig.sim,
                p_home_footprint=measured['p_ee_footprint'],psi_home_B=measured['psi_ee_B'],
                e_yaw_rad=0.,valid_reason='synthetic qualified unit fixture'))))
        observer.create_timer(.005,emit);rig.start()
        assert node.enable(Trigger.Request(),Trigger.Response()).success
        rig.pump(.25)
        assert not node.latched and node.arm_task.last['internal_yaw_target_rad']>0.
        assert node.arm_task.last['measured_projected_yaw_error_rad']==0.
        assert node.may_authorize_rotation()
        if mode=='WHOLE_BODY':
            assert node.arm_task.last['task_compliance_allocation_budget']>0.
            assert node.arm_task.last['measured_allocation_budget']==0.
            assert np.linalg.norm(node.base_twist)>0.
            node.trip('test stop')
            assert node.core is None and np.array_equal(node.base_twist,np.zeros(3))
            assert np.array_equal(node.task_compliance_budget.low,np.zeros(3))
        else:assert node.base_twist[2]==0.
    finally:rig.close()
