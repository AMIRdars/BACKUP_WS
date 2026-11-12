"""Independent 8-DOF feasibility, retimed derivatives and fail-closed ROS flow."""
import numpy as np
import pytest
import time
from types import SimpleNamespace
from geometry_msgs.msg import WrenchStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import ReliabilityPolicy
from std_srvs.srv import Trigger

from amir_compliance_control.grasp_priority_leader import GraspProgress, GraspPriorityLeader
from amir_compliance_control.object_center_rotation_trajectory import ObjectCenterRotation
from amir_compliance_control.whole_body import WholeBodyAllocator, whole_jacobian
from amir_compliance_control.wrench import LocalModel, rotation
from amir_compliance_control.yaw_whole_body import YawWholeBody
from test_local_yaw import LocalYawRig, URDF, HOME


@pytest.mark.parametrize('axis', [0, 1, 3, 4, 5])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_whole_body_real_model_recovers_independent_planar_tasks(axis, sign):
    model = LocalModel(URDF.read_text())
    allocator = WholeBodyAllocator(model, base_speed=[.002]*3, task_metric=[1., 1., 1., .1, .1, .4])
    desired = np.zeros(6); desired[axis] = sign*.0001
    for _ in range(100):
        velocity = allocator.step(HOME, HOME, desired, np.zeros(3), 1., .01)
    assert np.linalg.matrix_rank(whole_jacobian(model, HOME)) == 6
    np.testing.assert_allclose(whole_jacobian(model, HOME)@velocity, desired, atol=1e-8)
    assert np.all(abs(velocity) <= allocator.scale+1e-14)


def test_progress_warning_is_latched_monotone_bounded_and_finishes():
    progress = GraspProgress(.5, .5)
    previous = 1.
    for i in range(1000):
        progress.step(.01, 10 <= i < 12)
        assert .5 <= progress.rate <= previous
        assert previous-progress.rate <= .005+1e-14
        previous = progress.rate
    assert progress.warned and progress.rate == .5 and progress.elapsed > 5.
    with pytest.raises(ValueError): progress.step(.051, False)


def test_retiming_scales_tangent_velocity_and_both_acceleration_terms():
    obj, base, grasp = np.eye(4), np.eye(4), np.eye(4)
    base[0, 3], grasp[0, 3] = -1., -.6
    trajectory = ObjectCenterRotation(obj, base, grasp, duration=20., delta_yaw_rad=.1)
    p = GraspProgress(.5, .5); p.elapsed = 8.; p.rate = .5
    rate_dot, h = -.2, 1e-4
    ref = p.reference(trajectory, rate_dot)
    q0 = trajectory.sample(p.elapsed)
    plus = trajectory.sample(p.elapsed+p.rate*h+.5*rate_dot*h*h)
    minus = trajectory.sample(p.elapsed-p.rate*h+.5*rate_dot*h*h)
    np.testing.assert_allclose((plus.base_pose[:3, 3]-minus.base_pose[:3, 3])/(2*h), ref.base_velocity_world, atol=1e-10)
    np.testing.assert_allclose((plus.base_pose[:3, 3]-2*q0.base_pose[:3, 3]+minus.base_pose[:3, 3])/h**2, ref.base_acceleration_world, atol=5e-8)
    np.testing.assert_array_equal(ref.object_pose[:3, 3], [0.]*3)


@pytest.mark.parametrize('forces,risk', [([20.,20.],0.),([24.,20.],2./3.),([10.,20.],5./6.)])
def test_native_reserve_warns_before_existing_holding_limits(forces, risk):
    node = SimpleNamespace(sim=10., native_wall=time.monotonic(), native_grasp={
        'state':'HOLDING','healthy':True,'sim_time':10.,'own_contacts':{'left':True,'right':True},
        'own_forces_N':dict(zip(('left','right'),forces))})
    assert YawWholeBody.native_risk(node) == pytest.approx(risk)


@pytest.mark.parametrize('fault', ['stale','contact','underload','overload','nonfinite'])
def test_native_reserve_invalid_data_blocks_continuing_transport(fault):
    node = SimpleNamespace(sim=10., native_wall=time.monotonic(), native_grasp={
        'state':'HOLDING','healthy':True,'sim_time':10.,'own_contacts':{'left':True,'right':True},
        'own_forces_N':{'left':20.,'right':20.}})
    if fault=='stale':node.native_wall-=.2
    if fault=='contact':node.native_grasp['own_contacts']['left']=False
    if fault=='underload':node.native_grasp['own_forces_N']['left']=7.9
    if fault=='overload':node.native_grasp['own_forces_N']['left']=26.1
    if fault=='nonfinite':node.native_grasp['own_forces_N']['left']=float('nan')
    with pytest.raises(ValueError):YawWholeBody.native_risk(node)


@pytest.mark.parametrize('enabled,age',[(False,0.),(True,.2)])
def test_disabled_or_stale_follower_blocks_new_leader_goal(enabled,age):
    node=SimpleNamespace(follower_enabled=enabled,follower_wall=time.monotonic()-age,
                         rejected=0,p=lambda key:.15)
    GraspPriorityLeader.rotation_goal(node,None)
    assert node.rejected==1 and 'enabled follower' in node.last_rejection


def test_native_planning_projection_preserves_geometric_center_and_bounds_tilt():
    node=SimpleNamespace(p=lambda key:True,policy={'rotation_arm_angle_limit_rad':.01})
    obj=np.eye(4);obj[:3,3]=[1.,2.,3.];obj[:3,:3]=rotation([0.,0.,1.],.3)@rotation([1.,0.,0.],.0001)
    original=obj.copy()
    result=GraspPriorityLeader.planning_object_pose(node,obj)
    np.testing.assert_array_equal(obj,original)
    np.testing.assert_array_equal(result[:3,3],obj[:3,3])
    np.testing.assert_allclose(result[:3,:3],rotation([0.,0.,1.],.3),atol=1e-15)
    obj[:3,:3]=rotation([1.,0.,0.],.02)
    with pytest.raises(ValueError):GraspPriorityLeader.planning_object_pose(node,obj)


@pytest.mark.parametrize('axis',[0,1])
@pytest.mark.parametrize('sign',[-1.,1.])
def test_rotational_relief_reduces_independent_contact_spring_moment(axis,sign):
    from amir_compliance_control.admittance import PlanarAdmittance
    relief=PlanarAdmittance([.2]*2,[12.]*2,[4.]*2,[.001]*2,[.002]*2,[.02]*2)
    target=np.zeros(2);target[axis]=sign*.0003
    for _ in range(1000):
        moment=100.*(target-relief.x)
        relief.step(moment,.001)
    assert np.linalg.norm(moment)<.005
    assert sign*relief.x[axis]>0 and abs(relief.x[axis])<.001


@pytest.mark.parametrize('fault', ['raw_moment', 'raw_dropout', 'grasp_loss'])
@pytest.mark.parametrize('relief_enabled',[False,True])
def test_whole_body_has_one_output_and_preserves_inherited_stop_latch(tmp_path, fault, relief_enabled):
    rig = LocalYawRig(tmp_path)
    try:
        old = rig.ends['follower']; rig.executor.remove_node(old); rig.nodes.remove(old); old.destroy_node()
        node = YawWholeBody(namespace='/follower', parameter_overrides=[Parameter('use_sim_time', value=True),
            Parameter('urdf_file', value=str(rig.file)), Parameter('wrench_force_limit_N', value=.5),
            Parameter('wrench_torque_limit_Nm', value=.1),Parameter('rotational_relief_enabled',value=relief_enabled)])
        rig.ends['follower'] = node; rig.nodes.append(node); rig.executor.add_node(node)
        rig.start(); assert node.enable(Trigger.Request(), Trigger.Response()).success
        rig.pump(.1)
        assert node.role == 'WHOLE_BODY_XY_YAW' and node.arm_task.previous.shape == (8,)
        if relief_enabled:
            probe=WrenchStamped();probe.header.stamp=Time(seconds=rig.sim+.001).to_msg()
            probe.header.frame_id='follower/grasp_aligned';probe.wrench.torque.x=.001
            node.filtered(probe)
            assert node.relief.x[0]>0.
        assert sum(p.topic_name == '/follower/base_cmd' for p in node.publishers) == 1
        assert sum(p.topic_name == '/follower/arm_joint_cmd' for p in node.publishers) == 1
        for topic in ('wrench_at_grasp_raw','wrench_filtered'):
            streams=[s for s in node.subscriptions if s.topic_name=='/follower/'+topic]
            assert len(streams)==1 and streams[0].qos_profile.reliability==ReliabilityPolicy.RELIABLE
        assert not any(s.topic_name.startswith(('/leader/', '/world/', '/experiment/')) or 'goal' in s.topic_name or '/tf' in s.topic_name for s in node.subscriptions)
        if fault == 'raw_moment':
            msg = WrenchStamped(); msg.header.stamp = Time(seconds=rig.sim+.001).to_msg()
            msg.header.frame_id = 'follower/grasp_aligned'; msg.wrench.torque.z = .101; rig.raw_pub.publish(msg)
        elif fault == 'raw_dropout': rig.raw_dropout = True
        else: node.grasp = False; node.update_local()
        rig.pump(.3)
        if fault=='raw_moment':
            assert node.wrench_violation is not None and node.wrench_violation['torque_norm_Nm']>.1
        assert node.latched and node.arm_task is None and node.base is None and node.relief is None
        assert rig.ends['leader'].latched and rig.stop_calls > 0
        assert rig.output['leader'] == rig.output['follower'] == 0.
        rig.raw_dropout = False; rig.pump(.1)
        assert node.latched and node.core is None
    finally:
        rig.close()
