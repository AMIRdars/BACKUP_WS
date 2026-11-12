"""Independent model differences, QP oracle, hard bounds and ROS boundary."""
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from amir_compliance_control.wrench import LocalModel, rotation
from amir_compliance_control.whole_body import whole_jacobian, box_least_squares, WholeBodyAllocator
from amir_compliance_control.whole_body_node import WholeBodyFollower
from amir_compliance_control.planar_leader import PlanarLeader
from test_admittance import ArmRig
from test_recentering import JointOverride

ROOT = Path(__file__).resolve().parents[4]
URDF = ROOT/'audits/whole_body_compliance_20261019/cp6/generated/follower.urdf'
HOME = np.array([0., 1.4, -1.6, .2, 1.570796])


def actual(): return LocalModel(URDF.read_text())


@pytest.mark.parametrize('sensor_point', [False, True])
def test_actual_whole_jacobian_every_column_in_arbitrary_world_axes(sensor_point):
    m = actual(); rng = np.random.default_rng(81)
    for q in [HOME, *rng.uniform(m.lower+.1, m.upper-.1, (15, 5))]:
        angle = rng.uniform(-3., 3.); r = rotation([0, 0, 1], angle)
        def world(delta):
            tcp, sensor = m.frames(q+delta[:5]); p = sensor[:3, 3] if sensor_point else tcp
            # Base increments are BODY velocities, not world x/y increments.
            rb = r@rotation([0, 0, 1], delta[7])
            return rb@p+r@np.r_[delta[5:7], 0.], rb@sensor[:3, :3]
        numeric = np.zeros((6, 8)); h = 1e-6; _, r0 = world(np.zeros(8))
        for i in range(8):
            d = np.eye(8)[i]*h
            p1, r1 = world(d); p0, rm = world(-d)
            numeric[:3, i] = (p1-p0)/(2*h)
            skew = ((r1-rm)/(2*h))@r0.T
            numeric[3:, i] = [skew[2,1], skew[0,2], skew[1,0]]
        analytic = whole_jacobian(m, q, at_sensor=sensor_point)
        analytic[:3] = r@analytic[:3]; analytic[3:] = r@analytic[3:]
        np.testing.assert_allclose(numeric, analytic, atol=2e-9)
    assert np.linalg.matrix_rank(m.jacobian(HOME)) == 5
    assert np.linalg.matrix_rank(whole_jacobian(m, HOME)) == 6
    tcp, sensor = m.frames(HOME)
    assert abs(np.linalg.norm(tcp-sensor[:3,3])-.0795) < 1e-10


def oracle(a, b, lo, hi):
    candidates = []
    for status in product((-1, 0, 1), repeat=len(lo)):
        s = np.array(status); x = np.where(s < 0, lo, np.where(s > 0, hi, 0.))
        free = s == 0
        if free.any(): x[free] = np.linalg.lstsq(a[:,free], b-a[:,~free]@x[~free], rcond=None)[0]
        if np.all(x >= lo-1e-10) and np.all(x <= hi+1e-10): candidates.append((np.linalg.norm(a@x-b), x))
    return min(candidates, key=lambda v:v[0])[1]


def test_bounded_solver_matches_independent_exhaustive_oracle_and_beats_clipping():
    rng = np.random.default_rng(83); improved = 0
    for _ in range(60):
        a = rng.normal(size=(8,3)); b = rng.normal(size=8)
        lo, hi = rng.uniform(-.2,0,3), rng.uniform(0,.2,3)
        x = box_least_squares(a,b,lo,hi); expected = oracle(a,b,lo,hi)
        np.testing.assert_allclose(x,expected,atol=1e-9)
        clipped = np.clip(np.linalg.lstsq(a,b,rcond=None)[0],lo,hi)
        improved += np.linalg.norm(a@clipped-b) > np.linalg.norm(a@x-b)+1e-6
    assert improved > 10


def test_fixed_bounds_and_rank_deficient_regularization():
    np.testing.assert_allclose(box_least_squares(np.eye(3), [2.,3.,4.], [0.,-.1,0.], [0.,.1,1.]), [0.,.1,1.])
    with pytest.raises(ValueError): box_least_squares(np.zeros((3,3)), [0.,0.,0.], [-1.]*3, [1.]*3)


@pytest.mark.parametrize('axis', [0,1,5])
@pytest.mark.parametrize('sign', [-1.,1.])
def test_allocation_task_direction_with_speed_acceleration_and_joint_constraints(axis,sign):
    m = actual(); a = WholeBodyAllocator(m); q = HOME.copy(); previous = np.zeros(8)
    desired = np.zeros(6); desired[axis] = sign*.00005
    for _ in range(80):
        u = a.step(q,q,desired,[0.,0.,0.],1.,.01)
        assert np.all(abs(u) <= a.scale+1e-12)
        assert np.all(abs(u-previous) <= a.acceleration*.01+1e-12)
        q += u[:5]*.01
        assert np.all(q >= m.lower+.01) and np.all(q <= m.upper-.01)
        previous = u
    assert sign*(whole_jacobian(m,q)@u)[axis] > .00004
    assert np.linalg.norm((whole_jacobian(m,q)@u-desired)*[1,1,1,.1,.1,.1]) < 2e-6


@pytest.mark.parametrize('base_filter', [0., .2])
def test_arm_first_and_slow_base_authority_then_bounded_braking(base_filter):
    m=actual(); a=WholeBodyAllocator(m, base_filter_hz=base_filter); q=HOME.copy()
    u=a.step(q,q,[.00005,0,0,0,0,0],[0,0,0],0.,.01)
    assert not u[5:].any() and u[:5].any()
    for _ in range(20): u=a.step(q,q,[0,.0001,0,0,0,0],[0,.00005,0],1.,.01)
    assert u[6] > 0
    for _ in range(60):
        old=u.copy(); u=a.step(q,q,[0,0,0,0,0,0],[0,0,0],0.,.01)
        assert np.all(abs(u-old) <= a.acceleration*.01+1e-12)
    assert not u[5:].any()


def test_joint_margin_and_manipulability_change_weights_without_guessing_limits():
    m=actual(); a=WholeBodyAllocator(m); q=HOME.copy()
    a.step(q,q,np.zeros(6),np.zeros(3),1.,.01); normal=np.array(a.last['arm_weights'])
    q[0]=m.upper[0]-.011
    a.step(q,q,np.zeros(6),np.zeros(3),1.,.01)
    assert a.last['arm_weights'][0] > normal[0]*100
    bad=HOME.copy(); bad[0]=m.upper[0]-.009
    with pytest.raises(ValueError,match='margin'): a.step(bad,bad,np.zeros(6),np.zeros(3),1.,.01)
    q=HOME.copy(); q[1:4]=[.021,-.042,.021]
    b=WholeBodyAllocator(m);b.step(q,q,np.zeros(6),np.zeros(3),1.,.01)
    assert b.last['arm_min_singular'] < .1*a.last['arm_min_singular']
    assert b.last['arm_weights'][4] > normal[4]


def test_unavailable_whole_body_rank_is_rejected():
    root=ET.fromstring(URDF.read_text())
    for j in root.findall("joint[@type='revolute']"):
        j.find('axis').set('xyz','0 0 1')
    m=LocalModel(ET.tostring(root,encoding='unicode'));a=WholeBodyAllocator(m)
    with pytest.raises(ValueError,match='rank'): a.step(HOME,HOME,np.zeros(6),np.zeros(3),1.,.01)


@pytest.mark.parametrize('base_filter',[0.,.2])
def test_local_ros_force_does_not_directly_drive_base_and_sensor_loss_stops(tmp_path,base_filter):
    rig=ArmRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        node=WholeBodyFollower(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file)),Parameter('allow_ungrasped_test',value=True),Parameter('allocation_base_filter_hz',value=base_filter)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node)
        rig.override=JointOverride(rig.pubs['follower','joints']);rig.pubs['follower','joints']=rig.override
        rig.start();assert node.enable(Trigger.Request(),Trigger.Response()).success
        rig.pump(.15)
        assert node.core is not None and not node.base_twist.any(),node.reason
        q=np.zeros(5);q[0]=.0002;rig.override.positions=q;rig.pump(.3)
        assert node.base_twist.any(),node.reason
        subs={s.topic_name for s in node.subscriptions}
        assert not any(t.startswith('/leader/') or 'goal' in t or '/tf' in t or '/world/' in t for t in subs)
        rig.dropout=True;rig.pump(.3)
        assert node.latched and node.allocator is None and not node.base_twist.any()
        rig.dropout=False;rig.permit=True;rig.pump(.1)
        assert node.latched and node.core is None
    finally:rig.close()


@pytest.mark.parametrize('axis,sign',[(0,-1.),(1,1.),(2,1.),(2,-1.)])
def test_planar_leader_goal_units_axes_and_stop_clears_reference(tmp_path,axis,sign):
    rig=ArmRig(tmp_path)
    try:
        old=rig.ends['leader'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        node=PlanarLeader(namespace='/leader',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(rig.file))])
        rig.ends['leader']=node;rig.nodes.append(node);rig.executor.add_node(node)
        rig.start()
        msg=PoseStamped();msg.header.stamp=Time(seconds=rig.sim).to_msg();msg.header.frame_id='leader/base_footprint'
        msg.pose.orientation.w=1.
        if axis==0:msg.pose.position.x=sign*.0001
        if axis==1:msg.pose.position.y=sign*.0001
        if axis==2:msg.pose.orientation.z=np.sin(sign*.0002/2);msg.pose.orientation.w=np.cos(sign*.0002/2)
        node.goal(msg);rig.pump(.08)
        assert node.accepted==1 and node.planar_velocity[axis]*sign>0
        rig.permit=False;rig.pump(.1)
        assert node.latched and node.path_goal is None and not node.planar_velocity.any()
    finally:rig.close()


from test_cp7_safety import guard_rig
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def test_active_final_writer_forwards_without_extra_timer_period_and_rejects_expiry(guard_rig):
    rig=guard_rig;rig.start();guard=rig.guards['follower'];guard.timer.cancel()
    count=guard.arm_deliveries
    msg=JointTrajectory(joint_names=list(guard.model.names))
    msg.header.stamp=Time(seconds=rig.sim).to_msg()
    msg.points=[JointTrajectoryPoint(positions=[0.]*5)]
    msg.points[0].time_from_start.nanosec=40000000
    guard.arm(msg)
    assert guard.arm_deliveries==count+1 and guard.pending_arm is None and not guard.fault
    msg.header.stamp=Time(seconds=rig.sim-.1).to_msg()
    guard.arm(msg)
    assert guard.fault=='pending arm trajectory expired at final interlock'
    assert guard.arm_deliveries==count+1


@pytest.mark.parametrize('bad',['frame','stale','quaternion','roll','height','range','yaw','nan','zero'])
def test_planar_goal_rejects_invalid_units_frame_and_nonfinite_values(tmp_path,bad):
    rig=ArmRig(tmp_path)
    try:
        old=rig.ends['leader'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        node=PlanarLeader(namespace='/leader',parameter_overrides=[Parameter('use_sim_time',value=True),Parameter('urdf_file',value=str(rig.file))])
        rig.ends['leader']=node;rig.nodes.append(node);rig.executor.add_node(node);rig.start()
        msg=PoseStamped();msg.header.stamp=Time(seconds=rig.sim).to_msg();msg.header.frame_id='leader/base_footprint'
        msg.pose.orientation.w=1.;msg.pose.position.x=.0001
        if bad=='frame':msg.header.frame_id='world'
        if bad=='stale':msg.header.stamp=Time(seconds=rig.sim-1.).to_msg()
        if bad=='quaternion':msg.pose.orientation.w=0.
        if bad=='roll':msg.pose.orientation.x=.1
        if bad=='height':msg.pose.position.z=.01
        if bad=='range':msg.pose.position.x=.01
        if bad=='yaw':msg.pose.orientation.z=np.sin(.01/2);msg.pose.orientation.w=np.cos(.01/2)
        if bad=='nan':msg.pose.position.y=float('nan')
        if bad=='zero':msg.pose.position.x=0.
        node.goal(msg)
        assert node.accepted==0 and node.rejected==1 and node.path_goal is None
    finally:rig.close()


@pytest.mark.parametrize('axis',['x','y','yaw'])
def test_reset_rejects_motion_in_every_planar_axis(tmp_path,axis):
    rig=ArmRig(tmp_path)
    try:
        rig.start();rig.permit=False;rig.pump(.1);node=rig.ends['leader']
        twist=node.encoder.twist.twist
        if axis=='yaw':twist.angular.z=.0002
        else:setattr(twist.linear,axis,.0002)
        result=node.reset(Trigger.Request(),Trigger.Response())
        assert not result.success
    finally:rig.close()


def test_filtered_base_reallocates_arm_and_keeps_every_velocity_acceleration_bound():
    m=actual();q=HOME.copy();fast=WholeBodyAllocator(m);slow=WholeBodyAllocator(m,base_filter_hz=.2)
    desired=np.array([0.,.00005,0.,0.,0.,0.]);previous=np.zeros(8)
    for i in range(300):
        target=desired if i<150 else -desired
        direct=fast.step(q,q,target,np.zeros(3),1.,.01)
        u=slow.step(q,q,target,np.zeros(3),1.,.01)
        assert np.all(abs(u)<=slow.scale+1e-12)
        assert np.all(abs(u-previous)<=slow.acceleration*.01+1e-12)
        assert np.linalg.norm(whole_jacobian(m,q)@u-target-slow.last['residual_twist'])<1e-12
        if i==0:
            assert 0 < u[6] < direct[6]*.02
        if i==10:
            assert abs(u[0]) > abs(direct[0])*10  # arm recomputed for actual slow base
        previous=u
    assert slow.last['candidate_iterations']>0 and slow.last['iterations']>0


@pytest.mark.parametrize('cutoff',[-.1,float('nan')])
def test_invalid_base_filter_policy_is_rejected(cutoff):
    with pytest.raises(ValueError,match='filter'):WholeBodyAllocator(actual(),base_filter_hz=cutoff)


def test_filtered_base_rejects_fast_variation_after_persistent_authority_exists():
    m=actual();q=HOME.copy();peaks=[]
    # The old amplitude-only authority would still pass the high frequency
    # task directly to the base. Compare physical output to its candidate.
    for frequency in (.1,2.):
        a=WholeBodyAllocator(m,base_filter_hz=.2);out=[];candidate=[]
        for i in range(2200):
            desired=[0.,.00005*np.sin(2*np.pi*frequency*i*.01),0.,0.,0.,0.]
            u=a.step(q,q,desired,np.zeros(3),1.,.01)
            if i>=1200:
                out.append(u[6]);candidate.append(a.last['candidate_base_velocity'][1])
        peaks.append(float(np.linalg.norm(out)/np.linalg.norm(candidate)))
    assert peaks[0]>.8 and peaks[1]<.12
