from pathlib import Path
import subprocess

import numpy as np
import pytest

from amir_follower_control.core import (
    Admittance4D, ArmURDF, Recenter, WrenchObserver, allocate,
    allowed_subscriptions, base_jacobian, check_timing, interpolate_joints, rotation, transform_wrench)


@pytest.mark.parametrize('seed', range(6))
def test_static_load_matches_external_virtual_work_and_gravity_potential(arm,seed):
    q=np.random.default_rng(seed).uniform(arm.lower+.2,arm.upper-.2)
    force=np.array([2.,-1.,-40.])
    sensor=arm.evaluate(q)[1]
    raw=np.r_[sensor[:3,:3].T@force,[0.,0.,0.]]
    load=arm.quasistatic_joint_load(q,raw,1.)
    def potential(q_):
        frames,_,_=arm._kinematics(q_)
        energy=0.
        for anchor,_,mass,offset in arm.upstream_masses:
            com=frames[anchor][:3,3]+frames[anchor][:3,:3]@offset
            energy+=mass*9.81*com[2]
        return energy
    eps=1e-6
    expected=[]
    for k in range(5):
        plus,minus=q.copy(),q.copy()
        plus[k]+=eps;minus[k]-=eps
        ds=(arm.evaluate(plus)[1][:3,3]-arm.evaluate(minus)[1][:3,3])/(2*eps)
        expected.append(force@ds-(potential(plus)-potential(minus))/(2*eps))
    np.testing.assert_allclose(load,expected,atol=1e-7)
    # Sensor-side gravity is represented by the wrench, never added again.
    assert arm.quasistatic_joint_load(q,[0,0,0,.2,.3,.4],1.)[4]==pytest.approx(.3)


def test_static_load_includes_fixed_side_attachment(arm):
    import xml.etree.ElementTree as ET
    from amir_follower_control.core import origin
    from ament_index_python.packages import get_package_share_directory
    model=Path(get_package_share_directory('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
    root=ET.fromstring(subprocess.check_output(['xacro',str(model),'namespace:=amir2'],text=True))
    attachment=ET.SubElement(root,'link',name='test_fixed_mass')
    inertia=ET.SubElement(attachment,'inertial')
    ET.SubElement(inertia,'mass',value='2')
    joint=ET.SubElement(root,'joint',name='test_mass_mount',type='fixed')
    ET.SubElement(joint,'parent',link='link2_1')
    ET.SubElement(joint,'child',link='test_fixed_mass')
    ET.SubElement(joint,'origin',xyz='.1 0 0')
    changed=ArmURDF(ET.tostring(root,encoding='unicode'))
    q=np.array([0.,1.4,-1.6,.2,0.])
    difference=changed.quasistatic_joint_load(q,[0]*6,1.)-arm.quasistatic_joint_load(q,[0]*6,1.)
    eps=1e-6
    expected=[]
    for k in range(5):
        plus,minus=q.copy(),q.copy();plus[k]+=eps;minus[k]-=eps
        def height(q_):
            t=changed._kinematics(q_)[0]['link2_1']
            return (t@origin(joint.find('origin')))[2,3]
        expected.append(-2*9.81*(height(plus)-height(minus))/(2*eps))
    np.testing.assert_allclose(difference,expected,atol=1e-7)


@pytest.fixture(scope='module')
def arm():
    # Expand the actual installed model, never a hand-copied DH table.
    from ament_index_python.packages import get_package_share_directory
    model = Path(get_package_share_directory('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
    xml = subprocess.check_output(['xacro',str(model),'namespace:=amir2'], text=True)
    return ArmURDF(xml)


@pytest.mark.parametrize('seed', range(8))
def test_actual_urdf_jacobian_finite_difference(arm, seed):
    q = np.random.default_rng(seed).uniform(arm.lower+.2, arm.upper-.2)
    t, _, jac = arm.evaluate(q)
    numerical = np.zeros((4,5))
    eps = 1e-6
    for k in range(5):
        qp, qm = q.copy(), q.copy()
        qp[k] += eps
        qm[k] -= eps
        tp, _, _ = arm.evaluate(qp)
        tm, _, _ = arm.evaluate(qm)
        numerical[:3,k] = (tp[:3,3]-tm[:3,3])/(2*eps)
        omega = ((tp[:3,:3]-tm[:3,:3])/(2*eps)) @ t[:3,:3].T
        numerical[3,k] = omega[1,0]
    np.testing.assert_allclose(jac, numerical, atol=1e-8)


def test_sensor_shift_rotation_and_sign():
    t = np.eye(4)
    t[:3,:3] = rotation([0,0,1],np.pi/2)
    t[:3,3] = [0,1,0]
    # Force S+x -> B+y, lever G->S=(-1,1,0), tau_z=-2 Nm.
    out = transform_wrench([2,0,0,0,0,0],t,[1,0,0],1.)
    np.testing.assert_allclose(out,[0,2,0,0,0,-2],atol=1e-12)
    np.testing.assert_allclose(transform_wrench([2,0,0,0,0,0],t,[1,0,0],-1.),-out)
    with pytest.raises(ValueError):
        transform_wrench([0]*6,t,[0]*3,0.)


@pytest.mark.parametrize('axis', range(4))
def test_each_admittance_axis_isolated_bounded_and_passive(axis):
    axes = np.arange(4)==axis
    adm = Admittance4D(axes)
    w = np.zeros(4)
    w[axis] = 3 if axis < 3 else .3
    for _ in range(800):
        v = adm.step(w,.01)
        assert v[axis] >= 0
        assert np.all(v[~axes] == 0)
        assert np.all(np.abs(adm.x) <= adm.xmax)
    previous = .5*np.sum(adm.mass*adm.v**2+adm.stiffness*adm.x**2)
    for _ in range(500):
        adm.step(np.zeros(4),.01)
        energy = .5*np.sum(adm.mass*adm.v**2+adm.stiffness*adm.x**2)
        assert energy <= previous+1e-10
        previous = energy


@pytest.mark.parametrize('dt', [0.,-.01,.1,float('nan')])
def test_clock_faults(dt):
    with pytest.raises(ValueError):
        Admittance4D().step(np.zeros(4),dt)


def test_stamps_and_nonfinite():
    check_timing(1.,[.98,.99,1.])
    for stamps in ([.1,.99,1.],[1.01,1.,1.],[np.nan,1.,1.],[0.,0.,0.]):
        with pytest.raises(ValueError):
            check_timing(1.,stamps)


def test_wrench_time_joint_interpolation_has_no_latest_fallback():
    samples=[(1.,np.zeros(5)),(1.01,np.ones(5))]
    np.testing.assert_allclose(interpolate_joints(samples,1.005),.5)
    np.testing.assert_allclose(interpolate_joints(samples,1.01),1.)
    for t in (.99,1.02):
        with pytest.raises(ValueError,match='bracket'):
            interpolate_joints(samples,t)
    with pytest.raises(ValueError,match='gap'):
        interpolate_joints([(1.,np.zeros(5)),(1.1,np.ones(5))],1.05)


def test_base_jacobian_finite_difference():
    p = np.array([.5,-.3,.6])
    u = np.array([.01,.02,.04])
    eps = 1e-6
    next_p = rotation([0,0,1],eps*u[2])@p+np.r_[u[:2]*eps,0]
    np.testing.assert_allclose(base_jacobian(p)@u,np.r_[(next_p-p)/eps,u[2]],atol=1e-8)
    assert (base_jacobian(p)@u)[2] == 0


def test_dls_subtracts_base_and_respects_bounds(arm):
    q = np.array([.1,1.4,-1.6,.2,1.57])
    tg, _, jac = arm.evaluate(q)
    u = np.array([.005,-.005,.01])
    desired = base_jacobian(tg[:3,3])@u
    result = allocate(arm,q,np.zeros(5),jac,desired,u,.01,tg[:3,3])
    assert np.linalg.norm(result.qdot) < .01
    assert np.max(np.abs(result.qdot)) <= .005+1e-12
    with pytest.raises(ValueError):
        allocate(arm,arm.upper,np.zeros(5),jac,desired,u,.01,tg[:3,3])


def test_home_rank_three_accepts_fx_but_rejects_independent_lateral_yaw(arm):
    q=np.array([0,1.4,-1.6,.2,1.570796])
    tg,_,jac=arm.evaluate(q)
    assert np.linalg.matrix_rank(jac)==3
    allocate(arm,q,np.zeros(5),jac,[.01,0,0,0],np.zeros(3),.01,tg[:3,3])
    for request in ([0,.01,0,0],[0,0,0,.05]):
        with pytest.raises(ValueError,match='Jacobian range'):
            allocate(arm,q,np.zeros(5),jac,request,np.zeros(3),.01,tg[:3,3])


def test_zero_wrench_does_not_move_toward_joint_midlimits(arm):
    q=np.array([0.,1.377328,-1.113836,-.463492,1.570796])
    tg,_,jac=arm.evaluate(q)
    out=allocate(arm,q,np.zeros(5),jac,np.zeros(4),np.zeros(3),.01,tg[:3,3])
    np.testing.assert_allclose(out.qdot,0,atol=1e-12)


@pytest.mark.parametrize('axis',[0,2])
def test_planar_compliance_preserves_clamp_pitch_even_when_acceleration_limited(arm,axis):
    q=np.array([0.,1.377328,-1.113836,-.463492,1.570796])
    tg,_,jac=arm.evaluate(q)
    task=np.zeros(4)
    task[axis]=.02
    out=allocate(arm,q,np.zeros(5),jac,task,np.zeros(3),.01,tg[:3,3])
    angular=arm.evaluate(q,full_jacobian=True)[2][3:]@out.qdot
    np.testing.assert_allclose(angular,0,atol=1e-6)
    assert np.max(np.abs(out.qdot))<=.005+1e-10


def test_recenter_hysteresis_no_z_and_acceleration_limit():
    r = Recenter()
    np.testing.assert_allclose(r.step([0,0,.1],0,.01,True),0)
    u = r.step([.03,.01,0],0,.01,True)
    assert r.active and np.max(np.abs(u)) <= .0002+1e-12
    r.step([.01,0,0],0,.01,True)
    assert r.active
    r.step([.001,0,0],0,.01,True)
    assert not r.active


def test_small_load_recenter_profile_preserves_hysteresis():
    r=Recenter(.001,.0004)
    r.step([.0011,0.,.1],0.,.01,True)
    assert r.active and r.command[0]>0 and r.command[1]==0
    r.step([.0005,0.,.1],0.,.01,True)
    assert r.active
    r.step([.0003,0.,.1],0.,.01,True)
    assert not r.active
    with pytest.raises(ValueError):Recenter(.001,.002)


def test_hold_bias_uses_distinct_sensor_samples():
    observer = WrenchObserver(3)
    for _ in range(3):
        np.testing.assert_allclose(observer.update(np.ones(6),.01,True),0)
    assert observer.count == 3


def test_whitelist_excludes_ground_truth_and_leader():
    names = allowed_subscriptions('amir2')
    assert '/amir2/odom' not in names
    assert not any('amir1' in n or 'world' in n or 'payload' in n for n in names)
    assert '/amir2/mecanum_drive_controller/odometry' in names
