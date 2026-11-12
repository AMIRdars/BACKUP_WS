"""Physical URDF geometry: preserve XY without claiming a six-DOF arm."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.admittance import rotation_error
from amir_compliance_control.arm_only_dls import xy_velocity_dls, nullspace_pose_velocity


def model():
    path=Path(__file__).resolve().parents[1]/'audits/whole_body_compliance_20261019/cp6/generated/follower.urdf'
    xml=path.read_text();root=ET.fromstring(xml);m=LocalModel(xml)
    q=np.array([float(root.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text) for n in m.names])
    return m,q


def primary(m,q,v):return xy_velocity_dls(m,q,v,damping=.001,singular_stop=.001,margin=.01,speed=.03)[0]


def add_hold(m,q,d,v):return nullspace_pose_velocity(m,q,d,v,damping=.001,angular_weight=.1,speed=.03)


def test_height_and_pitch_side_effect_removed_without_xy_leak():
    m,q=model();j=m.jacobian(q);d=primary(m,q,[.001,0.]);dq,info=add_hold(m,q,d,[0.]*4)
    assert abs((j@d)[2])>.001  # measured geometry explains rigid grasp conflict
    np.testing.assert_allclose(j[:2]@dq,j[:2]@d,atol=1e-12)
    assert np.linalg.norm((j@dq)[2:])<1e-6
    assert np.linalg.norm(info['nullspace_xy_leak_m_s'])<1e-12


def test_5mm_straight_geometry_stays_in_pose_and_joint_envelope():
    m,q=model();p0,s0=m.frames(q);r0=s0[:3,:3].copy()
    for _ in range(500):
        p,s=m.frames(q);pose=rotation_error(s[:3,:3],r0)
        v=np.array([.001,0.]);d=primary(m,q,v);dq,info=add_hold(m,q,d,-4*np.r_[p[2]-p0[2],pose]);q+=dq*.01
        assert np.max(np.abs(dq))<=.03+1e-12
        assert np.min(np.minimum(q-m.lower,m.upper-q))>.01
    p,s=m.frames(q)
    np.testing.assert_allclose(p[:2]-p0[:2],[.005,0.],atol=2e-6)
    assert abs(p[2]-p0[2])<1e-6
    assert np.linalg.norm(rotation_error(s[:3,:3],r0))<1e-6


def test_unachievable_lateral_yaw_request_reports_nonzero_residual():
    m,q=model();j=m.jacobian(q);d=primary(m,q,[0.,.001]);dq,info=add_hold(m,q,d,[0.]*4)
    np.testing.assert_allclose(j[:2]@dq,j[:2]@d,atol=1e-10)
    assert abs(info['secondary_velocity_residual'][-1])>.001


@pytest.mark.parametrize('value',[0.,-.1,float('nan')])
def test_invalid_nullspace_policy_rejected(value):
    m,q=model()
    with pytest.raises(ValueError):nullspace_pose_velocity(m,q,np.zeros(5),np.zeros(4),damping=value,angular_weight=.1,speed=.03)
