"""An exact chain symmetry is distinct from an empirical fit or added sensor."""
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from amir_compliance_control.rotation_observer import ProjectedYaw,read_qualification,wrap_pi
from amir_compliance_control.wrench import rotation
from test_local_yaw import URDF,HOME
from test_rotation_source_alignment import aligned


def test_first_vertical_axis_rotates_the_whole_fk_and_preserves_yaw_sensitivity():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    proof=g.yaw_equivariance_proof;assert proof and proof['joint_index']==0
    root=np.array(proof['origin_footprint_m']);rng=np.random.default_rng(8351)
    for _ in range(12):
        q=HOME.copy();q[1:]+=rng.uniform(-1e-4,1e-4,4)
        original=g.measure(q)
        for angle in np.linspace(g.model.lower[0]+2*g.margin,g.model.upper[0]-2*g.margin,11):
            varied=q.copy();varied[0]=angle;m=g.measure(varied)
            np.testing.assert_allclose(m['p_ee_footprint'],root+rotation([0,0,1],angle)@(np.array(original['p_ee_footprint'])-root),atol=1e-14)
            assert abs(wrap_pi(m['psi_ee_B']-original['psi_ee_B']-angle))<1e-14
            assert m['projection_norm']==pytest.approx(original['projection_norm'],abs=1e-14)
            np.testing.assert_allclose(m['yaw_jacobian'],original['yaw_jacobian'],atol=1e-12)


def test_tilted_first_axis_cannot_receive_the_vertical_symmetry_certificate(tmp_path):
    xml=ET.fromstring(URDF.read_text());xml.find("joint[@name='Joint_1']/axis").set('xyz','0 .1 1')
    g=ProjectedYaw(ET.tostring(xml,encoding='unicode'),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    assert g.yaw_equivariance_proof is None


def test_extension_is_opt_in_and_keeps_other_joint_bounds_and_pose_envelope(aligned,tmp_path):
    node,_,q=aligned;node.cert['qualified_q_lower_rad'][0]=-1e-5;node.cert['qualified_q_upper_rad'][0]=1e-5
    moved=q.copy();moved[0]=.0001
    with pytest.raises(ValueError,match='outside measured'):node.require_region(moved)
    node.cert.update(yaw_equivariance_proof=node.geometry.yaw_equivariance_proof,
        equivariant_joint_pose_envelope_rad=node.geometry.envelope)
    file=tmp_path/'extended.json';file.write_text(json.dumps(node.cert))
    read_qualification(file,node.urdf_hash,node.geometry)
    node.require_region(moved)
    moved[0]=.010001
    with pytest.raises(ValueError,match='existing pose envelope'):node.require_region(moved)
    moved=q.copy();moved[1]=node.cert['qualified_q_upper_rad'][1]+1e-6
    with pytest.raises(ValueError,match='outside measured'):node.require_region(moved)
    node.cert['yaw_equivariance_proof']=dict(node.cert['yaw_equivariance_proof'],joint_index=1)
    file.write_text(json.dumps(node.cert))
    with pytest.raises(ValueError,match='equivariance'):read_qualification(file,node.urdf_hash,node.geometry)
