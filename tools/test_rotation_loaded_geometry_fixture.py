"""Candidate loaded fixture is file preparation, without copied permission."""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest
import yaml

from amir_rotation_loaded_geometry_fixture import prepare, sha


ROOT = Path(__file__).resolve().parents[1]/'audits/rotation_equivariant_expansion_20261105'
REFERENCE = ROOT/'wb_15_03/generated'
CANDIDATE = ROOT/'wheel_geometry_candidate_01'


def test_generated_fixture_preserves_loaded_world_interfaces_and_no_permission(tmp_path):
    source_hashes = {str(p): sha(p) for folder in (REFERENCE, CANDIDATE) for p in folder.iterdir() if p.is_file()}
    output = tmp_path/'fresh'
    proof = prepare(output, REFERENCE, CANDIDATE)
    for key in ('physics_started', 'ready_to_launch', 'transport_qualified', 'precision_qualified',
                'yaw_qualification_issued', 'grasp_calibration_transferred'):
        assert proof[key] is False
    for robot in ('leader', 'follower'):
        assert proof['robots'][robot]['canonical_model_identical'] is True
        assert (output/(robot+'_base_controller.yaml')).read_bytes() == (CANDIDATE/(robot+'_base_controller.yaml')).read_bytes()
    assert not list(output.glob('*calibration*.json'))
    assert not (output/'qualification.json').exists()
    assert {name: sha(Path(name)) for name in source_hashes} == source_hashes
    with pytest.raises(FileExistsError):
        prepare(output, REFERENCE, CANDIDATE)


@pytest.mark.parametrize('tamper', ['other_parameter', 'projection', 'world', 'model', 'interfaces'])
def test_rejects_changed_geometry_or_fixed_scene_even_with_updated_file_hash(tmp_path, tamper):
    reference, candidate = tmp_path/'reference', tmp_path/'candidate'
    shutil.copytree(REFERENCE, reference)
    shutil.copytree(CANDIDATE, candidate)
    p = candidate/'proof.json'
    proof = json.loads(p.read_text())
    if tamper in ('other_parameter', 'projection'):
        config = candidate/'leader_base_controller.yaml'
        data = yaml.safe_load(config.read_text())
        parameters = data['/**/mecanum_drive_controller']['ros__parameters']
        if tamper == 'other_parameter':
            parameters['enable_odom_tf'] = False
        else:
            parameters['kinematics']['sum_of_robot_center_projection_on_X_Y_axis'] = .3
        config.write_text(yaml.safe_dump(data))
        proof['robots']['leader']['candidate_controller_sha256'] = sha(config)
    elif tamper == 'world':
        f = reference/'world.sdf'
        tree = ET.parse(f)
        mass = tree.find("world/model[@name='cooperative_payload']/link/inertial/mass")
        mass.text = str(float(mass.text)+1.)
        tree.write(f)
    elif tamper == 'model':
        f = reference/'leader.urdf'
        tree = ET.parse(f)
        tree.getroot().set('name', 'changed_measured_model')
        tree.write(f)
    elif tamper == 'interfaces':
        f = reference/'leader_controllers.yaml'
        f.write_text(f.read_text()+'\n# modified controller interface file\n')
    p.write_text(json.dumps(proof))
    with pytest.raises(ValueError):
        prepare(tmp_path/'refused', reference, candidate)
