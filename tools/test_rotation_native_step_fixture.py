import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from amir_rotation_native_step_fixture import refine

WORLD=Path(__file__).resolve().parents[1]/'audits/object_center_rotation_20261103/cp8r3/native_generated/world.sdf'


def fixture(tmp_path):
    raw=WORLD.read_bytes();(tmp_path/'world.sdf').write_bytes(raw)
    (tmp_path/'follower.urdf').write_text('unit test immutable robot')
    hashes={name:hashlib.sha256((tmp_path/name).read_bytes()).hexdigest() for name in ('world.sdf','follower.urdf')}
    (tmp_path/'native_fixture_proof.json').write_text(json.dumps({'generated_sha256':hashes}))
    (tmp_path/'wrench_calibration.json').write_text(json.dumps({'sensor_calibration_passed':True,'sign':-1.}))
    return raw


@pytest.mark.parametrize('divisor',[2,4])
def test_refinement_changes_only_the_actual_baseline_step(tmp_path,divisor):
    raw=fixture(tmp_path);robot=(tmp_path/'follower.urdf').read_bytes()
    evidence=refine(tmp_path,divisor)
    tree=ET.parse(tmp_path/'world.sdf').getroot();original=ET.fromstring(raw)
    assert float(tree.findtext('world/physics/max_step_size'))==float(original.findtext('world/physics/max_step_size'))/divisor
    tree.find('world/physics/max_step_size').text=original.findtext('world/physics/max_step_size')
    assert ET.tostring(tree)==ET.tostring(original)
    assert (tmp_path/'follower.urdf').read_bytes()==robot
    assert evidence['new_sensor_dynamic_calibration_verified'] is False
    assert evidence['autonomous_yaw_qualification_issued'] is False
    assert (tmp_path/'world_before_step_refinement.sdf').read_bytes()==raw
    cert=json.loads((tmp_path/'wrench_calibration.json').read_text())
    assert cert['sign']==-1. and cert['numerical_refinement']==evidence


@pytest.mark.parametrize('divisor',[1,0,-2,3,2.,True])
def test_invalid_resolution_request_leaves_fixture_unchanged(tmp_path,divisor):
    raw=fixture(tmp_path)
    with pytest.raises(ValueError):refine(tmp_path,divisor)
    assert (tmp_path/'world.sdf').read_bytes()==raw


def test_changed_fixture_cannot_be_refined_as_the_same_baseline(tmp_path):
    raw=fixture(tmp_path);(tmp_path/'follower.urdf').write_text('changed')
    with pytest.raises(ValueError,match='baseline fixture changed'):refine(tmp_path,2)
    assert (tmp_path/'world.sdf').read_bytes()==raw
