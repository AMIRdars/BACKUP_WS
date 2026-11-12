"""Generate only: no LaunchService or simulation during these checks."""
import json
from pathlib import Path
import shutil

import pytest
import yaml

from amir_rotation_base_fixture import prepare, sha


ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT/'audits/rotation_equivariant_expansion_20261105/wb_15_03/generated'
CANDIDATE = ROOT/'audits/rotation_equivariant_expansion_20261105/wheel_geometry_candidate_01'


def test_baseline_matches_measured_robot_and_retains_default_controller_bytes(tmp_path):
    actions, proof = prepare(tmp_path/'baseline', REFERENCE)
    assert actions and proof['ungrasped_only'] and not proof['candidate_applied']
    assert not proof['transport_qualified']
    assert 'GraspHeartbeat' not in proof['plugins']
    for robot in ('leader', 'follower'):
        assert proof['robots'][robot]['canonical_model_identical']
        assert (tmp_path/'baseline'/(robot+'_base_controller.yaml')).read_bytes() == (REFERENCE/(robot+'_base_controller.yaml')).read_bytes()


def test_candidate_is_applied_only_to_new_fixture_and_not_reference(tmp_path):
    hashes = {p: sha(p) for p in REFERENCE.glob('*') if p.is_file()}
    actions, proof = prepare(tmp_path/'candidate', REFERENCE, CANDIDATE)
    assert actions and proof['candidate_applied']
    for robot in ('leader', 'follower'):
        config = tmp_path/'candidate'/(robot+'_base_controller.yaml')
        assert config.read_bytes() == (CANDIDATE/config.name).read_bytes()
    assert hashes == {p: sha(p) for p in hashes}


def test_candidate_hash_tampering_fails_before_any_launch(tmp_path):
    changed = tmp_path/'changed'
    shutil.copytree(CANDIDATE, changed)
    p = changed/'leader_base_controller.yaml'
    p.write_text(p.read_text()+'\n')
    with pytest.raises(ValueError, match='hashes mismatch'):
        prepare(tmp_path/'rejected', REFERENCE, changed)


def test_candidate_cannot_smuggle_unreviewed_radius_even_with_updated_hash(tmp_path):
    changed = tmp_path/'changed'
    shutil.copytree(CANDIDATE, changed)
    p = changed/'leader_base_controller.yaml'
    data = yaml.safe_load(p.read_text())
    next(iter(data.values()))['ros__parameters']['kinematics']['wheels_radius'] += .001
    p.write_text(yaml.safe_dump(data))
    proof = json.loads((changed/'proof.json').read_text())
    proof['robots']['leader']['candidate_controller_sha256'] = sha(p)
    (changed/'proof.json').write_text(json.dumps(proof))
    with pytest.raises(ValueError, match='more than the audited'):
        prepare(tmp_path/'rejected', REFERENCE, changed)
