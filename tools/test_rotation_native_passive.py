"""Synthetic unit evidence is confined to tests; it never authorizes a run."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import pytest
import yaml

from amir_compliance_control.rotation_observer import ProjectedYaw
from amir_rotation_native_passive import NativePassiveYaw, configure_passive, export_passive_csv, require_native_gate0
from amir_cp8r3_native_trial import prepare_control_policies


WORKSPACE = Path(__file__).resolve().parents[1]
NATIVE = WORKSPACE/'audits/object_center_rotation_20261103/cp8r3'
SOURCE = NATIVE/'native_generated'
METROLOGY = NATIVE/'native_metrology/calibration/calibration.json'
LEADER_LIMITS = WORKSPACE/'audits/rotation_equivariant_expansion_20261105/wb_15_03/leader_policy.yaml'
HOME_MAPPING = json.loads((NATIVE/'native_smoke005_08/result.json').read_text())['preparation_final']['follower']['home_from_actual_urdf']
HOME = [HOME_MAPPING[f'Joint_{i}'] for i in range(1, 6)]


def auditor():
    geometry = ProjectedYaw((SOURCE/'follower.urdf').read_text(), orientation_envelope_rad=.01,
                            joint_margin_rad=.01, heading_frame='grasp_heading')
    return NativePassiveYaw(geometry, force_limit_N=.5, torque_limit_Nm=.05)


def row(index=0, phase='static', angle=0., rate=0.):
    t = 1.+index*.02
    q = HOME.copy(); q[0] += angle
    return dict(phase=phase, timestamp=t, joint_stamp=t, wrench_stamp=t, odom_stamp=t,
        q_rad=q, qdot_rad_s=[rate, 0., 0., 0., 0.], wrench_tcp=[0.]*5+[64.*angle],
        base_twist=[0.]*3, base_command=[0.]*3, grasp_attached=True)


def samples():
    rows = [row(i, angle=1e-10*np.sin(i)) for i in range(32)]
    for phase, start, change in (('positive', 0., 5e-5), ('negative', 5e-5, -1e-4)):
        for i in range(1, 101):
            s = i/100.
            angle = start+change*(10*s**3-15*s**4+6*s**5)
            rate = change*(30*s*s-60*s**3+30*s**4)/2.
            rows.append(row(len(rows), phase, angle, rate))
    return rows


def test_detects_positive_negative_own_fk_yaw_without_any_base_rotation():
    a = auditor()
    for r in samples():
        a.add(r)
    result = a.finish({'unit-evidence-only': 'unit-hash'})
    assert result['native_contact_only'] and result['passive_positive_negative_passed']
    assert result['measured_signal_rad']['positive'] > 0. and result['measured_signal_rad']['negative'] < 0.
    assert result['minimum_signal_to_noise'] > 6.
    assert all(r['base_twist'] == [0.]*3 for r in a.records)


def test_duplicate_raw_ft_excursion_is_not_hidden_by_fk_deduplication():
    a = auditor(); a.add(row())
    r = row(); r['wrench_tcp'][5] = .051
    with pytest.raises(ValueError, match='wrench exceeded'):
        a.add(r)
    assert a.maximum_moment == pytest.approx(.051)


@pytest.mark.parametrize('change, message', [
    ({'base_command': [0., 0., 1e-6]}, 'stationary'),
    ({'base_twist': [.0001, 0., 0.]}, 'stationary'),
    ({'grasp_attached': False}, 'grasp not retained'),
    ({'wrench_stamp': .94}, 'stale/future/skewed'),
    ({'odom_stamp': 1.03}, 'stale/future/skewed'),
    ({'qdot_rad_s': [999.]*5}, 'URDF'),
    ({'wrench_tcp': [float('nan')]*6}, 'nonfinite'),
    ({'object_pose_GT': [[1.]*4]*4}, 'own measurements only'),
])
def test_invalid_or_forbidden_measurements_cannot_be_qualified(change, message):
    a = auditor(); r = row(); r.update(change)
    with pytest.raises(ValueError, match=message):
        a.add(r)


def test_repeated_joint_stamp_with_changed_fk_is_rejected():
    a = auditor(); a.add(row()); r = row(); r['q_rad'][0] += 1e-8
    with pytest.raises(ValueError, match='discontinuity'):
        a.add(r)


def test_flat_yaw_cannot_obtain_native_passive_certificate():
    a = auditor()
    for i, phase in enumerate(['static']*32+['positive']*32+['negative']*32):
        a.add(row(i, phase))
    with pytest.raises(ValueError, match='not detectable'):
        a.finish({'unit-only': 'unit-hash'})


def test_reversed_yaw_sign_is_rejected_by_measured_detectability():
    a = auditor()
    for r in samples():
        r['q_rad'][0] = 2.*HOME[0]-r['q_rad'][0]
        r['qdot_rad_s'][0] *= -1
        a.add(r)
    with pytest.raises(ValueError, match='not detectable'):
        a.finish({'unit-only': 'unit-hash'})


def test_static_phase_is_mandatory_before_motion():
    with pytest.raises(ValueError, match='eight'):
        auditor().add(row(0, 'positive'))


def test_own_fk_and_measured_joint_velocity_must_be_consistent():
    a = auditor()
    for r in samples():
        r['qdot_rad_s'] = [0.]*5
        a.add(r)
    with pytest.raises(ValueError, match='inconsistent'):
        a.finish({'unit-only': 'unit-hash'})


def policies(tmp_path):
    return prepare_control_policies(SOURCE, tmp_path, 75., 0., 0., 0., 0., .4, METROLOGY)


def test_passive_policy_keeps_arm_mechanics_native_caps_and_the_existing_endpoint(tmp_path):
    leader, follower = policies(tmp_path)
    before = copy.deepcopy(next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters'])
    follower.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body': {'ros__parameters': before}}))
    result = configure_passive(leader, follower, delta_deg=.005, duration_s=75., leader_limits_file=LEADER_LIMITS)
    after = yaml.safe_load(follower.read_text())
    assert set(after) == {'/follower/follower_yaw_whole_body'}
    p = next(iter(after.values()))['ros__parameters']
    for key in ('arm_task_metric', 'virtual_yaw_inertia', 'virtual_yaw_damping', 'virtual_yaw_stiffness',
                'wrench_force_limit_N', 'wrench_torque_limit_Nm', 'base_velocity_limits', 'base_acceleration_limits'):
        assert p[key] == before[key]
    assert p['base_gains'] == [0.]*3 and p['passive_yaw_calibration']
    assert p['arm_yaw_admittance_enabled'] and not p['rotation_observation_required']
    assert not result['autonomous_follower_yaw_enabled']


@pytest.mark.parametrize('angle,duration', [(3., 75.), (-.005, 75.), (.005, 30.), (float('nan'), 75.)])
def test_initial_passive_experiment_is_bounded_before_policy_mutation(tmp_path, angle, duration):
    leader, follower = policies(tmp_path)
    before = (leader.read_bytes(), follower.read_bytes())
    with pytest.raises(ValueError, match='initial native passive envelope'):
        configure_passive(leader, follower, delta_deg=angle, duration_s=duration, leader_limits_file=LEADER_LIMITS)
    assert before == (leader.read_bytes(), follower.read_bytes())


def test_passive_csv_preserves_phase_duplicate_ft_and_terminal_fault(tmp_path):
    one = row(); two = row(); two['wrench_tcp'][5] = .051
    source = tmp_path/'raw.jsonl'
    source.write_text('\n'.join(json.dumps(r) for r in (one, two, dict(timestamp=1., fault='raw overload')))+'\n')
    target = tmp_path/'samples.csv'
    assert export_passive_csv(source, target, auditor().geometry) == 3
    with target.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]['phase'] == 'static' and rows[0]['cmd_wz_rad_s'] == '0.0'
    assert rows[0]['joint_stamp'] == rows[1]['joint_stamp'] and rows[1]['tz_Nm'] == '0.051'
    assert rows[2]['fault'] == 'raw overload'


def gate0_unit_fixture(tmp_path):
    """A synthetic report tests validation; it is not an experiment result."""
    result_dir = tmp_path/'synthetic_gate0'; result_dir.mkdir()
    files = ['world.sdf', *[r+s for r in ('leader', 'follower') for s in ('.urdf', '_controllers.yaml', '_base_controller.yaml')]]
    hashes = {}
    for name in files:
        target = result_dir/'fixture_snapshot'/name; target.parent.mkdir(exist_ok=True)
        target.write_bytes((SOURCE/name).read_bytes()); hashes[name] = hashlib.sha256(target.read_bytes()).hexdigest()
    code = WORKSPACE/'tools/amir_rotation_native_static.py'
    target = result_dir/'source_snapshot/tools'/code.name; target.parent.mkdir(parents=True)
    target.write_bytes(code.read_bytes())
    (result_dir/'source_sha256.json').write_text(json.dumps({str(code): hashlib.sha256(code.read_bytes()).hexdigest()}))
    report = dict(condition='native_static_yaw_gate0', passed=True,
        both_outputs_zero_after_stop=True, both_grasps_retained_after_stop=True,
        leader_input_boundary_passed=True, follower_input_boundary_passed=True,
        grasp_loss_events=[], autonomous_yaw_qualification_issued=False,
        native_gate0=dict(gate0_static_passed=True, yaw_std_rad=1e-8, samples=32, measured_duration_s=1.,
            force_limit_N=.5, torque_limit_Nm=.05),
        states={r: dict(state='SAFE_STOP', accepted_goals=0, completed_goals=0) for r in ('leader', 'follower')},
        argv=['test-only', '--static-yaw-audit', '--run-directory', str(SOURCE)],
        fixture_unchanged=True, fixture_sha256=hashes)
    effective = dict(rotation_error_limit_rad=.01, joint_margin_rad=.01,
                     wrench_force_limit_N=.5, wrench_torque_limit_Nm=.05, yaw_heading_frame='grasp_heading')
    report['effective_audit_parameters'] = effective
    (result_dir/'static_effective_parameters.json').write_text(json.dumps(effective))
    path = result_dir/'result.json'; path.write_text(json.dumps(report))
    return path, report


def test_matching_recorded_gate0_model_and_snapshots_are_required(tmp_path):
    path, _ = gate0_unit_fixture(tmp_path)
    proof = require_native_gate0(path, SOURCE)
    assert proof['model_world_controllers_identical']
    assert proof['result_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize('change', ['load_fault', 'wrong_scope', 'nonzero_goal', 'fixture_changed', 'source_missing', 'noise_nan', 'changed_limit'])
def test_failed_or_changed_gate0_cannot_start_native_rotation(tmp_path, change):
    path, report = gate0_unit_fixture(tmp_path)
    if change == 'load_fault':
        report['passed'] = False
    elif change == 'wrong_scope':
        report['condition'] = 'fixed_grasp_qualified_D'
    elif change == 'nonzero_goal':
        report['states']['leader']['accepted_goals'] = 1
    elif change == 'fixture_changed':
        (path.parent/'fixture_snapshot/leader.urdf').write_text('changed')
    elif change == 'source_missing':
        shutil.rmtree(path.parent/'source_snapshot')
    elif change == 'noise_nan':
        report['native_gate0']['yaw_std_rad'] = float('nan')
    elif change == 'changed_limit':
        report['effective_audit_parameters']['wrench_force_limit_N'] *= 2.
    path.write_text(json.dumps(report))
    with pytest.raises(ValueError):
        require_native_gate0(path, SOURCE)
