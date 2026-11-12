"""O2 unit evidence lives in pytest temporary folders, never experiment runs."""
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

import amir_rotation_native_o2 as module
from amir_cp8r3_native_trial import prepare_control_policies
from test_rotation_native_passive import auditor, samples, gate0_unit_fixture, SOURCE, METROLOGY
from test_rotation_native_arm_policy import unit_policy


def passive_unit_result(tmp_path):
    path, report = gate0_unit_fixture(tmp_path)
    report.update(condition='native_stationary_follower_base_passive_yaw',
        passive_yaw_qualification_issued=True, positive_negative_programs_completed=True,
        autonomous_follower_yaw_enabled=False, passive_policy=dict(duration_s=75.),delta_deg=.005)
    report['states']['leader'].update(accepted_goals=2, completed_goals=2)
    raw = path.parent/'native_passive_local.jsonl'
    rows = samples(); raw.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    a = auditor()
    for row in rows:
        a.add(row)
    certificate = a.finish({str(raw): hashlib.sha256(raw.read_bytes()).hexdigest()})
    report['native_arm_policy']=unit_policy()
    report['native_arm_policy'].update({k:report['effective_audit_parameters'][k] for k in (
        'rotation_error_limit_rad','joint_margin_rad','wrench_force_limit_N','wrench_torque_limit_Nm')})
    certificate['native_arm_policy']=copy.deepcopy(report['native_arm_policy'])
    certificate.update(robot='follower', urdf_sha256=hashlib.sha256((SOURCE/'follower.urdf').read_bytes()).hexdigest())
    (path.parent/'qualification.json').write_text(json.dumps(certificate))
    path.write_text(json.dumps(report))
    return path, report, certificate


def configure(tmp_path):
    prior = tmp_path/'prior'; prior.mkdir()
    path, report, certificate = passive_unit_result(prior)
    output = tmp_path/'new'; output.mkdir()
    _, follower = prepare_control_policies(SOURCE, output, 75., 0., 0., 0., 0., .4, METROLOGY)
    p = yaml.safe_load(follower.read_text()).pop('/follower/follower_yaw_arm_base')
    follower.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body': p}))
    return path, report, certificate, output, follower


def test_o2_angle_is_bounded_by_measured_native_signal_and_preserves_qualification(tmp_path):
    path, report, certificate, output, follower = configure(tmp_path)
    plan = module.configure_o2(path, SOURCE, output, follower)
    bound = .5*min(certificate['measured_signal_rad']['positive'], -certificate['measured_signal_rad']['negative'])
    assert plan['probe_angle_rad'] == bound and plan['probe_angle_rad'] < math.radians(.005)
    policy = next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters']
    assert policy['base_gains'] == [0.]*3 and not policy['passive_yaw_calibration']
    assert policy['rotation_observation_required'] and policy['arm_yaw_admittance_enabled']
    assert policy['base_velocity_limits'][2] == pytest.approx(.5*1.875*bound/75.)
    saved = json.loads((output/'qualification.json').read_text())
    for key in certificate:
        if key != 'urdf_sha256':
            assert saved[key] == certificate[key]
    assert saved['native_contact_only'] and not plan['autonomous_follower_yaw_enabled']


@pytest.mark.parametrize('change', ['failed', 'fixed_certificate', 'wrong_scope', 'missing_hold', 'wrong_goal_count',
    'source_model', 'raw_evidence', 'snapshot'])
def test_invalid_native_passive_evidence_cannot_authorize_o2(tmp_path, change):
    path, report, certificate, output, follower = configure(tmp_path)
    if change == 'failed':
        report['passed'] = False
    elif change == 'fixed_certificate':
        certificate['native_contact_only'] = False
    elif change == 'wrong_scope':
        report['condition'] = 'fixed_grasp_stationary_follower_base_calibration'
    elif change == 'missing_hold':
        del report['both_grasps_retained_after_stop']
    elif change == 'wrong_goal_count':
        report['states']['leader']['completed_goals'] = 1
    elif change == 'source_model':
        report['fixture_sha256']['leader.urdf'] = 'changed'
    elif change == 'raw_evidence':
        (path.parent/'native_passive_local.jsonl').write_text('changed')
    elif change == 'snapshot':
        (path.parent/'fixture_snapshot/leader.urdf').unlink()
    path.write_text(json.dumps(report)); (path.parent/'qualification.json').write_text(json.dumps(certificate))
    with pytest.raises(ValueError):
        module.configure_o2(path, SOURCE, output, follower)


def recorder(tmp_path, fault=False):
    phase = ['native_o2_probe']
    r = module.NativeO2Recorder(tmp_path, phase, force_limit_N=.5, torque_limit_Nm=.05, fault_injection=fault)
    return r, phase


def debug(sequence=7, wrong_compensation=False):
    omega = 1e-6; p = [.5, -.2, .6]
    factor = 2. if wrong_compensation else 1.
    return dict(sim_time=1., state='TRANSPORT', probe_phase='positive',
        probe_requested_wz_rad_s=2e-6, probe_position_footprint_m=p,
        probe_limited_twist_footprint=[-.2*omega*factor, -.5*omega*factor, omega],
        base_twist=[-.2*omega*factor, -.5*omega*factor, omega],
        consumed_observation=dict(publication_sequence=sequence, source_age_ns=12_000_000))


def test_o2_compensation_uses_actual_limited_yaw_and_exact_consumed_observation(tmp_path):
    r, _ = recorder(tmp_path)
    try:
        r.observe(dict(publication_sequence=7, yaw_valid=True, marker='consumed'))
        r.observe(dict(publication_sequence=8, yaw_valid=True, marker='latest'))
        r.record(debug())
        assert not r.fault and r.moving == r.limited == 1 and r.missing == 0
        data = json.loads((tmp_path/'native_o2_local.jsonl').read_text())
        assert data['consumed_observation']['marker'] == 'consumed'
    finally:
        r.close()


def test_using_pre_limit_yaw_for_compensation_is_a_fault(tmp_path):
    r, _ = recorder(tmp_path)
    try:
        r.record(debug(wrong_compensation=True))
        assert 'limited yaw command' in r.fault
    finally:
        r.close()


def test_actual_base_command_cannot_diverge_from_verified_compensation(tmp_path):
    r,_=recorder(tmp_path)
    try:
        value=debug();value['base_twist']=[0.,0.,1e-6];r.record(value)
        assert 'actual allocated base command differs' in r.fault
    finally:r.close()


def test_missing_consumed_snapshot_is_unknown_not_replaced_by_latest(tmp_path):
    r, _ = recorder(tmp_path)
    try:
        r.observe(dict(publication_sequence=8, yaw_valid=True))
        r.record(debug())
        data = json.loads((tmp_path/'native_o2_local.jsonl').read_text())
        assert data['consumed_observation'] is None and r.missing == 1
    finally:
        r.close()


def test_invalid_observation_is_sticky_and_expected_fault_requires_injection(tmp_path):
    r, _ = recorder(tmp_path, fault=True)
    try:
        r.observe(dict(yaw_valid=False, valid_reason='stale'))
        r.observe(dict(yaw_valid=True))
        assert 'invalid' in r.fault
    finally:
        r.close()
    other = tmp_path/'other'; other.mkdir()
    r, _ = recorder(other, fault=True)
    try:
        r.injected = True; r.observe(dict(yaw_valid=False, valid_reason='stale'))
        assert not r.fault
    finally:
        r.close()


def test_o2_checks_every_raw_ft_event_with_duplicate_joint_stamp(tmp_path):
    from test_rotation_native_passive import row
    r, _ = recorder(tmp_path)
    try:
        first, bad = row(), row(); bad['wrench_tcp'][5] = .051
        r.wrench(first); r.wrench(bad)
        assert r.raw_events == 2 and r.maximum_moment == pytest.approx(.051)
        assert 'wrench exceeded' in r.fault
        events = [json.loads(line) for line in (tmp_path/'native_o2_raw_ft.jsonl').read_text().splitlines()]
        assert len(events) == 2 and 'fault' in events[-1]
    finally:
        r.close()


def test_o2_raw_source_staleness_cannot_be_hidden_by_expected_observer_fault(tmp_path):
    from test_rotation_native_passive import row
    r, _ = recorder(tmp_path, fault=True)
    try:
        r.injected = True
        bad = row(); bad['wrench_stamp'] -= .1
        r.wrench(bad)
        assert 'stale/future/skewed' in r.fault
    finally:
        r.close()


@pytest.mark.parametrize('fault', [False, True])
def test_o2_sends_no_object_goal_and_restores_injected_observer(tmp_path, monkeypatch, fault):
    phase = ['tare']; recorder = SimpleNamespace(latest=dict(reference_sequence=0),
        debug={}, limited=0, moving=0, raw_events=8, fault='', injected=False, summary=lambda: dict(unit_test_only=True))
    states = {r: dict(state='READY', accepted_goals=0) for r in ('leader', 'follower')}
    calls, signals = [], []
    def service(name):
        calls.append(name)
        if name.endswith('latch_reference'):
            recorder.latest.update(reference_sequence=1, yaw_valid=True)
        if name.endswith('start_compensation_probe'):
            recorder.limited = recorder.moving = 1
            recorder.debug = dict(probe_phase='positive' if fault else 'complete')
    def kill(pid, sig):
        signals.append((pid, sig))
        if sig == module.signal.SIGSTOP:
            for state in states.values():
                state.update(state='SAFE_STOP', reason='local yaw source age 52000000 ns exceeds 50000000 ns')
    monkeypatch.setattr(module, 'owned_observer_pids', lambda p, d: [999999999])
    monkeypatch.setattr(module.os, 'kill', kill)
    monkeypatch.setenv('IGN_PARTITION', 'amir_cp9_cp8r3_native_o2_unit')
    monkeypatch.setenv('ROS_DOMAIN_ID', '211')
    def wait(predicate, timeout, check):
        assert predicate()
    result = module.execute_o2(SimpleNamespace(o2_observer_stop=fault), service=service,
        settle=lambda seconds: None, wait=wait, phase=phase, states=states,
        outputs={'follower':[0.,0.,1e-6]}, recorder=recorder, ready=dict(seen=True,ever_true=False), source_hashes={})
    assert result['leader_object_goal_count'] == 0
    assert not result['autonomous_follower_yaw_enabled']
    assert calls == ['/follower/rotation/latch_reference', '/follower/admittance/enable', '/follower/rotation/start_compensation_probe']
    if fault:
        assert result['observer_stop_detected']
        assert signals == [(999999999,module.signal.SIGSTOP),(999999999,module.signal.SIGCONT)]
    else:
        assert result['positive_negative_pulse_completed'] and not signals


def test_fault_pid_lookup_refuses_non_owned_partition():
    with pytest.raises(ValueError,match='isolated native O2 owner'):
        module.owned_observer_pids('unrelated_simulator','207')
