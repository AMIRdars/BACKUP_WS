"""Check goal routing and refusal paths without creating any ROS node."""
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from amir_rotation_native_passive import execute_passive


def execution(tmp_path, *, ready_true=False, source_changed=False):
    evidence = tmp_path/'native_passive_local.jsonl'; evidence.write_text('unit-test-only\n')
    code = tmp_path/'source'; code.write_text('original')
    hashes = {str(code): hashlib.sha256(code.read_bytes()).hexdigest()}
    if source_changed:
        code.write_text('changed')
    states = {'leader': dict(accepted_goals=0, completed_goals=0)}
    sent = []
    def publish(msg):
        sent.append(msg.data)
        states['leader'] = dict(accepted_goals=len(sent), completed_goals=len(sent))
    def writers(topic):
        robot = topic.split('/')[1]
        return [SimpleNamespace(node_name='leader_object_rotation' if robot == 'leader' else 'follower_yaw_whole_body', node_namespace='/'+robot)]
    def wait(predicate, timeout, check_fault):
        assert check_fault and predicate()
    auditor = Mock(); auditor.finish.return_value = {'unit_only': True}
    services, phases = [], []
    phase = ['tare']
    args = SimpleNamespace(output=tmp_path, static_observe_s=10., delta_deg=.005, duration_s=75.)
    stream = evidence.open('a')
    try:
        result = execute_passive(args, node=SimpleNamespace(get_publishers_info_by_topic=writers),
            service=lambda name: services.append(name), settle=lambda t: phases.append((phase[0], t)),
            wait=wait, phase=phase, goal=SimpleNamespace(publish=publish), states=states,
            auditor=auditor, rows_stream=stream, ready=dict(seen=True, ever_true=ready_true), source_hashes=hashes)
    finally:
        stream.close()
    return result, sent, services, phases


def test_only_two_relative_leader_goals_are_sent_after_static_validation(tmp_path):
    import math
    result, sent, services, phases = execution(tmp_path)
    assert result == {'unit_only': True}
    assert sent == pytest.approx([math.radians(.005), math.radians(-.010)])
    assert services == ['/follower/admittance/enable']
    assert phases[0] == ('native_passive_static', 2.)
    assert ('native_passive_static', 10.) in phases
    assert ('native_passive_positive', 3.) in phases and ('native_passive_negative', 3.) in phases


def test_ready_true_is_a_fault_before_calibration_goals(tmp_path):
    with pytest.raises(ValueError, match='cannot publish autonomous'):
        execution(tmp_path, ready_true=True)


def test_changed_source_prevents_first_leader_goal(tmp_path):
    with pytest.raises(ValueError, match='source changed'):
        execution(tmp_path, source_changed=True)


PATH = Path(__file__).resolve().parents[1]/'audits/rotation_native_passive_20261105/queue.py'
spec = importlib.util.spec_from_file_location('native_passive_queue_test', PATH)
queue = importlib.util.module_from_spec(spec); spec.loader.exec_module(queue)


@pytest.mark.parametrize('field', ['base_failed', 'native_failed', 'missing_hold', 'wrong_scope'])
def test_failed_predecessor_prevents_passive_simulation(field):
    base = dict(state='independent_metrology_complete', exit_code=0)
    native = dict(state='native_static_complete', result=dict(passed=True, both_outputs_zero_after_stop=True,
        both_grasps_retained_after_stop=True, condition='native_static_yaw_gate0'))
    if field == 'base_failed':
        base['state'] = 'independent_metrology_failed'
    elif field == 'native_failed':
        native['result']['passed'] = False
    elif field == 'missing_hold':
        del native['result']['both_grasps_retained_after_stop']
    elif field == 'wrong_scope':
        native['result']['condition'] = 'fixed_grasp_qualified_D'
    with pytest.raises(ValueError):
        queue.validate_predecessor(base, native)


def test_unknown_gate0_refuses_cli_before_output_creation(tmp_path):
    import os
    import subprocess
    import sys
    root = PATH.parents[2]
    output = tmp_path/'no-output'
    command = [sys.executable, str(root/'tools/amir_cp8r3_native_trial.py'),
        '--run-directory', str(tmp_path/'unused'), '--output', str(output),
        '--native-metrology-directory', str(tmp_path/'unused'), '--wrench-calibration-file', str(tmp_path/'unused'),
        '--passive-yaw-audit', '--delta-deg', '.005', '--duration-s', '75', '--arm-yaw-metric', '.4',
        '--force-memory-gain', '0', '--force-velocity-gain', '0', '--xy-gain', '0', '--yaw-gain', '0']
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode != 0 and 'native passive audit requires Gate-0' in result.stderr
    assert not output.exists()


@pytest.mark.parametrize('options', [[], ['--resume-completed-base', '--base-owner-pid', '1']])
def test_passive_resume_and_live_owner_modes_are_explicit_and_exclusive(monkeypatch, options):
    forbidden = Mock(side_effect=AssertionError('physics before validation'))
    monkeypatch.setattr(queue.subprocess, 'call', forbidden)
    monkeypatch.setattr(sys, 'argv', ['queue.py', '--case', 'unit_test_unused', *options])
    with pytest.raises(SystemExit) as exc:
        queue.main()
    assert exc.value.code == 2 and not forbidden.called
