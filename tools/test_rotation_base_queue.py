"""The waiting worker must require real completion and verified stop/hold."""
import copy
import importlib.util
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest

PATH = Path(__file__).resolve().parents[1]/'audits/rotation_base_metrology_20261105/queue.py'
spec = importlib.util.spec_from_file_location('base_metrology_queue', PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def evidence():
    sweep = dict(state='selected_sweep_complete', through_deg=90.)
    fixed = dict(passed=True, positive_negative_programs_completed=True,
        stop_and_hold_confirmed=True, both_fixed_grasps_retained=True,
        completed_goals=2, delta_deg=90., grasp_losses=[])
    native = dict(state='native_static_complete', result=dict(passed=True,
        both_outputs_zero_after_stop=True, both_grasps_retained_after_stop=True,
        condition='native_static_yaw_gate0', autonomous_yaw_qualification_issued=False,
        grasp_loss_events=[], states={r: dict(state='SAFE_STOP', accepted_goals=0, completed_goals=0) for r in ('leader', 'follower')}))
    return sweep, fixed, native


def test_complete_predecessor_evidence_allows_next_independent_stage():
    module.validate_predecessors(*evidence())


@pytest.mark.parametrize('failure', ['fixed_incomplete', 'grasp_loss', 'stop_missing', 'native_waiting',
    'native_failed', 'native_output', 'native_grasp', 'native_state', 'unknown_loss', 'native_goal', 'wrong_scope'])
def test_incomplete_or_failed_predecessor_cannot_launch_physics(failure):
    sweep, fixed, native = evidence()
    if failure == 'fixed_incomplete':
        fixed['completed_goals'] = 1
    elif failure == 'grasp_loss':
        fixed['grasp_losses'] = [dict(robot='follower', state='detached')]
    elif failure == 'stop_missing':
        fixed.pop('stop_and_hold_confirmed')
    elif failure == 'native_waiting':
        native['state'] = 'waiting_for_completed_fixed_90'
    elif failure == 'native_failed':
        native['result']['passed'] = False
    elif failure == 'native_output':
        native['result']['both_outputs_zero_after_stop'] = False
    elif failure == 'native_grasp':
        native['result']['both_grasps_retained_after_stop'] = False
    elif failure == 'native_state':
        native['result']['states']['follower']['state'] = 'TRANSPORT'
    elif failure == 'unknown_loss':
        del native['result']['grasp_loss_events']
    elif failure == 'native_goal':
        native['result']['states']['leader']['accepted_goals'] = 1
    elif failure == 'wrong_scope':
        native['result']['autonomous_yaw_qualification_issued'] = True
    with pytest.raises(ValueError):
        module.validate_predecessors(sweep, fixed, native)


def test_dead_owner_cannot_be_mistaken_for_running_owner():
    assert module.identity(999999999) is None


@pytest.mark.parametrize('options', [[], ['--resume-completed-predecessors', '--fixed-owner-pid', '1'],
    ['--fixed-owner-pid', '1']])
def test_resume_and_live_owner_modes_cannot_be_mixed_or_left_implicit(monkeypatch, options):
    forbidden = Mock(side_effect=AssertionError('physics forbidden before predecessor validation'))
    monkeypatch.setattr(module.subprocess, 'call', forbidden)
    monkeypatch.setattr(sys, 'argv', ['queue.py', '--attempt', '99999999', *options])
    with pytest.raises(SystemExit) as exc:
        module.main()
    assert exc.value.code == 2 and not forbidden.called
