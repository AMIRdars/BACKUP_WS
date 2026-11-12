"""Check the actual launch entry's rejection and completion paths without motion."""
import importlib.util
import ast
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import EmitEvent, ExecuteProcess
from cp9_trial_progress import TrialProgress


ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / 'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_contact_transport.launch.py'
spec = importlib.util.spec_from_file_location('cp9_launch_entry', SOURCE)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def context(tmp_path, **overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'headless': 'true', 'forward_distance_m': '.0001',
        'translation_velocity_m_s': '.00005', 'rotation_angle_deg': '0',
        'world_y_distance_m': '0', 'record_data': 'true',
        'grip_maintenance': 'false',
        'workspace_directory': str(ROOT), 'run_directory': '',
        'metrology_directory': '', 'output_directory': str(tmp_path / 'run'),
        'ros_domain_id': '93', 'ign_partition': '',
        **overrides,
    })
    return ctx


@pytest.mark.parametrize(('name', 'bad'), [
    ('forward_distance_m', '1.0001'), ('forward_distance_m', '0'),
    ('forward_distance_m', '-.0001'), ('forward_distance_m', 'nan'),
    ('translation_velocity_m_s', '.10'), ('translation_velocity_m_s', '.00501'),
    ('translation_velocity_m_s', '0'),
    ('translation_velocity_m_s', 'inf'), ('rotation_angle_deg', '90'),
    ('world_y_distance_m', '1'), ('rotation_angle_deg', 'nan'),
    ('ros_domain_id', '233'), ('ign_partition', 'unisolated'),
    ('translation_acceleration_m_s2', '.025'),
    ('angular_velocity_deg_s', '1'), ('angular_acceleration_deg_s2', '.25'),
])
def test_unsafe_or_unvalidated_arguments_fail_before_starting_any_process(tmp_path, name, bad):
    with pytest.raises(ValueError):
        entry.setup(context(tmp_path, **{name: bad}))


def test_existing_results_are_never_reused(tmp_path):
    (tmp_path / 'run').mkdir()
    with pytest.raises(ValueError, match='already exists'):
        entry.setup(context(tmp_path))


def test_missing_calibration_prevents_any_process_start(tmp_path):
    with pytest.raises(ValueError, match='calibration.json'):
        entry.setup(context(tmp_path, metrology_directory=str(tmp_path)))


def test_headless_uses_one_external_trial_with_workspace_cwd(tmp_path):
    ctx = context(tmp_path)
    actions = entry.setup(ctx)
    processes = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(processes) == 1
    process = processes[0]
    process.prepare(ctx)
    assert process.process_details['cwd'] == str(ROOT)
    assert '--distance' in process.process_details['cmd']
    assert process.process_details['env']['ROS_LOCALHOST_ONLY'] == '1'
    assert process.process_details['env']['IGN_PARTITION'].startswith('amir_cp9_')
    assert not (tmp_path / 'run').exists()


def test_one_metre_is_forwarded_without_clamping_or_converting_to_millimetres(tmp_path):
    ctx = context(tmp_path, forward_distance_m='1.0')
    processes = [a for a in entry.setup(ctx) if isinstance(a, ExecuteProcess)]
    processes[0].prepare(ctx)
    command = processes[0].process_details['cmd']
    assert command[command.index('--distance') + 1] == '1.0'
    assert command[command.index('--speed') + 1] == '5e-05'


@pytest.mark.parametrize('enabled', [False, True])
def test_grip_maintenance_is_explicit_opt_in_and_does_not_change_the_goal(tmp_path, enabled):
    ctx = context(tmp_path, grip_maintenance=str(enabled).lower(), forward_distance_m='1.0')
    process = next(a for a in entry.setup(ctx) if isinstance(a, ExecuteProcess))
    process.prepare(ctx)
    cmd = process.process_details['cmd']
    assert ('--grip-maintenance' in cmd) == enabled
    assert cmd[cmd.index('--distance') + 1] == '1.0'


def test_gui_is_only_a_client_in_same_isolated_partition(tmp_path):
    ctx = context(tmp_path, headless='false', ign_partition='amir_cp9_launch_test')
    actions = entry.setup(ctx)
    processes = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(processes) == 2
    for process in processes:
        process.prepare(ctx)
        assert process.process_details['env']['IGN_PARTITION'] == 'amir_cp9_launch_test'
    gui_cmd = processes[0].process_details['cmd']
    assert gui_cmd[:4] == ['ign', 'gazebo', '-g', '--gui-config']
    assert Path(gui_cmd[4]).is_file()
    assert gui_cmd[5:] == ['--force-version', '6']


def test_progress_shows_phase_changes_immediately_and_throttles_repeated_messages(tmp_path, capsys):
    clock = [0.]
    progress = TrialProgress(tmp_path, distance_m=1., clock=lambda: clock[0])
    def update(phase):
        progress.update(phase=phase, sim_time=26.6, preparation={}, encoders={})
    update('startup')
    clock[0] = 1.
    update('startup')
    update('closing')
    events = [json.loads(line) for line in (tmp_path/'progress.jsonl').read_text().splitlines()]
    assert [e['phase'] for e in events] == ['startup', 'closing']
    assert 'まだ搬送しません' in capsys.readouterr().out
    clock[0] = 6.
    update('closing')
    assert len((tmp_path/'progress.jsonl').read_text().splitlines()) == 3


def test_displayed_distance_uses_each_encoder_origin_and_is_preserved_during_settling(tmp_path):
    progress = TrialProgress(tmp_path, distance_m=1.)
    progress.update(phase='motion', sim_time=50., preparation={},
                    encoders={'leader': .001, 'follower': -.002})
    progress.update(phase='settle', sim_time=270., preparation={},
                    encoders={'leader': 1.001, 'follower': -1.002})
    event = json.loads((tmp_path/'progress.jsonl').read_text().splitlines()[-1])
    assert event['encoder_displacement_m'] == pytest.approx({'leader': 1., 'follower': 1.})
    assert event['goal_distance_m'] == 1.


@pytest.mark.parametrize(('code', 'passed'), [(1, True), (0, False), (1, False)])
def test_failure_never_reports_success_or_leaves_launch_running(tmp_path, code, passed):
    (tmp_path / 'result.json').write_text(json.dumps({'passed': passed, 'error': 'test refusal'}))
    with pytest.raises(RuntimeError, match='FAILED'):
        entry.trial_finished(SimpleNamespace(returncode=code), LaunchContext(), output=tmp_path)


def test_missing_result_is_failure(tmp_path):
    with pytest.raises(RuntimeError, match='no readable result'):
        entry.trial_finished(SimpleNamespace(returncode=0), LaunchContext(), output=tmp_path)


def test_passing_trial_shuts_down_gui_and_launch(tmp_path):
    (tmp_path / 'result.json').write_text('{"passed":true}')
    actions = entry.trial_finished(SimpleNamespace(returncode=0), LaunchContext(), output=tmp_path)
    assert any(isinstance(action, EmitEvent) for action in actions)


def test_duplicate_terminal_and_forwarded_sigint_cannot_abort_cleanup():
    # Run the actual trial handler in an isolated Python process and deliver
    # two real signals: the second must not escape the first interruption.
    tree = ast.parse((ROOT / 'tools/amir_cp9_transport_trial.py').read_text())
    handler = next(n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == 'interrupt_once')
    script = 'import os, signal\n' + ast.unparse(handler) + '''
signal.signal(signal.SIGINT, interrupt_once)
try:
    os.kill(os.getpid(), signal.SIGINT)
except KeyboardInterrupt:
    pass
else:
    raise RuntimeError('first interrupt was lost')
os.kill(os.getpid(), signal.SIGINT)
print('cleanup continues after duplicate SIGINT')
'''
    result = subprocess.run(['/usr/bin/python3', '-c', script],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'cleanup continues' in result.stdout
