"""Verify isolation and fail-closed selection of the recovered CP-13 release."""
from pathlib import Path
import os
import subprocess

import pytest

import round_trip_runtime as runtime


ROOT = Path(__file__).resolve().parents[1]


def test_qualified_archive_and_installed_imports_agree():
    _, folder = runtime.validate(ROOT)
    env = dict(os.environ, **runtime.environment(folder, os.environ))
    result = subprocess.run(['/usr/bin/python3', '-c',
        'import amir_compliance_control.whole_body as m; '
        'from ament_index_python.packages import get_package_prefix as p; '
        'print(m.__file__);print(p("amir_compliance_control"));'
        'print(p("cooperative_transport_bringup"))'],
        env=env, check=True, text=True, capture_output=True)
    assert all(str(folder/'install') in line for line in result.stdout.splitlines())


@pytest.mark.parametrize('part', ['src/humbleble_ws/amir_compliance_control/amir_compliance_control/whole_body.py',
    'install/amir_compliance_control/lib/python3.10/site-packages/amir_compliance_control/whole_body.py',
    'tools/amir_cp9_grasp_preparation.py'])
def test_changed_source_or_installed_controller_is_rejected(monkeypatch, part):
    original = runtime.sha
    target = ROOT/runtime.RUNTIME/part
    monkeypatch.setattr(runtime, 'sha', lambda path: 'changed' if path == target else original(path))
    with pytest.raises(ValueError, match='changed'):
        runtime.validate(ROOT)


def test_installed_safety_launch_resolves_runtime_root():
    _, folder = runtime.validate(ROOT)
    launch = folder/'install/cooperative_transport_bringup/share/cooperative_transport_bringup/launch/compliance_round_trip_safety.launch.py'
    assert launch.resolve().parents[4] == folder
    assert (launch.resolve().parents[4]/'tools/round_trip_safety_gate.py').is_file()


def test_changed_controller_yaml_at_historical_absolute_path_is_rejected(monkeypatch):
    original = runtime.sha
    target = ROOT/'audits/whole_body_compliance_20261019/cp9_transport/generated/leader_controllers.yaml'
    monkeypatch.setattr(runtime, 'sha', lambda path: 'changed' if path == target else original(path))
    with pytest.raises(ValueError, match='shared dependency changed'):
        runtime.validate(ROOT)


def test_cpu_policy_separates_gui_and_inherits_to_children():
    groups = runtime.cpu_groups()
    if groups is None:
        pytest.skip('homogeneous CPU host')
    fast, background = groups
    parent = os.sched_getaffinity(0)
    assert fast.isdisjoint(background)
    assert fast | background <= parent
    child = subprocess.Popen(['/usr/bin/python3', '-c',
        'input();import os;print(sorted(os.sched_getaffinity(0)))'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        runtime.assign_cpu(child.pid, fast)
        output, _ = child.communicate('\n', timeout=5.)
        assert output.strip() == str(sorted(fast))
        assert os.sched_getaffinity(0) == parent
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()


@pytest.mark.parametrize('argument', ['forward_distance_m:=2.0', 'translation_velocity_m_s:=1.0',
                                     'rotation_angle_deg:=90.0'])
def test_unsupported_motion_is_rejected_before_any_simulation(argument):
    result = subprocess.run(['ros2', 'launch', 'cooperative_transport_bringup',
        'compliance_round_trip.launch.py', 'control_mode:=WHOLE_BODY',
        'translation_velocity_m_s:=0.1', 'headless:=true', argument],
        cwd=ROOT, text=True, capture_output=True, timeout=30.)
    output = result.stdout+result.stderr
    assert result.returncode != 0
    assert 'process started with pid' not in output
    assert 'supports exactly' in output or 'route and acceleration are fixed' in output
