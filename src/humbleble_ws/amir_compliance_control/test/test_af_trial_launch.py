"""Wrapper ownership, requested experiment settings and failure propagation."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import os
import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

ROOT=Path(__file__).resolve().parents[4]
FILE=ROOT/'src/humbleble_ws/amir_compliance_control/launch/arm_only_af_trial.launch.py'
spec=importlib.util.spec_from_file_location('af_trial_launch',FILE)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def context(**overrides):
    c=LaunchContext()
    c.launch_configurations.update(workspace=str(ROOT),ros_domain_id='70',output_directory='',gui='false')
    c.launch_configurations.update(overrides)
    return c


def process(c):
    return next(a for a in module.setup(c) if isinstance(a,ExecuteProcess))


def test_exact_requested_profile_without_shell_or_second_stack():
    c=context();p=process(c)
    argv=[perform_substitutions(c,t) for t in p.cmd]
    assert argv[2]==str(ROOT/'tools/amir_af_trial.py')
    expected={'--stage':'AF-2','--profile':'pose_hold_micro','--virtual-mass-kg':'40',
        '--virtual-damping-ns-m':'80','--virtual-stiffness-n-m':'5',
        '--arm-command-reference':'servo_trajectory','--arm-horizon':'.08',
        '--wrench-cutoff-hz':'8','--leader-reference-acceleration':'.0012',
        '--distance':'.009','--speed':'.003','--require-speed-fraction':'.98'}
    assert all(argv[argv.index(k)+1]==v for k,v in expected.items())
    assert '--control-on-wrench' in argv and '--wrench-on-measurement' in argv
    assert module.default_workspace()==str(ROOT)
    assert '--gui' not in argv


def test_gui_option_only_enables_the_existing_scene_display():
    c=context(gui='true');p=process(c)
    argv=[perform_substitutions(c,t) for t in p.cmd]
    assert '--gui' in argv
    assert argv[argv.index('--speed')+1]=='.003'
    assert argv[argv.index('--distance')+1]=='.009'
    with pytest.raises(ValueError,match='gui must be'):module.setup(context(gui='invalid'))


def test_child_partition_is_unique_and_parent_environment_is_untouched(monkeypatch):
    monkeypatch.setenv('IGN_PARTITION','existing_parent_partition')
    c=context()
    def env(p):return {perform_substitutions(c,k):perform_substitutions(c,v) for k,v in p.additional_env}
    a,b=process(c),process(c);ea,eb=env(a),env(b)
    assert ea['IGN_PARTITION']!=eb['IGN_PARTITION']
    assert ea['IGN_PARTITION']!='existing_parent_partition'
    assert ea['ROS_DOMAIN_ID']=='70' and ea['ROS_LOCALHOST_ONLY']=='1'
    assert os.environ['IGN_PARTITION']=='existing_parent_partition'
    av=[perform_substitutions(c,t) for t in a.cmd];bv=[perform_substitutions(c,t) for t in b.cmd]
    assert av[av.index('--output')+1]!=bv[bv.index('--output')+1]


@pytest.mark.parametrize('domain',['-1','233','not-a-number'])
def test_invalid_domain_cannot_start_trial(domain):
    with pytest.raises(ValueError,match='ros_domain_id'):module.setup(context(ros_domain_id=domain))


def test_existing_output_cannot_be_overwritten(tmp_path):
    with pytest.raises(ValueError,match='already exists'):module.setup(context(output_directory=str(tmp_path)))


def test_missing_workspace_fails_before_process_start(tmp_path):
    with pytest.raises(ValueError,match='dependency missing'):module.setup(context(workspace=str(tmp_path)))


def test_failed_trial_is_not_reported_as_success():
    with pytest.raises(RuntimeError,match='trial failed'):module.trial_exit(SimpleNamespace(returncode=1),context())
    assert len(module.trial_exit(SimpleNamespace(returncode=0),context()))==1
    c=context();c._set_is_shutdown(True)
    assert module.trial_exit(SimpleNamespace(returncode=1),c)==[]
