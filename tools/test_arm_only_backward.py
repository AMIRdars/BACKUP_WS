"""Real-URDF reach rejection and public-launch backward/ownership boundaries."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions

from amir_compliance_control.arm_reach import backward_reach_bound, translation_progress
import arm_only_micro_runtime as runtime


ROOT = Path(__file__).resolve().parents[1]
SEED = ROOT/runtime.SEED
PUBLIC = ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py'
spec = importlib.util.spec_from_file_location('public_backward', PUBLIC)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def context(tmp_path, **overrides):
    ctx = LaunchContext()
    for action in entry.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations.update(workspace_directory=str(ROOT), headless='true',
                                     output_directory=str(tmp_path/'run'),arm_only_direction='backward',
                                     arm_only_distance_m='.009',keep_gui_open='false',start_delay_s='0',arm_only_trial='micro')
    ctx.launch_configurations.update(overrides)
    return ctx


def test_half_metre_is_impossible_even_with_unrestricted_height_and_orientation():
    evidence = backward_reach_bound(SEED.read_text(), .5)
    assert evidence['geometrically_impossible']
    shoulder = next(s for s in evidence['spheres'] if s['joint'] == 'Joint_2')
    assert shoulder['link_length_sum_m'] == pytest.approx(.740)
    assert shoulder['target_horizontal_distance_m'] > .976
    assert .26 < evidence['backward_distance_upper_bound_m'] < .27
    archive = ROOT/'.simulation_runtimes/cp13_round_trip/audits/whole_body_compliance_20261019/cp12_speed_release/generated_gui20/follower.urdf'
    old = backward_reach_bound(archive.read_text(), .5)
    assert old['home_joint_rad'] == evidence['home_joint_rad']
    assert old['home_tcp_m'] == evidence['home_tcp_m']


def test_bound_is_necessary_only_and_includes_tcp_offset():
    xml = SEED.read_text()
    bound = backward_reach_bound(xml, .5)['backward_distance_upper_bound_m']
    assert not backward_reach_bound(xml, bound)['geometrically_impossible']
    assert backward_reach_bound(xml, bound+1e-6)['geometrically_impossible']
    assert not backward_reach_bound(xml, .009)['geometrically_impossible']


def test_moving_shoulder_cannot_be_used_as_a_fixed_sphere_center():
    root = ET.fromstring(SEED.read_text())
    root.find("joint[@name='Joint_2']/origin").set('xyz', '.05 0 .097')
    result = backward_reach_bound(ET.tostring(root, encoding='unicode'), .5)
    assert 'Joint_2' not in [s['joint'] for s in result['spheres']]


@pytest.mark.parametrize('value', [0., -.5, float('nan'), float('inf')])
def test_invalid_distance_cannot_be_certified(value):
    with pytest.raises(ValueError):
        backward_reach_bound(SEED.read_text(), value)


def test_backward_qualification_requires_signed_leader_and_payload_translation():
    assert translation_progress(.009, 'backward', -.009, [-.009, 0., 0.])
    assert translation_progress(.009, 'forward', .009, [.009, 0., 0.])
    assert not translation_progress(.009, 'backward', .009, [-.009, 0., 0.])
    assert not translation_progress(.009, 'backward', -.009, [.009, 0., 0.])
    assert not translation_progress(.009, 'backward', -.009, [0., .1, .1])


def test_public_refuses_half_metre_before_start_and_saves_reason(tmp_path):
    ctx = context(tmp_path,backward_distance_m='.5')
    assert ctx.launch_configurations['control_mode'] == 'ARM_ONLY'
    with pytest.raises(ValueError, match='geometrically impossible'):
        entry.setup(ctx)
    result = json.loads((tmp_path/'run/result.json').read_text())
    assert result['executed'] is False and result['passed'] is False
    assert (tmp_path/'run/REPORT.md').is_file()


@pytest.mark.parametrize(('distance', 'speed', 'reason'), [('.02', '.003', 'only the'),
    ('.009', '.1', 'requires finite speed'), ('.009', 'nan', 'requires finite speed')])
def test_no_unmeasured_envelope_is_silently_enabled(tmp_path, distance, speed, reason):
    with pytest.raises(ValueError, match=reason):
        entry.setup(context(tmp_path, backward_distance_m=distance, translation_velocity_m_s=speed))
    assert json.loads((tmp_path/'run/result.json').read_text())['executed'] is False


@pytest.mark.parametrize('headless', ['true', 'false'])
def test_micro_backward_uses_only_strict_local_stack_with_child_only_partition(tmp_path, monkeypatch, headless):
    monkeypatch.setenv('IGN_PARTITION', 'parent_must_survive')
    ctx = context(tmp_path, backward_distance_m='.009', headless=headless)
    actions = entry.setup(ctx)
    processes = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(processes) == 1
    process = processes[0]
    argv = [perform_substitutions(ctx, p) for p in process.cmd]
    assert argv[2] == str(ROOT/'tools/amir_af_trial.py')
    assert argv[argv.index('--direction')+1] == 'backward'
    assert argv[argv.index('--distance')+1] == '0.009'
    assert ('--gui' in argv) == (headless == 'false')
    env = {perform_substitutions(ctx, k): perform_substitutions(ctx, v) for k, v in process.additional_env}
    assert env['IGN_PARTITION'].startswith('amir_af_p0_')
    assert env['IGN_PARTITION'] != 'parent_must_survive'
    assert ('__NV_PRIME_RENDER_OFFLOAD' in env) == (headless == 'false')
    assert not any('WHOLE_BODY' in a or 'round_trip_trial' in a for a in argv)


def test_micro_output_cannot_be_overwritten(tmp_path):
    ctx = context(tmp_path, backward_distance_m='.009')
    entry.setup(ctx)
    with pytest.raises(ValueError, match='already exists'):
        entry.setup(ctx)


def test_public_defaults_are_forward_micro_and_persistent_gui(tmp_path):
    ctx=LaunchContext()
    for action in entry.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(ctx)
    ctx.launch_configurations.update(workspace_directory=str(ROOT),output_directory=str(tmp_path/'new'),arm_only_trial='micro')
    actions=entry.setup(ctx)
    process=next(a for a in actions if isinstance(a,ExecuteProcess))
    argv=[perform_substitutions(ctx,p) for p in process.cmd]
    assert argv[argv.index('--direction')+1]=='forward'
    assert argv[argv.index('--distance')+1]=='0.0093'
    assert argv[argv.index('--speed')+1]=='0.005'
    assert '--gui' in argv and '--keep-gui-open' in argv
    assert argv[argv.index('--start-delay-s')+1]=='10.0'
    assert not any('base_recentering' in arg or 'whole_body' in arg for arg in argv)
