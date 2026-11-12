"""Initial native transport is gated by real-shaped, unit-only O1/O2 proofs."""
import csv
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from amir_rotation_native_follow import configure_follow,require_o2_pair,NativeFollowRecorder,execute_follow,evaluation_diagnostics
from test_rotation_native_o2 import configure,SOURCE

ROOT=Path(__file__).resolve().parents[1]
FIXED=ROOT/'audits/rotation_equivariant_expansion_20261105'


def follow_case(tmp_path):
    passive,_,certificate,output,follower=configure(tmp_path)
    passive_hash=hashlib.sha256(passive.read_bytes()).hexdigest()
    files=[]
    for inject,name in ((False,'normal'),(True,'fault')):
        directory=tmp_path/name;directory.mkdir();path=directory/'result.json'
        report=dict(condition='native_O2_local_compensation',passed=True,native_o2_verified=True,
            both_outputs_zero_after_stop=True,both_grasps_retained_after_stop=True,fixture_unchanged=True,
            leader_input_boundary_passed=True,follower_input_boundary_passed=True,
            observer_stop_injected=inject,grasp_loss_events=[],autonomous_follower_yaw_enabled=False,
            states={r:dict(state='SAFE_STOP',accepted_goals=0,completed_goals=0) for r in ('leader','follower')},
            native_o2_reference=dict(native_passive_result_sha256=passive_hash,reduced_yaw_speed_cap_rad_s=1e-6),
            native_o2_measurement={'observer_stop_detected' if inject else 'positive_negative_pulse_completed':True},
            native_o2_statistics=dict(local_fault='',raw_ft_events=32,moving_samples=20,limited_samples=10))
        path.write_text(json.dumps(report));files.append(path)
    fields=['phase','leader_output_vx','leader_output_vy','leader_output_wz','follower_output_wz']
    with (files[0].parent/'timeseries.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(fields)
        for speed in (5e-7,-4e-7):writer.writerow(['native_o2_probe',0.,0.,0.,speed])
    args=SimpleNamespace(native_passive_result=passive,native_o2_result=files[0],native_o2_fault_result=files[1],
        delta_deg=.005,duration_s=75.,xy_gain=.5,yaw_gain=2.,
        native_allocation_policy=FIXED/'whole_body_allocation_policy.yaml',
        passive_leader_limits_file=FIXED/'wb_15_03/leader_policy.yaml')
    return args,output,follower,certificate


def test_native_follow_keeps_observer_and_allocator_exclusive_and_only_reduces_rate(tmp_path):
    args,output,follower,certificate=follow_case(tmp_path)
    plan=configure_follow(args,SOURCE,output,output/'rotation_policy.yaml',follower)
    p=next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters']
    assert not p['passive_yaw_calibration'] and p['rotation_observation_required']
    assert p['allocation_task_compliance_budget'] and p['allocation_base_regularization']==3e-5
    assert p['base_gains']==[.5,.5,2.] and p['base_force_velocity_gains']==[0.]*3
    assert p['base_velocity_limits'][2]<=4e-7 and plan['leader_angular_speed_cap_rad_s']<=4e-7
    assert not any(k.startswith('probe_') for k in p)
    assert plan['goal_duration_s']>=1.875*math.radians(.010)/plan['leader_angular_speed_cap_rad_s']
    saved=json.loads((output/'qualification.json').read_text())
    for key in certificate:
        if key!='urdf_sha256':assert saved[key]==certificate[key]


@pytest.mark.parametrize('change',['missing_fault','different_passive','raw_ft_fault','one_direction','leader_moved'])
def test_native_follow_cannot_use_unrelated_or_failed_o2(tmp_path,change):
    args,output,follower,_=follow_case(tmp_path)
    if change in ('missing_fault','different_passive','raw_ft_fault'):
        path=args.native_o2_fault_result;report=json.loads(path.read_text())
        if change=='missing_fault':report['native_o2_measurement']={}
        elif change=='different_passive':report['native_o2_reference']['native_passive_result_sha256']='changed'
        else:report['native_o2_statistics']['local_fault']='raw wrench exceeded'
        path.write_text(json.dumps(report))
    else:
        p=args.native_o2_result.parent/'timeseries.csv';text=p.read_text()
        if change=='one_direction':text=text.replace('-4e-07','0.0')
        else:text=text.replace('native_o2_probe,0.0,0.0,0.0','native_o2_probe,0.0,0.0,0.001')
        p.write_text(text)
    with pytest.raises(ValueError):require_o2_pair(args.native_o2_result,args.native_o2_fault_result,args.native_passive_result)


def test_native_follow_cannot_expand_angle_before_any_policy_changes(tmp_path):
    args,output,follower,_=follow_case(tmp_path);before=follower.read_bytes();args.delta_deg=.015
    with pytest.raises(ValueError,match='initial native follow'):configure_follow(args,SOURCE,output,output/'rotation_policy.yaml',follower)
    assert follower.read_bytes()==before


def record(tmp_path):
    return NativeFollowRecorder(tmp_path,['native_follow_positive'],force_limit_N=.5,torque_limit_Nm=.05)


def own_debug():
    return dict(sim_time=1.,state='TRANSPORT',enabled=True,measured_error_xy_yaw=[1e-5,2e-5,3e-5],
        base_twist=[1e-7,-2e-7,3e-7],velocity=[0.]*5+[1e-7,-2e-7,3e-7],
        consumed_observation=dict(publication_sequence=7,source_age_ns=12_000_000),
        tau_z_at_tcp_Nm=.001,grasp_utilization=.1)


def test_native_follow_has_one_eight_variable_command_and_no_latest_snapshot_substitution(tmp_path):
    r=record(tmp_path)
    try:
        r.observe(dict(publication_sequence=8,yaw_valid=True));r.record(own_debug())
        result=r.summary();assert result['minimum_allocation_variables']==8 and result['maximum_allocation_base_output_difference']==0.
        assert result['missing_consumed_snapshot_rows']==1
        line=json.loads((tmp_path/'native_follow_local.jsonl').read_text())
        assert line['consumed_observation'] is None and not line['exact_consumed_snapshot_match']
        assert not r.fault
    finally:r.close()


def test_a_second_base_command_outside_the_allocator_is_rejected(tmp_path):
    r=record(tmp_path)
    try:
        row=own_debug();row['base_twist'][2]=1e-5;r.record(row)
        assert 'second base command' in r.fault
    finally:r.close()


def test_following_requires_all_eight_allocator_variables(tmp_path):
    r=record(tmp_path)
    try:
        row=own_debug();row['velocity']=[0.]*5;r.record(row)
        assert r.fault and r.minimum_variables is None
    finally:r.close()


def test_follow_sends_only_two_leader_goals_after_loaded_qualification(tmp_path):
    phase=['tare'];states={'leader':dict(accepted_goals=0,completed_goals=0),'follower':dict(accepted_goals=0)}
    recorder=SimpleNamespace(latest={'reference_sequence':0},debug=dict(enabled=True,velocity=[0.]*8),
        fault='',raw_events=32,minimum_variables=8)
    sent=[];services=[]
    def service(name):
        services.append(name)
        if name.endswith('latch_reference'):recorder.latest.update(reference_sequence=1,yaw_valid=True)
    def publish(msg):
        sent.append(msg.data);states['leader'].update(accepted_goals=len(sent),completed_goals=len(sent))
    def wait(predicate,timeout,check):assert predicate()
    result=execute_follow(SimpleNamespace(delta_deg=.005),dict(goal_duration_s=1000.),service=service,
        settle=lambda t:None,wait=wait,phase=phase,goal=SimpleNamespace(publish=publish),states=states,
        recorder=recorder,ready=dict(value=True),source_hashes={})
    assert result['positive_negative_programs_completed'] and result['angle_accuracy_is_diagnostic']
    assert sent==pytest.approx([math.radians(.005),math.radians(-.010)])
    assert services==['/follower/rotation/latch_reference','/follower/admittance/enable']


def test_gt_metrics_are_only_offline_diagnostics_and_do_not_claim_center_fixed():
    row=dict(object_center_GT_x=0.,object_center_GT_y=0.,object_center_GT_z=.6,object_yaw_GT_rad=0.)
    rows=[dict(row,phase='native_follow_baseline'),dict(row,phase='native_follow_positive',object_center_GT_x=.002,object_yaw_GT_rad=.00009)]
    report=evaluation_diagnostics(rows)
    assert report['evaluation_only'] and not report['physical_center_fix_achieved']
    assert report['maximum_object_center_drift_m']==pytest.approx(.002)
