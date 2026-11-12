"""Native transport success is separate from static/O2 calibration success."""
import importlib.util
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('native_follow_queue_test',ROOT/'audits/rotation_native_follow_20261106/queue.py')
queue=importlib.util.module_from_spec(spec);spec.loader.exec_module(queue)


def evidence():
    return dict(condition='native_qualified_WHOLE_BODY_transport',passed=True,native_follow_verified=True,
        positive_negative_programs_completed=True,both_outputs_zero_after_stop=True,both_grasps_retained_after_stop=True,
        fixture_unchanged=True,leader_input_boundary_passed=True,follower_input_boundary_passed=True,
        autonomous_follower_yaw_enabled=True,grasp_loss_events=[],
        states={r:dict(state='SAFE_STOP',accepted_goals=n,completed_goals=n) for r,n in (('leader',2),('follower',0))},
        native_follow_statistics=dict(local_fault='',minimum_allocation_variables=8,raw_ft_events=32))


def test_only_actual_completed_native_transport_is_classified_as_success():queue.validate_follow(evidence())


@pytest.mark.parametrize('change',['hold','output','goal','follower_goal','scope','variables','local_fault','ready','raw_missing'])
def test_incomplete_or_unsafe_native_transport_is_not_promoted(change):
    d=evidence()
    if change=='hold':d['both_grasps_retained_after_stop']=False
    elif change=='output':d['both_outputs_zero_after_stop']=False
    elif change=='goal':d['states']['leader']['completed_goals']=1
    elif change=='follower_goal':d['states']['follower']['accepted_goals']=1
    elif change=='scope':d['condition']='native_O2_local_compensation'
    elif change=='variables':d['native_follow_statistics']['minimum_allocation_variables']=5
    elif change=='local_fault':d['native_follow_statistics']['local_fault']='yaw source stale'
    elif change=='ready':d['autonomous_follower_yaw_enabled']=False
    elif change=='raw_missing':d['native_follow_statistics']['raw_ft_events']=0
    with pytest.raises(ValueError):queue.validate_follow(d)


def test_missing_proofs_refuse_native_follow_before_output_or_ros_start(tmp_path):
    import subprocess,sys
    out=tmp_path/'forbidden'
    argv=[sys.executable,str(ROOT/'tools/amir_cp8r3_native_trial.py'),'--run-directory',str(tmp_path/'fixture'),
        '--output',str(out),'--native-metrology-directory',str(tmp_path/'source'),
        '--wrench-calibration-file',str(tmp_path/'calibration'),'--native-follow-audit',
        '--delta-deg','.005','--duration-s','75','--arm-yaw-metric','.4',
        '--force-memory-gain','0','--force-velocity-gain','0','--xy-gain','.5','--yaw-gain','2']
    r=subprocess.run(argv,capture_output=True,text=True)
    assert r.returncode!=0 and 'native follow requires actual Gate-0/passive/O2/fault' in r.stderr
    assert not out.exists()
