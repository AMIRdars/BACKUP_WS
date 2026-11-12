"""No O2 or fault trial starts from incomplete native predecessor evidence."""
import copy
import importlib.util
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('native_o2_queue_test',ROOT/'audits/rotation_native_o2_20261106/queue.py')
queue=importlib.util.module_from_spec(spec);spec.loader.exec_module(queue)


def o2_result(inject=False):
    return dict(passed=True,native_o2_verified=True,both_outputs_zero_after_stop=True,
        both_grasps_retained_after_stop=True,condition='native_O2_local_compensation',
        observer_stop_injected=inject,autonomous_follower_yaw_enabled=False,grasp_loss_events=[],
        states={r:dict(state='SAFE_STOP',accepted_goals=0,completed_goals=0) for r in ('leader','follower')},
        native_o2_measurement={'observer_stop_detected' if inject else 'positive_negative_pulse_completed':True})


@pytest.mark.parametrize('inject',[False,True])
def test_completed_normal_and_expected_fault_require_separate_proofs(inject):
    queue.validate_o2(o2_result(inject),observer_stop=inject)


@pytest.mark.parametrize('change',['grasp','output','scope','goal','injection','normal_proof','fault_proof','state'])
def test_incomplete_o2_cannot_advance_to_another_rotation(change):
    inject=change=='fault_proof';data=o2_result(inject)
    if change=='grasp':data['both_grasps_retained_after_stop']=False
    elif change=='output':data['both_outputs_zero_after_stop']=False
    elif change=='scope':data['condition']='fixed_O2'
    elif change=='goal':data['states']['leader']['accepted_goals']=1
    elif change=='injection':data['observer_stop_injected']=True
    elif change=='normal_proof':data['native_o2_measurement']={}
    elif change=='fault_proof':data['native_o2_measurement']={}
    elif change=='state':data['states']['follower']['state']='TRANSPORT'
    with pytest.raises(ValueError):queue.validate_o2(data,observer_stop=inject)


def test_missing_native_passive_certificate_cannot_start_o2():
    with pytest.raises(ValueError):queue.validate_passive(dict(state='native_passive_complete',exit_code=0,result=dict(passed=True)))


def test_native_o2_cli_cannot_start_without_predecessor_files(tmp_path):
    import subprocess,sys
    target=tmp_path/'forbidden'
    args=[sys.executable,str(ROOT/'tools/amir_cp8r3_native_trial.py'),'--run-directory',str(tmp_path/'fixture'),
        '--output',str(target),'--native-metrology-directory',str(tmp_path/'source'),
        '--wrench-calibration-file',str(tmp_path/'calibration'),'--native-o2-audit',
        '--delta-deg','.005','--duration-s','75','--arm-yaw-metric','.4',
        '--force-memory-gain','0','--force-velocity-gain','0','--xy-gain','0','--yaw-gain','0']
    result=subprocess.run(args,capture_output=True,text=True)
    assert result.returncode!=0 and 'native O2 requires native Gate-0/passive' in result.stderr
    assert not target.exists()
