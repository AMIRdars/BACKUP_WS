import importlib.util
from pathlib import Path

import pytest

PATH=Path(__file__).resolve().parents[1]/'audits/rotation_native_enable_20261106/queue_step_metrology.py'
spec=importlib.util.spec_from_file_location('step_metrology_queue',PATH)
queue=importlib.util.module_from_spec(spec);spec.loader.exec_module(queue)


def evidence():
    return dict(condition='native_static_yaw_gate0',passed=True,fixture_unchanged=True,
        both_outputs_zero_after_stop=True,both_grasps_retained_after_stop=True,
        grasp_loss_events=[],autonomous_yaw_qualification_issued=False,native_gate0=dict(gate0_static_passed=True),
        states={r:dict(state='SAFE_STOP',accepted_goals=0,completed_goals=0) for r in ('leader','follower')})


def test_only_sensor_remeasurement_accepts_failed_baseline_load_with_verified_hold():
    candidate=evidence();queue.validate_static(candidate,must_pass=True)
    candidate['passed']=False;queue.validate_static(candidate,must_pass=False)
    with pytest.raises(ValueError):queue.validate_static(candidate,must_pass=True)


@pytest.mark.parametrize('failure',['hold','stop','fixture','goals','loss','scope','static_missing'])
def test_incomplete_candidate_evidence_does_not_start_sensor_identification(failure):
    result=evidence()
    if failure=='hold':result['both_grasps_retained_after_stop']=False
    elif failure=='stop':result['both_outputs_zero_after_stop']=False
    elif failure=='fixture':result['fixture_unchanged']=False
    elif failure=='goals':result['states']['leader']['accepted_goals']=1
    elif failure=='loss':result['grasp_loss_events']=[{}]
    elif failure=='scope':result['condition']='fixed_grasp'
    else:result['native_gate0']={}
    with pytest.raises(ValueError):queue.validate_static(result,must_pass=True)
