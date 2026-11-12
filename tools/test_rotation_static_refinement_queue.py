import importlib.util
from pathlib import Path

import pytest

PATH=Path(__file__).resolve().parents[1]/'audits/rotation_native_enable_20261106/queue_static_refinement.py'
spec=importlib.util.spec_from_file_location('static_resolution_queue',PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def evidence():
    result=dict(condition='native_static_yaw_gate0',passed=False,both_outputs_zero_after_stop=True,
        both_grasps_retained_after_stop=True,fixture_unchanged=True,grasp_loss_events=[],autonomous_yaw_qualification_issued=False,
        states={r:dict(state='SAFE_STOP',accepted_goals=0,completed_goals=0) for r in ('leader','follower')})
    return result,dict(partition='own',domain='214'),dict(IGN_PARTITION='own',ROS_DOMAIN_ID='214')


def test_failed_load_measurement_with_verified_stop_allows_only_next_static_diagnostic():
    module.validate(*evidence())


@pytest.mark.parametrize('change',['stop','hold','loss','goal','state','domain','partition'])
def test_unverified_physical_stop_cannot_start_next_static_test(change):
    result,cleanup,command=evidence()
    if change=='stop':result['both_outputs_zero_after_stop']=False
    elif change=='hold':result['both_grasps_retained_after_stop']=False
    elif change=='loss':result['grasp_loss_events']=[{}]
    elif change=='goal':result['states']['leader']['accepted_goals']=1
    elif change=='state':result['states']['leader']['state']='TRANSPORT'
    elif change=='domain':cleanup['domain']='wrong'
    else:cleanup['partition']='wrong'
    with pytest.raises(ValueError):module.validate(result,cleanup,command)
