"""No wider experiment may be authorized by partial or failed O1 evidence."""
import pytest
from amir_rotation_integration_audit import require_completed_passive_range


def completed():
    return dict(passed=True,certificate_issued=True,positive_negative_programs_completed=True,
        stop_and_hold_confirmed=True,both_fixed_grasps_retained=True,delta_deg=.005)


def test_larger_comparison_uses_completed_measured_range_not_old_literal_cap():
    require_completed_passive_range(.005,completed())
    require_completed_passive_range(.001,completed())
    with pytest.raises(ValueError,match='exceeds'):require_completed_passive_range(.005001,completed())


@pytest.mark.parametrize('field',['passed','certificate_issued','positive_negative_programs_completed',
                                'stop_and_hold_confirmed','both_fixed_grasps_retained'])
def test_missing_or_failed_physical_gate_cannot_authorize_expansion(field):
    for missing in (False,None):
        evidence=completed();evidence[field]=missing
        with pytest.raises(ValueError,match='evidence required'):require_completed_passive_range(.001,evidence)


@pytest.mark.parametrize('angle',[float('nan'),float('inf'),-.001,0.,.100001])
def test_invalid_requested_or_evidence_angle_is_refused(angle):
    with pytest.raises(ValueError):require_completed_passive_range(angle,completed())
    evidence=completed();evidence['delta_deg']=angle
    with pytest.raises(ValueError):require_completed_passive_range(.001,evidence)


def test_passive_duration_preserves_old_profile_and_can_reduce_speed():
    from amir_rotation_integration_audit import passive_profile_duration
    assert passive_profile_duration(None,30.)==30.
    # 0.002 -> 0.005 degree with the same quintic peak angular speed.
    slower=30.*.005/.002
    assert passive_profile_duration(slower,30.)==75.


@pytest.mark.parametrize('duration',[float('nan'),float('inf'),0.,-1.,29.])
def test_passive_duration_cannot_accelerate_existing_profile(duration):
    from amir_rotation_integration_audit import passive_profile_duration
    with pytest.raises(ValueError):passive_profile_duration(duration,30.)


def fixture_setup(tmp_path,period=None):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    source=Path(__file__).resolve().parents[1]/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp1.launch.py'
    spec=importlib.util.spec_from_file_location('rotation_fixture_test',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext();context.launch_configurations.update(run_directory=str(tmp_path),headless='true',
        constraint_fixture='true',force_torque_system='true',calibration_fixture='true',joint_load_sensors='true',weld_cfm='.1')
    if period is not None:context.launch_configurations['grasp_state_publish_period_s']=str(period)
    module.setup(context)
    return tmp_path


def test_faster_actual_grasp_status_preserves_all_physics_and_legacy_default(tmp_path):
    import xml.etree.ElementTree as ET
    old=fixture_setup(tmp_path/'legacy');fast=fixture_setup(tmp_path/'fast',.02)
    baseline=ET.parse(old/'world.sdf').getroot();changed=ET.parse(fast/'world.sdf').getroot()
    name="world/plugin[@name='amir_follower::GraspHeartbeat']"
    assert baseline.find(name+'/state_publish_period_s') is None
    plugin=changed.find(name);assert float(plugin.findtext('state_publish_period_s'))==.02
    assert plugin.findtext('manage_joints')=='true'
    plugin.remove(plugin.find('state_publish_period_s'))
    assert ET.tostring(baseline).decode().replace(str(old),'RUN')==ET.tostring(changed).decode().replace(str(fast),'RUN')
    for robot in ['leader','follower']:
        assert (old/(robot+'.urdf')).read_text().replace(str(old),'RUN')==(fast/(robot+'.urdf')).read_text().replace(str(fast),'RUN')


@pytest.mark.parametrize('period',[0.,-.1,float('nan'),float('inf')])
def test_invalid_grasp_state_period_is_rejected_before_scene_execution(tmp_path,period):
    with pytest.raises(ValueError,match='positive finite'):fixture_setup(tmp_path,period)


def active_completed():
    return dict(passed=True,positive_negative_programs_completed=True,stop_and_hold_confirmed=True,
        both_fixed_grasps_retained=True,completed_goals=2,mode='XY_YAW_COMP',delta_deg=.005,grasp_losses=[])


def test_active_extension_requires_completed_same_mode_and_exact_geometry_proof():
    from amir_rotation_integration_audit import require_completed_active_step
    prior=active_completed();proof={'yaw_equivariance_proof':{'joint_index':0}}
    require_completed_active_step(.01,prior,'XY_YAW_COMP',proof)
    with pytest.raises(ValueError,match='same-mode'):require_completed_active_step(.01,prior,'WHOLE_BODY',proof)
    with pytest.raises(ValueError,match='certificate'):require_completed_active_step(.01,prior,'XY_YAW_COMP',{})


@pytest.mark.parametrize('field',['passed','positive_negative_programs_completed','stop_and_hold_confirmed','both_fixed_grasps_retained'])
def test_failed_or_incomplete_active_trial_cannot_authorize_next_step(field):
    from amir_rotation_integration_audit import require_completed_active_step
    prior=active_completed();prior[field]=False
    with pytest.raises(ValueError,match='evidence'):require_completed_active_step(.01,prior,'XY_YAW_COMP',{'yaw_equivariance_proof':True})


@pytest.mark.parametrize('delta',[0.,-.005,float('nan'),float('inf'),.004,.015001,90.001])
def test_active_expansion_cannot_skip_the_bounded_progression(delta):
    from amir_rotation_integration_audit import require_completed_active_step
    with pytest.raises(ValueError,match='step'):require_completed_active_step(delta,active_completed(),'XY_YAW_COMP',{'yaw_equivariance_proof':True})


def test_leader_planning_can_only_reduce_declared_motion_limits():
    from amir_rotation_integration_audit import conservative_leader_limits
    old={'rotation_max_angular_speed':.002,'rotation_tracking_limit_rad':.01}
    assert conservative_leader_limits(old,{'rotation_max_angular_speed':.0004})['rotation_max_angular_speed']==.0004
    assert old['rotation_max_angular_speed']==.002
    for value in [0.,-.1,.002001,float('nan'),float('inf')]:
        with pytest.raises(ValueError):conservative_leader_limits(old,{'rotation_max_angular_speed':value})
    with pytest.raises(ValueError):conservative_leader_limits(old,{'rotation_tracking_limit_rad':.02})


def test_formal_ablation_cannot_exceed_completed_whole_body_reference():
    from amir_rotation_integration_audit import require_qualified_ablation
    ref=active_completed();ref.update(mode='WHOLE_BODY',delta_deg=5.)
    qualification={'yaw_equivariance_proof':True}
    for mode in ['XY_ONLY','XY_YAW','XY_YAW_COMP']:require_qualified_ablation(5.,ref,mode,qualification)
    with pytest.raises(ValueError,match='endpoint'):require_qualified_ablation(5.001,ref,'XY_ONLY',qualification)
    with pytest.raises(ValueError,match='reference'):require_qualified_ablation(5.,ref,'WHOLE_BODY',qualification)
    ref['passed']=False
    with pytest.raises(ValueError,match='evidence'):require_qualified_ablation(5.,ref,'XY_YAW_COMP',qualification)
