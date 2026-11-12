"""Phase 1 cannot silently enable base assist or proxy force inputs."""
from pathlib import Path
import runpy
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch'


def context_for(filename):
    module = runpy.run_path(str(LAUNCH/filename))
    ctx = LaunchContext()
    for action in module['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    return module, ctx


@pytest.mark.parametrize('name', ['compliance_arm_first.launch.py', 'compliance_round_trip.launch.py'])
def test_base_assist_rejected_before_creating_output(tmp_path, name):
    module, ctx = context_for(name)
    ctx.launch_configurations.update(enable_base_assist='true', output_directory=str(tmp_path/'out'))
    with pytest.raises(ValueError, match='enable_base_assist=false'):
        module['setup'](ctx)
    assert not (tmp_path/'out').exists()


def test_phase2_cannot_be_selected():
    module, ctx = context_for('compliance_arm_first.launch.py')
    ctx.launch_configurations['phase'] = '2'
    with pytest.raises(ValueError, match='Only Phase 1'):
        module['setup'](ctx)


def test_phase1_uses_original_native_grasp_and_small_stroke():
    module, ctx = context_for('compliance_arm_first.launch.py')
    action, = module['setup'](ctx)
    assert isinstance(action, IncludeLaunchDescription)
    args = {k: perform_substitutions(ctx, [v]) if not isinstance(v, str) else v
            for k, v in action.launch_arguments}
    assert args['enable_base_assist'] == 'false'
    assert args['wrench_source_mode'] == 'sensor'
    assert args['control_mode'] == 'ARM_ONLY'
    assert args['arm_only_trial'] == 'native_stroke'
    assert args['leader_base_distance_m'] == '0.009'
    assert args['arm_only_speed_m_s'] == '0.005'


@pytest.mark.parametrize('mode', ['target_proxy', 'test_injection', 'missing'])
def test_public_launch_does_not_mislabel_proxy_as_force_tracking(mode):
    module, ctx = context_for('compliance_round_trip.launch.py')
    ctx.launch_configurations['wrench_source_mode'] = mode
    with pytest.raises(ValueError, match='F/T sensor'):
        module['setup'](ctx)


@pytest.mark.parametrize('overrides', [{'enable_base_assist':True}, {'wrench_source_mode':'target_proxy'}])
def test_policy_cannot_silently_select_assist_or_proxy(tmp_path,overrides):
    import json
    policy=tmp_path/'policy.json';policy.write_text(json.dumps({'controller_parameters':overrides}))
    module=runpy.run_path(str(ROOT/'src/humbleble_ws/amir_compliance_control/launch/arm_only_af.launch.py'))
    ctx=LaunchContext();ctx.launch_configurations.update(control_mode='ARM_ONLY',rotation_tracking_enabled='false',
        run_directory=str(tmp_path),experiment_policy=str(policy))
    with pytest.raises(ValueError,match='Phase 1 policy requires'):
        module['setup'](ctx)


def measured_row():
    row={'admittance_enabled':'True','adm_enable_base_assist':'False','adm_wrench_source_mode':'sensor',
         'adm_phase_mode':'ARM_ONLY','adm_r_arm_xy_m':'.002'}
    for key,vals in [('adm_p_actual_m',[.002,0.,0.]),('adm_p_home_m',[0.,0.,0.]),
                     ('follower_q_actual',[.002]*5),('adm_ik_residual_xy_m_s',[0.,0.]),
                     ('follower_qdot_actual',[.001]*5),('follower_wrench_raw',[.1]*6)]:
        row.update({key+'_'+str(i):str(v) for i,v in enumerate(vals)})
    return row


def test_evidence_rejects_command_only_movement():
    from amir_arm_first_phase1_evidence import phase_checks
    rows=[measured_row(),measured_row()]
    # Nonzero requested motion must never substitute for stationary FK/q_actual.
    for row in rows:
        row.update(adm_p_actual_m_0='0.',adm_r_arm_xy_m='0.',adm_q_cmd_rad_0='.5')
    checks,_=phase_checks(rows)
    assert not checks['arm_measured_travel_exceeds_1mm']
    assert not checks['joint_actual_position_changes']


def test_evidence_requires_consistent_measured_radius_and_real_sensor_mode():
    from amir_arm_first_phase1_evidence import phase_checks
    first=measured_row();second=measured_row()
    first.update(adm_p_actual_m_0='0.',adm_r_arm_xy_m='0.',follower_q_actual_0='0.')
    checks,_=phase_checks([first,second]);assert all(checks.values())
    second.update(adm_r_arm_xy_m='.4',adm_wrench_source_mode='target_proxy')
    checks,_=phase_checks([first,second])
    assert not checks['r_from_measured_FK'] and not checks['real_sensor_mode_only']
