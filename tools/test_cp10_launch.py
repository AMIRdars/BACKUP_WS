"""Speed trial preflight: units, qualification, isolation and GUI ownership."""
import importlib.util
import hashlib
import json
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess

ROOT = Path(__file__).resolve().parent.parent
PATH = ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_speed_trial.launch.py'
spec = importlib.util.spec_from_file_location('cp10_speed_entry',PATH)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def context(tmp_path,**overrides):
    ctx = LaunchContext()
    for action in entry.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations.update(workspace_directory=str(ROOT),headless='true',
                                     output_directory=str(tmp_path/'new_output'))
    ctx.launch_configurations.update(overrides)
    return ctx


@pytest.mark.parametrize(('name','value'),[
    ('forward_distance_m','1.0'),('forward_distance_m','251'),('forward_distance_m','nan'),
    ('translation_velocity_m_s','1.01'),('translation_velocity_m_s','1.0'),('translation_velocity_m_s','0'),
    ('translation_velocity_m_s','nan'),('rotation_angle_deg','90'),('world_y_distance_m','1'),
    ('translation_acceleration_m_s2','.025'),('ros_domain_id','233')])
def test_unqualified_motion_fails_before_process_creation(tmp_path,name,value):
    with pytest.raises(ValueError):
        entry.setup(context(tmp_path,**{name:value}))


def test_absent_qualification_refuses_even_numerically_valid_request(tmp_path):
    with pytest.raises(ValueError,match='qualification is incomplete'):
        entry.setup(context(tmp_path,workspace_directory=str(tmp_path)))


@pytest.mark.parametrize('gui',[False,True])
@pytest.mark.parametrize('offload',[False,True])
def test_one_command_forwards_metres_and_speed_and_keeps_gui_in_same_partition(tmp_path,monkeypatch,gui,offload):
    # Wiring only: no fabricated motion evidence is written or used to run ROS.
    monkeypatch.setattr(entry,'qualification',lambda root:{'base_speed_evidence':'proof-unused',
        'speed_m_s':.1, 'distance_m':3.})
    ctx = context(tmp_path,headless='false' if gui else 'true',gui_nvidia_offload='true' if offload else 'false')
    processes = [a for a in entry.setup(ctx) if isinstance(a,ExecuteProcess)]
    assert len(processes) == (2 if gui else 1)
    for process in processes:
        process.prepare(ctx)
    details = processes[-1].process_details
    cmd = details['cmd']
    assert cmd[cmd.index('--distance')+1] == '3.0'
    assert cmd[cmd.index('--speed')+1] == '0.1'
    assert '--experimental-speed-profile' in cmd and '--base-speed-evidence' in cmd
    assert '--grip-maintenance' in cmd
    assert '--display-scene' in cmd
    assert details['env']['RMW_FASTRTPS_PUBLICATION_MODE'] == 'ASYNCHRONOUS'
    assert details['cwd'] == str(ROOT) and details['env']['ROS_LOCALHOST_ONLY'] == '1'
    assert details['env']['IGN_PARTITION'].startswith('amir_cp9_cp12_launch_')
    if gui:
        gui_cmd = processes[0].process_details['cmd']
        assert gui_cmd[:3] == ['ign', 'gazebo', '-g']
        assert '--gui-config' in gui_cmd
        assert gui_cmd[gui_cmd.index('--gui-config')+1].endswith('compliance_speed_gui.config')
        assert processes[0].process_details['env']['IGN_PARTITION'] == details['env']['IGN_PARTITION']
        assert (processes[0].process_details['env'].get('__NV_PRIME_RENDER_OFFLOAD') == '1') == offload
        assert (processes[0].process_details['env'].get('__GLX_VENDOR_LIBRARY_NAME') == 'nvidia') == offload
        assert '__NV_PRIME_RENDER_OFFLOAD' not in details['env']
    assert not (tmp_path/'new_output').exists()


@pytest.mark.parametrize(('speed','distance'),[('.05','3'),('.1','2.5'),('.1','2.9')])
def test_only_measured_distance_and_speed_are_released(tmp_path,monkeypatch,speed,distance):
    monkeypatch.setattr(entry,'qualification',lambda root:{'base_speed_evidence':'proof-unused',
        'speed_m_s':.1, 'distance_m':3.})
    with pytest.raises(ValueError,match='exactly 3 m'):
        entry.setup(context(tmp_path,translation_velocity_m_s=speed,forward_distance_m=distance))


@pytest.fixture
def saved_proof(tmp_path, monkeypatch):
    """Isolated guard fixtures only; never write production qualification or run ROS."""
    monkeypatch.setattr(entry, 'STAGE', tmp_path)
    control = ROOT/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    source_files = [*control.glob('*.py'), PATH, ROOT/'tools/amir_cp9_transport_trial.py',
        ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp12_display_scene.launch.py',
        ROOT/'tools/cp12_display_scene.py',
        ROOT/'src/humbleble_ws/amir_compliance_control/launch/native_grasp_monitor.launch.py',
        ROOT/'src/humbleble_ws/amir_compliance_control/launch/whole_body_transport.launch.py',
        ROOT/'src/humbleble_ws/cooperative_transport_bringup/config/compliance_speed_gui.config']
    source_files += [ROOT/entry.DISPLAY_SCENE/name for name in entry.SCENE_FILES]
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    control_hashes = {p.name:sha(p) for p in control.glob('*.py')}
    normal = {'passed':True, 'both_arm_base_inactive':True, 'follower_input_boundary_passed':True,
        'distance_m':3., 'speed_m_s':.1, 'cooperative_contact_transport_verified':True,
        'grip_maintenance_enabled':True, 'final_arm_offset_xy_m':1e-8,
        'final_follower_speed_m_s':1e-7, 'independent_speed_evaluation':{
            r:{'steady_gt_mean_m_s':.1} for r in ('leader','follower','cooperative_payload')}}
    fault = {**normal, 'fault_test':'nonfinite', 'fault_injected':True,
        'no_automatic_restart':True, 'dual_zero_latency_wall_s':.1,
        'twists_at_injection':{'leader':[.1,0.,0.], 'follower':[-.1,0.,0.]},
        'measured_encoder_vx_at_injection':{'leader':.1, 'follower':-.1},
        'post_fault_controller_states':{r:{c:'inactive' for c in
            ('arm_controller','mecanum_drive_controller')} for r in ('leader','follower')},
        'maintenance_at_dual_stop':{r:{'sent_goals':3} for r in ('leader','follower')},
        'maintenance_after_fault_recovery':{r:{'sent_goals':3, 'state':'FAULT'} for r in ('leader','follower')}}
    manifest = {'schema_version':3, 'passed':True, 'simulation_only':True,
        'grip_maintenance':True, 'publication_mode':'ASYNCHRONOUS', 'speed_m_s':.1, 'distance_m':3.,
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in source_files}, 'evidence_sha256':{}}
    for label, result in (('normal',normal), ('fault',fault)):
        folder = tmp_path/label; folder.mkdir()
        manifest[label+'_trial'] = str(folder)
        for name, data in [('result.json',result), ('control_source_hashes.json',control_hashes),
                ('trial_source_hashes.json',{str(ROOT/'tools/amir_cp9_transport_trial.py'):sha(ROOT/'tools/amir_cp9_transport_trial.py')}),
                ('control_policy.json',{'base_force_acceleration_gains':[.05,0.,0.], 'base_force_filter_hz':.1,
                    'base_integral_gains':[0.,0.,0.], 'base_stationary_hold':True, 'arm_command_horizon_s':.02,'allocation_on_wrench':True,'allocation_use_clock':True}), ('leader_policy.json',{}),
                ('command.json',{'RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS'}),
                ('original_profiles_restored.json',{'leader':True,'follower':True})]:
            p=folder/name;p.write_text(json.dumps(data));manifest['evidence_sha256'][str(p)]=sha(p)
    target = tmp_path/'qualified_speed_010.json'
    target.write_text(json.dumps(manifest))
    def update(label, filename, mutate):
        p=tmp_path/label/filename; data=json.loads(p.read_text());mutate(data)
        p.write_text(json.dumps(data));manifest['evidence_sha256'][str(p)]=sha(p)
        target.write_text(json.dumps(manifest))
    return manifest, target, update


def test_qualification_checks_current_runtime_sources(saved_proof):
    manifest, target, update = saved_proof
    assert entry.qualification(ROOT)['schema_version'] == 3
    update('normal', 'control_source_hashes.json', lambda d:d.update({'grip_maintenance.py':'old-source'}))
    with pytest.raises(ValueError, match='runtime control sources differ'):
        entry.qualification(ROOT)


def test_old_fixed_grip_proof_cannot_release_maintained_grip(saved_proof):
    _, _, update = saved_proof
    update('normal', 'result.json', lambda d:d.update(grip_maintenance_enabled=False))
    with pytest.raises(ValueError, match='enable local grip maintenance'):
        entry.qualification(ROOT)


def test_modified_result_without_new_digest_refuses_release(saved_proof):
    manifest, _, _ = saved_proof
    p=Path(manifest['normal_trial'])/'result.json'
    p.write_text(p.read_text()+'\n')
    with pytest.raises(ValueError, match='qualification evidence changed'):
        entry.qualification(ROOT)


def test_resumed_gripper_after_fault_refuses_release(saved_proof):
    _, _, update = saved_proof
    update('fault','result.json',lambda d:d['maintenance_after_fault_recovery']['follower'].update(sent_goals=4))
    with pytest.raises(ValueError, match='must not resume'):
        entry.qualification(ROOT)


def test_commanded_speed_without_measured_cruise_refuses_release(saved_proof):
    _, _, update = saved_proof
    update('fault','result.json',lambda d:d['measured_encoder_vx_at_injection'].update(follower=-.07))
    with pytest.raises(ValueError, match='own encoders'):
        entry.qualification(ROOT)
