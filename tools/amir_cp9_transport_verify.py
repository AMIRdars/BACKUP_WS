#!/usr/bin/env python3
"""Cross-check native carry evidence and the unchanged original control path."""
import argparse
import ast
import hashlib
import json
import math
import re
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--normal-trials',nargs='+',required=True)
    p.add_argument('--fault-trials',nargs='+',required=True)
    p.add_argument('--metrology',default='metrology_02')
    p.add_argument('--report-name',default='final_checks.json')
    a=p.parse_args();root=a.audit_directory;checks={}
    baseline=read(root/'before/source_hashes.json')
    changes=[f for f,h in baseline.items() if not Path(f).is_file() or sha(Path(f))!=h]
    checks['only_intended_existing_source_edit']=changes==['src/humbleble_ws/amir_compliance_control/setup.py']
    checks['tracked_user_diff_preserved']=subprocess.check_output(['git','diff','--binary'])==(root/'before/tracked.patch').read_bytes()
    original=[v for v in read(root.parent/'cp0/source_files.json') if Path(v['path']).suffix in {'.urdf','.xacro','.sdf','.yaml'}]
    checks['original_105_models_and_yaml_unchanged']=len(original)==105 and all(sha(Path(v['path']))==v['sha256'] for v in original)
    checks['prior_cp9_static_evidence_preserved']=all(read(root.parent/'cp9/final_checks.json')['checks'].values())
    checks['hardware_skipped_and_yaw_pivot_unaccepted']=read(root/'scope.json')['hardware']=='SKIPPED_BY_USER' and not read(root.parent/'cp8/final_checks.json')['all_five_cooperative_motions_passed']
    code_root=Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control')
    core=code_root/'native_grasp.py';tree=ast.parse(core.read_text())
    imports={n.module.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.ImportFrom) and n.module}
    checks['native_monitor_no_simulator_tf_or_evaluator_import']=not imports.intersection({'ros_gz_interfaces','tf2_msgs','tools','gazebo_msgs'})
    publishers={n.args[1].value for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr=='create_publisher' and len(n.args)>1 and isinstance(n.args[1],ast.Constant)}
    checks['native_monitor_has_no_motion_publisher']=publishers=={'grasp/state','grasp/monitor_debug','/transport/emergency_stop'}
    old_policy=ast.parse(Path('tools/amir_cp9_local_grasp.py').read_text())
    preparation_policy=ast.parse(Path('tools/amir_cp9_grasp_preparation.py').read_text())
    for function in ('home_from_urdf','position_step','mimic_measurement_ok'):
        get=lambda t:next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name==function)
        checks['preparation_inherits_previous_'+function]=ast.dump(get(old_policy))==ast.dump(get(preparation_policy))
    deps={v.text for v in ET.parse(code_root.parent/'package.xml').getroot() if v.tag.endswith('depend')}
    checks['control_package_no_gazebo_or_evaluation_dependency']=not any('gazebo' in d or d.startswith('ros_gz') or 'evaluation' in d for d in deps)
    measurement=root/a.metrology;cert=read(measurement/'calibration/calibration.json')
    checks['new_six_axis_positive_and_negative_calibration']=cert['sensor_calibration_passed'] and len(cert['tests'])==12 and all(v['passed'] for v in cert['tests'])
    runtime=read(measurement/'static_proof/runtime.json')
    checks['both_actual_joint_reaction_streams_measured']=all(runtime['counts'].get('/'+r+'/joint_load/Joint_'+str(i),0)>=100
        for r in ('leader','follower') for i in range(1,5))
    generated=root/'generated';world=ET.parse(generated/'world.sdf').getroot()
    def canonical(element):
        return (element.tag,tuple(sorted(element.attrib.items())),(element.text or '').strip(),
            tuple(canonical(child) for child in element))
    for robot in ('leader','follower'):
        previous=root.parent/'cp1/run3_generated'
        old_robot=ET.parse(previous/(robot+'.urdf')).getroot()
        new_robot=ET.parse(generated/(robot+'.urdf')).getroot()
        checks[robot+'_physical_links_and_joints_match_motion_evidence']=all(
            [canonical(v) for v in old_robot.findall(tag)]==[canonical(v) for v in new_robot.findall(tag)]
            for tag in ('link','joint'))
        old_controller=yaml.safe_load((previous/(robot+'_controllers.yaml')).read_text())
        new_controller=yaml.safe_load((generated/(robot+'_base_controller.yaml')).read_text())
        checks[robot+'_mecanum_parameters_match_motion_evidence']=old_controller['/**/mecanum_drive_controller']==new_controller['/**/mecanum_drive_controller']
    plugins={v.get('name').rsplit('::',1)[-1] for v in world.findall('.//plugin')}
    checks['native_world_has_no_attachment_or_external_force_plugin']=plugins=={'Physics','UserCommands','SceneBroadcaster','Contact','Sensors','ForceTorque','DartContactParameters'} and not world.findall('.//joint')
    checks['metrology_force_fixture_separate']=ET.parse(measurement/'generated/world.sdf').find("world/plugin[@name='amir_follower::CalibrationWrench']") is not None and sha(generated/'world.sdf')!=sha(measurement/'generated/world.sdf')
    checks['native_robot_matches_measured_certificate']=sha(generated/'follower.urdf')==cert['urdf_sha256']==sha(measurement/'generated/follower.urdf')
    mass=float(world.findtext("world/model[@name='cooperative_payload']/link/inertial/mass"))
    checks['native_payload_existing_mass']=mass==.5
    summary=[]
    selected=a.normal_trials+a.fault_trials
    native_hashes={read(root/n/'control_source_hashes.json')['native_grasp.py'] for n in selected}
    checks['all_selected_trials_use_same_current_native_controller']=native_hashes=={sha(core)}
    for name in selected:
        folder=root/name;r=read(folder/'result.json');tag=name+'__'
        checks[tag+'passed']=bool(r['passed']) and not r.get('hardware_motion_gate_open',True)
        source_ok=True
        for f,h in read(folder/'trial_source_hashes.json').items():
            current=Path(f)
            if sha(current)!=h:
                archived=folder/'source_snapshot'/current.name
                if name=='forward_01':archived=root/'prototype_source'/current.name
                source_ok=source_ok and archived.is_file() and sha(archived)==h
        for f,h in read(folder/'control_source_hashes.json').items():
            if sha(code_root/f)!=h:
                archived=root/'prototype_source'/f
                source_ok=source_ok and name=='forward_01' and archived.is_file() and sha(archived)==h
        checks[tag+'exact_runtime_source_current_or_archived']=source_ok
        monitor_hash=read(folder/'control_source_hashes.json')['native_grasp.py']
        checks[tag+'live_monitor_matches_recorded_source']=len(r['monitor_before_teardown'])==2 and all(v['source_sha256']==monitor_hash for v in r['monitor_before_teardown'].values())
        checks[tag+'native_world_matches_run_hash']=r['generated_hashes']['world.sdf']==sha(generated/'world.sdf')
        for robot in ('leader','follower'):
            profile=read(folder/'profiles_used'/(robot+'_safety.json'))
            checks[tag+robot+'_matching_measured_profile']=(profile['urdf_sha256']==sha(generated/(robot+'.urdf'))
                and profile['world_sha256']==sha(generated/'world.sdf') and profile['require_grasp_for_base_motion']
                and not profile['ungrasped_base_test_only'] and profile['joint_reaction_projection_verified']
                and profile['max_static_projection_error_Nm']<.02
                and profile['controller_yaml_sha256']==sha(generated/(robot+'_controllers.yaml'))
                and profile['base_controller_yaml_sha256']==sha(generated/(robot+'_base_controller.yaml')))
        checks[tag+'handoff_rejects_prep_writer']=all(r['premature_takeover_rejected'].values()) and len(r['premature_takeover_rejected'])==2
        checks[tag+'preparation_exited_before_interlocks']=r['handoff']['preparation_processes_exited'] and r['handoff']['preparation_arm_writers_absent']
        placements=[json.loads(l) for l in (folder/'fixture_placement.jsonl').read_text().splitlines()]
        checks[tag+'only_two_initial_pose_requests']=len(placements)==2 and all(set(v['states'].values())=={'READY_TO_CLOSE'} and v['phase']=='fixture_initialization' for v in placements)
        checks[tag+'native_static_weight_and_hold']=abs(r['static_check']['measured_weight_N']-mass*9.81)<.1 and max(r['static_check']['late_drift_xyz_m'])<.0001
        graph=read(folder/'graph.json')
        required=['/follower/'+n for n in ('follower_whole_body','wrench_preprocessor','safety_gate','actuator_interlock','controller_audit','native_grasp_monitor')]
        allowed_global={'/clock','/transport/safety/follower','/transport/interlock/follower','/transport/emergency_stop','/transport/safety_permit'}
        subscriptions=[topic for n in required if n in graph for topic,types in graph[n]['subscriptions']]
        checks[tag+'strict_follower_input_boundary']=all(n in graph for n in required) and all(
            (t.startswith('/follower/') or t in allowed_global) and not any(s in t for s in ('/leader/','goal','ground_truth','/world/'))
            and not t.endswith(('/tf','/tf_static')) for t in subscriptions)
        checks[tag+'prep_nodes_absent_after_handoff']=not any(n.endswith(('/cp9_local_grasp','/cp9_contact_preparation')) for n in graph)
        checks[tag+'exclusive_final_arm_writers']=all([n for n,v in graph.items() for t,_ in v['publishers']
            if t=='/'+robot+'/arm_controller/joint_trajectory']==['/'+robot+'/actuator_interlock'] for robot in ('leader','follower'))
        checks[tag+'no_follower_goal']=r['endpoint_states']['follower']['accepted_goals']==0
        rows=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
        gt_rows=[v for v in rows if 'payload_qw' in v]
        checks[tag+'gt_finite_unit_quaternions']=bool(gt_rows) and all(all(math.isfinite(v['payload_'+k]) for k in ('x','y','z','qx','qy','qz','qw'))
            and abs(sum(v['payload_q'+k]**2 for k in 'xyzw')-1.)<1e-5 for v in gt_rows)
        item={'trial':name,'fault':r['fault_test'],'passed':r['passed'],'distance_m':r['distance_m'],'speed_m_s':r['speed_m_s']}
        if name in a.normal_trials:
            checks[tag+'actual_cooperative_carry']=r['cooperative_contact_transport_verified'] and all(.7<=v<=1.3 for v in r['tracking_ratios'].values())
            checks[tag+'settled_with_grip_intact']=(r['final_arm_offset_xy_m']<=.0001 and r['final_follower_speed_m_s']<=.00001
                and all(v['state']=='HOLDING' for v in r['monitor_before_teardown'].values()))
            motion=[v for v in rows if v['phase'] in ('motion','settle')]
            item.update(gt_delta_xy_yaw=r['gt_delta_xy_yaw'],tracking_ratios=r['tracking_ratios'],
                final_arm_offset_xy_m=r['final_arm_offset_xy_m'],final_follower_speed_m_s=r['final_follower_speed_m_s'],
                peak_measured_arm_error_m=max(math.hypot(v['measured_error_x'],v['measured_error_y']) for v in motion),
                peak_low_pass_arm_error_m=max(math.hypot(v['low_error_x'],v['low_error_y']) for v in motion),
                peak_follower_base_speed_m_s=max(math.hypot(v['follower_output_vx'],v['follower_output_vy']) for v in motion))
        else:
            checks[tag+'fault_injected_while_both_really_moving']=(r.get('fault_injected',False)
                and all(abs(v)>5e-6 for v in r['measured_encoder_vx_at_injection'].values())
                and all(v>1e-6 for v in r['gt_travel_before_injection_m'].values()))
            budget={'contacts':1.,'grasp_monitor':1.2}.get(r['fault_test'],.4)
            checks[tag+'dual_zero_and_real_controller_stop']=(r['dual_zero_latency_wall_s']<=budget
                and r['both_arm_base_inactive'] and max(r['post_fault_max_wheel_speed_rad_s'].values())<1e-4)
            checks[tag+'no_automatic_restart']=r['no_automatic_restart'] and max(r['post_fault_joint_span_rad'])<1e-5
            snap=read(folder/'post_fault_controller_snapshot.json')
            cleanup=[json.loads(l) for l in (folder/'service_results.jsonl').read_text().splitlines()
                if json.loads(l)['service']=='/transport/safety/stop']
            checks[tag+'fresh_controller_rpc_before_cleanup_stop']=snap['before_teardown_stop'] and all(
                v.get(c)=='inactive' for v in snap['states'].values() for c in ('arm_controller','mecanum_drive_controller')) and bool(cleanup) and snap['wall_time']<cleanup[0]['wall_time']
            if r['fault_test']=='ownership':checks[tag+'duplicate_writer_removed_before_recovery_check']=r.get('injected_extra_writer_removed',False)
            if r['fault_test']=='contacts':checks[tag+'contact_heartbeat_restored_without_restart']=r.get('contact_inputs_fresh_after_recovery',False)
            item.update(dual_zero_latency_wall_s=r['dual_zero_latency_wall_s'],measured_encoder_vx_at_injection=r['measured_encoder_vx_at_injection'],
                gt_travel_before_injection_m=r['gt_travel_before_injection_m'],actual_final_controller_states=r['actual_final_controller_states'])
        summary.append(item)
    remaining=[]
    for f in Path('/proc').iterdir():
        if not f.name.isdigit():continue
        try:
            env=(f/'environ').read_bytes().split(b'\0');argv=(f/'cmdline').read_bytes()
            if any(e.startswith(b'IGN_PARTITION=amir_cp9_') for e in env) and any(s in argv for s in
                (b'/lib/amir_compliance_control/',b'ign gazebo',b'parameter_bridge',b'finger_contact_aggregator',b'amir_cp9_local_grasp.py',b'amir_cp9_grasp_preparation.py')):
                remaining.append(int(f.name))
        except OSError:pass
    checks['no_remaining_cp9_simulation_processes']=not remaining
    tests=0
    test_log=root/'all_tests_final.log'
    if test_log.exists():
        match=re.findall(r'(\d+) passed',test_log.read_text())
        tests=int(match[-1]) if match else 0
    checks['final_complete_test_suite_passed']=tests==298 and (root/'final_test_exit.json').exists() and read(root/'final_test_exit.json')['returncode']==0
    report={'checkpoint':'CP-9 native grasp guarded carry','checks':checks,'passed_count':sum(checks.values()),
        'check_count':len(checks),'all_passed':all(checks.values()),'prior_source_count':len(baseline),'intended_existing_source_edits':changes,
        'hardware_motion_gate_open':False,'hardware':'SKIPPED_BY_USER','native_cooperative_linear_carry_verified':all(checks.get(n+'__actual_cooperative_carry',False) for n in a.normal_trials),
        'moving_faults_verified':[read(root/n/'result.json')['fault_test'] for n in a.fault_trials if checks.get(n+'__dual_zero_and_real_controller_stop',False)],
        'yaw_pivot_verified':False,'remaining_owned_pids':remaining,'tests_passed':tests,'trials':summary}
    (root/a.report_name).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['all_passed']:raise SystemExit(1)


if __name__=='__main__':main()
