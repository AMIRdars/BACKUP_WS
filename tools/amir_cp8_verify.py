#!/usr/bin/env python3
"""Final CP-8 evidence gates, after all owned process groups have ended."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--audit-directory',type=Path,required=True);a=parser.parse_args();p=a.audit_directory
    read=lambda f:json.loads(f.read_text());sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();checks={}
    checks['preexisting_tracked_diff_preserved']=subprocess.check_output(['git','diff','--binary'])==(p/'before/tracked.patch').read_bytes()
    originals=[r for r in read(p.parent/'cp0/source_files.json') if Path(r['path']).suffix in ('.urdf','.xacro','.sdf','.yaml')]
    checks['original_105_models_and_yaml_preserved']=len(originals)==105 and all(sha(Path(r['path']))==r['sha256'] for r in originals)
    generated=p.parent/'cp6/generated';certificate=read(p.parent/'cp6/calibration/calibration.json')
    checks['matching_six_axis_certificate']=certificate['sensor_calibration_passed'] and len(certificate['verified_axes'])==6 and sha(generated/'follower.urdf')==certificate['urdf_sha256']
    checks['physical_load_limits_not_certified']=not certificate['load_limits_verified']
    for robot in ('leader','follower'):
        profile=read(generated/(robot+'_safety.json'))
        checks[robot+'_normal_measured_profile_restored']=(profile['urdf_sha256']==sha(generated/(robot+'.urdf')) and profile['world_sha256']==sha(generated/'world.sdf')
            and profile['simulation_only'] and profile['require_grasp_for_base_motion'] and not profile['ungrasped_base_test_only']
            and all(profile[k+'_sha256']==sha(Path(profile[k+'_file'])) for k in ('controller_yaml','base_controller_yaml')))
    model=read(p/'model_audit.json')
    checks['every_whole_jacobian_column_finite_differences']=max(model['finite_difference_max_error'].values())<2e-9 and model['finite_difference_samples_per_point']==51
    checks['actual_model_arm_rank5_whole_rank6']=model['arm_rank']==5 and model['whole_rank']==6 and model['sha256']==sha(generated/'follower.urdf')
    checks['actual_sensor_tcp_offset_accounted']=abs(model['sensor_to_tcp_norm_m']-.0795)<1e-10
    code=Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control')
    current=lambda folder:all(sha(code/f)==h for f,h in read(folder/'control_source_hashes.json').items())
    for name,count in [('accepted_standalone_xy',4),('accepted_standalone_yaw',2)]:
        r=read(p/name/'result.json')
        checks[name+'_passed']=r['passed'] and len(r['sign_tests'])==count and all(v['passed'] for v in r['sign_tests'])
        checks[name+'_local_and_current_source']=r['follower_input_boundary_passed'] and current(p/name)
        checks[name+'_profiles_restored']=all(read(p/(name+'_profiles_restored.json')).values())
    comparisons=read(p/'final_comparison_batch.json')
    checks['ten_final_same_task_comparisons']=len(comparisons)==10 and {(r['mode'],r['axis']) for r in comparisons}=={(m,x) for m in ('ARM_BASE','WHOLE_BODY') for x in ('forward','backward','lateral','yaw','pivot')}
    metrics=read(p/'comparison_metrics.json')
    for r in comparisons:
        name=r['trial'];folder=p/name;result=read(folder/'result.json');policy=read(folder/'control_policy.json')
        checks[name+'_local_leader_only_goal']=result['follower_input_boundary_passed'] and result['endpoint_states']['leader']['accepted_goals']==1 and result['endpoint_states']['follower']['accepted_goals']==0
        checks[name+'_current_source']=current(folder)
        checks[name+'_same_physics_and_load_profile']=result['weld_cfm']==.1 and abs(result['static_check']['mass_kg']-.52)<1e-12 and all(read(folder/'profiles_used'/(bot+'_safety.json'))==read(generated/(bot+'_safety.json')) for bot in ('leader','follower'))
        checks[name+'_same_xy_policies']=policy['virtual_mass']==[20,20] and policy['virtual_damping']==[1200,1200] and policy['virtual_stiffness']==[400,400] and policy['base_gains']==[.5,.5,1] and not policy['allow_ungrasped_test']
        checks[name+'_zero_external_excitation']=all(all(json.loads(l).get(key,0)==0 for key in ('applied_fx_N','applied_fy_N','applied_fz_N','applied_tx_Nm','applied_ty_Nm','applied_tz_Nm')) for l in (folder/'samples.jsonl').read_text().splitlines())
        if r['mode']=='WHOLE_BODY':
            checks[name+'_stated_virtual_yaw_and_filter_policy']=policy['virtual_yaw_inertia']==20 and policy['virtual_yaw_damping']==1200 and policy['virtual_yaw_stiffness']==400 and policy['allocation_base_filter_hz']==.2
            checks[name+'_actual_allocation_constraints_verified']=next(v for v in metrics if v['trial']==name)['allocation_speed_acceleration_verified']
        checks[name+'_twelve_second_settling_observed']=result['settle_seconds']==12
        outputs=[json.loads(l) for l in (folder/'command_outputs.jsonl').read_text().splitlines() if 'arm_request' not in l]
        last={bot:next(v for v in reversed(outputs) if v['robot']==bot) for bot in ('leader','follower')}
        checks[name+'_explicit_end_stop_confirmed']=not result['final_safety_state']['armed'] and all(max(abs(last[bot][axis]) for axis in ('vx','vy','wz'))<1e-12 for bot in last)
    for axis in ('forward','backward','lateral','yaw','pivot'):
        rs=[read(p/r['trial']/'result.json') for r in comparisons if r['axis']==axis]
        checks[axis+'_matched_target_and_speed']=len(rs)==2 and max(np.max(abs(np.array(rs[0]['relative_goal_xy_yaw'])-r['relative_goal_xy_yaw'])) for r in rs[1:])<1e-8 and len({r['speed_m_s'] for r in rs})==1
    forbidden={'tools','ros_gz_interfaces','ros_gz_sim','cooperative_transport_gazebo','cooperative_transport_bringup','amir_follower_gazebo'};imports=set()
    for file in code.glob('*.py'):
        for n in ast.walk(ast.parse(file.read_text())):
            if isinstance(n,ast.Import):imports.update(v.name.split('.')[0] for v in n.names)
            if isinstance(n,ast.ImportFrom) and n.module:imports.add(n.module.split('.')[0])
    checks['control_imports_independent_of_evaluation_and_simulator']=not imports.intersection(forbidden)
    deps={e.text for e in ET.parse(code.parent/'package.xml').getroot() if e.tag.endswith('depend')}
    checks['control_package_dependencies_independent']=not deps.intersection(forbidden)
    validation=read(p/'final_faults_validation.json') if (p/'final_faults_validation.json').exists() else []
    fault_trials={r['fault']:r for r in validation}
    fault_names=('gate','interlock','supervisor','relay','audit','controller','ownership','action','dropout','preprocessor','clock')
    checks['eleven_distinct_injected_faults_validated']=len(validation)==11 and set(fault_trials)==set(fault_names)
    for fault in fault_names:
        selected=fault_trials.get(fault,{'trial':'final_fault_'+fault,'preserved_failures':[]})
        folder=p/selected['trial']
        if not (folder/'result.json').exists():checks['fault_'+fault+'_exists']=False;continue
        r=read(folder/'result.json')
        checks['fault_'+fault+'_preinjection_failures_preserved']=all((p/name/'result.json').exists() and not read(p/name/'result.json')['passed'] and 'twists_at_injection' not in read(p/name/'result.json') for name in selected['preserved_failures'])
        if not r.get('passed'):
            checks['fault_'+fault+'_passed_current_local']=False
            continue
        checks['fault_'+fault+'_passed_current_local']=r['passed'] and r['follower_input_boundary_passed'] and current(folder)
        checks['fault_'+fault+'_both_moving_at_injection']=all(abs(r['twists_at_injection'][bot][0])>5e-6 for bot in ('leader','follower'))
        checks['fault_'+fault+'_bounded_stop_no_replay']=r['dual_zero_latency_wall_s']<=.4 and r['no_automatic_restart'] and max(r['post_fault_joint_span_rad'])<1e-5
        checks['fault_'+fault+'_actual_wheels_and_controllers_stopped']=max(r['post_fault_max_wheel_speed_rad_s'].values())<1e-4 and all(r['controller_states'][bot][c]=='inactive' for bot in ('leader','follower') for c in ('arm_controller','mecanum_drive_controller'))
        if fault=='action':checks['external_action_canceled']=r['injected_action_accepted'] and r['injected_action_final_status'] in (5,6)
    test_log=(p/'all_tests_final.log').read_text() if (p/'all_tests_final.log').exists() else ''
    counts=re.findall(r'(\d+) passed',test_log)
    collected=re.findall(r'(\d+) tests collected', (p/'broad_collection_final.log').read_text()) if (p/'broad_collection_final.log').exists() else []
    checks['all_unit_and_ros_regressions_passed']=bool(counts and collected) and int(counts[-1])==int(collected[-1]) and int(counts[-1])>=181 and not re.search(r'\d+ failed|ERROR collecting',test_log)
    checks['final_build_passed']='1 package finished' in (p/'build_final.log').read_text() if (p/'build_final.log').exists() else False
    for name in ('legacy_cp7_transport','cp5_regression_retry'):
        if (p/name/'result.json').exists():
            r=read(p/name/'result.json');checks[name+'_passed']=r['passed']
        else:checks[name+'_passed']=False
    checks['comparison_csv_and_figures_saved']=all((p/f).exists() for f in ('comparison.csv','comparison_motion.png','comparison_forces.png'))
    remaining=[]
    for proc in Path('/proc').iterdir():
        try:
            if proc.name.isdigit() and any(v.startswith(b'IGN_PARTITION=amir_cp8_') for v in (proc/'environ').read_bytes().split(b'\0')):remaining.append(int(proc.name))
        except OSError:pass
    checks['all_cp8_owned_processes_stopped']=not remaining
    acceptance={r['axis']:r['outcome']=='TRACKED' for r in metrics if r['mode']=='WHOLE_BODY' and r.get('yaw_profile')=='filtered'}
    improvements=[axis for axis in ('lateral','yaw','pivot') if acceptance.get(axis) and any(r['mode']=='ARM_BASE' and r['axis']==axis and r['outcome']!='TRACKED' for r in metrics)]
    report={'checkpoint':'CP-8','date_jst':'2026-10-20','checks':checks,'all_checks_passed':all(checks.values()),
        'test_count':int(counts[-1]) if counts else 0,'performance_by_direction':acceptance,'newly_demonstrated_directions':improvements,
        'cp8_comparison_complete':all(checks.values()) and acceptance.get('forward',False) and acceptance.get('backward',False) and bool(improvements),
        'all_five_cooperative_motions_passed':len(acceptance)==5 and all(acceptance.values()),
        'remaining_unvalidated_cooperative_motions':[axis for axis,ok in acceptance.items() if not ok],
        'hardware_motion_gate_open':False,
        'next_phase_gate':'CP-9 documentation/measurement assessment only; cooperative yaw/pivot convergence and physical FT/load/grip/emergency-stop evidence are still required before hardware transport.',
        'remaining_owned_pids':remaining,'scope':'small simulated motions, constraint grasp; no hardware qualification'}
    paths=[*code.parent.rglob('*.py'),code.parent/'package.xml',code.parent/'README.md',*Path('tools').glob('amir_cp8*.py'),p/'README.md',p.parent/'README.md']
    (p/'source_manifest.json').write_text(json.dumps({str(f):sha(f) for f in sorted(set(paths)) if f.exists()},indent=2)+'\n')
    (p/'final_checks.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='checks'},indent=2));print('failed checks:',[k for k,v in checks.items() if not v])
    if not report['cp8_comparison_complete']:raise SystemExit(1)


if __name__=='__main__':main()
