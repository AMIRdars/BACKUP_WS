#!/usr/bin/env python3
"""Offline CP-7 evidence gates. Run after all trial process groups have ended."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-directory',type=Path,required=True)
    a=parser.parse_args();p=a.audit_directory;read=lambda f:json.loads(f.read_text())
    sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();checks={}
    checks['preexisting_tracked_diff_preserved']=subprocess.check_output(['git','diff'])==(p/'preexisting.patch').read_bytes()
    originals=[r for r in read(p.parent/'cp0/source_files.json') if Path(r['path']).suffix in ('.urdf','.xacro','.sdf','.yaml')]
    checks['original_105_models_and_yaml_preserved']=len(originals)==105 and all(sha(Path(r['path']))==r['sha256'] for r in originals)
    generated=p.parent/'cp6/generated';certificate=read(p.parent/'cp6/calibration/calibration.json')
    checks['matching_six_axis_certificate']=certificate['sensor_calibration_passed'] and len(certificate['verified_axes'])==6 and sha(generated/'follower.urdf')==certificate['urdf_sha256']
    checks['physical_load_limits_not_certified']=not certificate['load_limits_verified']
    world=ET.parse(generated/'world.sdf')
    mass=sum(float(x.text) for x in world.findall("world/model[@name='cooperative_payload']/link/inertial/mass"))
    checks['same_052kg_fixture']=abs(mass-.52)<1e-12
    for robot in ('leader','follower'):
        profile=read(generated/(robot+'_safety.json'))
        checks[robot+'_measured_profile_matches']=(profile['urdf_sha256']==sha(generated/(robot+'.urdf')) and
            profile['world_sha256']==sha(generated/'world.sdf') and profile['simulation_only'] and profile['require_grasp_for_base_motion'] and
            all(profile[k+'_sha256']==sha(Path(profile[k+'_file'])) for k in ('controller_yaml','base_controller_yaml')))
    expected={(m,d) for m in ('POSITION','ARM_ONLY','ARM_BASE') for d in (0,50,100,200)}
    comparisons=[r for r in read(p/'comparison.json') if r['delay_ms'] in (0,50,100,200)]
    checks['twelve_comparison_conditions']={(r['mode'],round(r['delay_ms'])) for r in comparisons}==expected and len(comparisons)==12
    for key in ('world_sha256','leader_urdf_sha256','follower_urdf_sha256','target_distance_m','speed_m_s','cfm'):
        checks['comparison_same_'+key]=len({r[key] for r in comparisons})==1
    for r in comparisons:
        mode,delay=r['mode'],round(r['delay_ms']);name=r['trial']
        expected_outcome=('TRACKED' if mode=='POSITION' and delay<200 or mode=='ARM_BASE' and delay==0 else
                          'NOT_TRACKED' if mode=='ARM_ONLY' and delay==0 else 'SAFE_STOP')
        checks[name+'_outcome_verified']=r['outcome']==expected_outcome
        result=read(p/name/'result.json')
        if mode!='POSITION':
            checks[name+'_leader_only_goal']=result['endpoint_states']['follower']['accepted_goals']==0 and result['endpoint_states']['leader']['accepted_goals']==1
            policy=read(p/name/'control_policy.json')
            checks[name+'_same_tuning_and_pose_monitor']=policy['virtual_mass']==[20,20] and policy['virtual_damping']==[1200,1200] and policy['monitor_pose'] and policy['preserve_orientation']
    code=Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control')
    forbidden={'tools','ros_gz_interfaces','ros_gz_sim','cooperative_transport_gazebo','cooperative_transport_bringup','amir_follower_gazebo'}
    imports=set()
    for file in code.glob('*.py'):
        for node in ast.walk(ast.parse(file.read_text())):
            if isinstance(node,ast.Import):imports.update(v.name.split('.')[0] for v in node.names)
            if isinstance(node,ast.ImportFrom) and node.module:imports.add(node.module.split('.')[0])
    checks['control_imports_independent_of_evaluation_and_simulator']=not imports.intersection(forbidden)
    package=ET.parse(code.parent/'package.xml')
    deps={e.text for e in package.getroot() if e.tag.endswith('depend')}
    checks['control_package_dependencies_independent']=not deps.intersection(forbidden)
    for fault in ('gate','interlock','supervisor','relay','audit','controller','ownership','action','dropout','preprocessor','clock'):
        name='final_guarded_fault_'+fault
        path=p/name
        if not (path/'result.json').exists():checks[name+'_exists']=False;continue
        r=read(path/'result.json')
        checks[name+'_passed']=r['passed'] and r['follower_input_boundary_passed']
        checks[name+'_both_moving_at_injection']=all(abs(r['twists_at_injection'][robot][0])>5e-6 for robot in ('leader','follower'))
        checks[name+'_bounded_stop_and_no_replay']=r['dual_zero_latency_wall_s']<=.4 and r['no_automatic_restart'] and max(r['post_fault_joint_span_rad'])<1e-5
        checks[name+'_actual_wheel_stop']=max(r['post_fault_max_wheel_speed_rad_s'].values())<1e-4
        checks[name+'_both_controllers_inactive']=all(r['controller_states'].get(robot,{}).get(c)=='inactive' for robot in ('leader','follower') for c in ('arm_controller','mecanum_drive_controller'))
        checks[name+'_current_control_source']=all(sha(code/file)==digest for file,digest in read(path/'control_source_hashes.json').items())
        if fault=='action':checks[name+'_action_canceled_or_aborted']=r['injected_action_accepted'] and r['injected_action_final_status'] in (5,6)
    inventory=read(p/'experiment_inventory.json')
    checks['every_saved_graph_local_boundary']=all(r['boundary_passed'] for r in inventory)
    checks['all_nonstartup_graphs_have_required_nodes']=all(r['required_nodes_complete'] for r in inventory if r['error_phase']!='startup')
    checks['all_144_tests_passed']='144 passed' in (p/'all_tests_complete.log').read_text()
    checks['final_build_passed']='1 package finished' in (p/'build_final.log').read_text()
    if (p/'final_transport/result.json').exists():
        r=read(p/'final_transport/result.json')
        checks['final_source_1mm_transport']=r['passed'] and r['distance_m']==.001 and r['endpoint_states']['follower']['accepted_goals']==0 and r['endpoint_states']['leader']['accepted_goals']==1
        checks['final_transport_current_control_source']=all(sha(code/file)==digest for file,digest in read(p/'final_transport/control_source_hashes.json').items())
    else:checks['final_source_1mm_transport']=False
    if (p/'cp5_regression/result.json').exists():
        reg=read(p/'cp5_regression/result.json')
        checks['cp5_runtime_regression']=reg['passed'] and reg['saturation_observed'] and len(reg['direction_tests'])==4 and all(x['passed'] for x in reg['direction_tests'])
        outs=[json.loads(line) for line in (p/'cp5_regression/command_outputs.jsonl').open()]
        checks['cp5_all_base_axes_zero']=all(max(abs(r[k]) for k in ('vx','vy','wz'))<1e-12 for r in outs)
    else:checks['cp5_runtime_regression']=False
    remaining=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if any(v.startswith(b'IGN_PARTITION=amir_cp7_') for v in (proc/'environ').read_bytes().split(b'\0')):remaining.append(int(proc.name))
        except OSError:pass
    checks['all_cp7_owned_processes_stopped']=not remaining
    paths=[*Path('src/humbleble_ws/amir_compliance_control').rglob('*.py'),Path('src/humbleble_ws/amir_compliance_control/package.xml'),
           Path('src/humbleble_ws/amir_compliance_control/README.md'),*Path('tools').glob('amir_cp7*.py'),p/'README.md',p.parent/'README.md']
    (p/'source_manifest.json').write_text(json.dumps({str(f):sha(f) for f in sorted(set(paths))},indent=2)+'\n')
    report={'checkpoint':'CP-7','date_jst':'2026-10-20','checks':checks,'all_passed':all(checks.values()),'remaining_owned_pids':remaining,
        'scope':'simulated straight transport, independent software stop and fixed-condition command delay evaluation',
        'unresolved':['CP-6 isolated timestamp rejection has no captured stamp values; causal attribution remains unproved. Explicit epoch/actuator ack and lost-home tests cover identified ordering/loss cases.',
            'Some startup/controller discovery timeouts remain fail-closed; bounded retries and all failures are recorded.',
            'Physical hardware ratings/grip/contact/physical emergency stop remain CP-9; whole-body/lateral/yaw allocation remains CP-8.']}
    (p/'final_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'checks':len(checks),'all_passed':report['all_passed'],'failed':[k for k,v in checks.items() if not v]},indent=2))
    if not report['all_passed']:raise SystemExit(1)


if __name__=='__main__':main()
