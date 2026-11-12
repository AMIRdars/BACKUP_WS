#!/usr/bin/env python3
"""Check AF commands, measured-state identities, boundaries and preservation."""
import argparse,json,csv,hashlib,difflib,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,default=ROOT/'audits/arm_only_20261107');p.add_argument('--case',action='append');a=p.parse_args();r=a.directory.resolve()
    proof_cases=['af1_pose_hold','af2_5mm_proof','af3_motion_dropout_proof_retry',
                 'af3_motion_force_proof','af3_motion_saturation_proof','af3_motion_constraint_proof']
    cases=a.case or (proof_cases if all((r/c/'result.json').exists() for c in proof_cases) else
        ['af1_pose_hold','af2_5mm_pose_hold_ref',*['af3_pose_hold_'+f+'_clean' for f in ('dropout','force','saturation','constraint')]])
    checks={};limitations=[];request=json.loads((r/'request.json').read_text())
    for f,h in request['baseline'].items():
        if not f.endswith(('/setup.py','/transport.py')):checks['preserved:'+f]=sha(ROOT/f)==h
    checks['tracked_preexisting_diff_preserved']=subprocess.check_output(['git','diff','--binary'],cwd=ROOT)==(r/'preexisting.patch').read_bytes()
    for case in cases:
        folder=r/case;j=json.loads((folder/'result.json').read_text());g=json.loads((folder/'graph.json').read_text());rows=list(csv.DictReader((folder/'timeseries.csv').open()));active=[x for x in rows if x.get('admittance_enabled')=='True'];checks[case+':passed']=j['passed']
        checks[case+':stopped_hold']=j['safe_stop_verified'] and (j['stage']=='AF-1' or j['grasp_held_after_stop'])
        if 'base_stop_evidence' in j:
            checks[case+':actual_base_stop']=len(j['base_stop_evidence'])==2 and all(e['stopped']
                and e['wheel_velocity_peak_rad_s']<1e-4 and e['ground_truth_position_span_m']<1e-6
                for e in j['base_stop_evidence'].values())
        if case.startswith('af3_motion'):
            checks[case+':injected_during_measured_motion']=j.get('leader_odom_velocity_at_injection',0)>0 and j.get('leader_command_at_injection',[0])[0]>0
            checks[case+':stop_latency']=0<=j.get('stop_latency_wall_s',float('inf'))<=.4
        if 'remaining_owned_pids' in j:checks[case+':cleanup_complete']=not j['remaining_owned_pids']
        events=[json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
        commands=[e for e in events if e['robot']=='follower' and e['kind'] in ('base_cmd','rover_twist')]
        checks[case+':all_base_zero']=bool(commands) and all(v==0. for e in commands for v in e['value'])
        for topic,node in [('/follower/base_cmd','/follower/follower_arm_only'),('/follower/rover_twist','/follower/actuator_interlock'),('/follower/arm_controller/joint_trajectory','/follower/actuator_interlock')]:
            checks[case+':writer:'+topic]=len(g[topic])==1 and g[topic][0]['node']==node
        for node in ('follower_arm_only','wrench_preprocessor'):
            key='/follower/'+node+'/subscriptions'
            if key not in g:
                checks[case+':local_inputs:'+node]=False
                limitations.append(case+': missing graph subscription snapshot for '+node)
                continue
            topics=[v[0] for v in g[key]]
            checks[case+':local_inputs:'+node]=not any(t.startswith('/leader/') or 'ground_truth' in t or 'goal' in t or t.startswith('/world/') for t in topics)
        def vec(row,key,count):return np.array([float(row[key+'_'+str(i)]) for i in range(count)])
        measured=[]
        for x in active:
            if not x.get('adm_e_arm_m_0'):continue
            home=vec(x,'adm_p_home_m',3);actual=vec(x,'adm_p_actual_m',3);ref=vec(x,'adm_p_ref_xy_m',2)
            measured.append(np.max(np.abs(np.r_[vec(x,'adm_e_arm_m',3)-(actual-home),
                vec(x,'adm_e_track_xy_m',2)-(ref-actual[:2]),ref-home[:2]-vec(x,'adm_delta_p_adm_m',2)])))
        checks[case+':distinct_state_identities']=bool(measured) and max(measured)<1e-12
        home_samples=[vec(x,'adm_p_home_m',3) for x in active if x.get('adm_p_home_m_0')]
        checks[case+':home_latched']=bool(home_samples) and np.max(np.ptp(home_samples,axis=0))<1e-12
        checks[case+':only_arm_and_rotation_off']=all(x.get('adm_mode')=='ARM_ONLY' and x.get('adm_rotation_tracking_enabled')=='False' for x in active)
        checks[case+':joint_speed_bound']=all(np.max(np.abs(vec(x,'adm_qdot_cmd_rad_s',5)))<=.0300000001 for x in active if x.get('adm_qdot_cmd_rad_s_0'))
        checks[case+':fixture_proof']=all(v.get('only_controller_path_changed') and v.get('world_unchanged') for v in json.loads((folder/'generated/fixture_relocation.json').read_text()).values())
    files=[*map(Path,['src/humbleble_ws/amir_compliance_control/setup.py',
        'src/humbleble_ws/amir_compliance_control/amir_compliance_control/interlock.py',
        'src/humbleble_ws/amir_compliance_control/amir_compliance_control/safety.py',
        'src/humbleble_ws/amir_compliance_control/amir_compliance_control/transport.py',
        'src/humbleble_ws/amir_compliance_control/launch/guarded_safety.launch.py',
        'src/humbleble_ws/amir_compliance_control/amir_compliance_control/strict_arm_only.py',
        'src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_only_dls.py',
        'src/humbleble_ws/amir_compliance_control/launch/arm_only_af.launch.py',
        'src/humbleble_ws/amir_compliance_control/test/test_strict_arm_only.py',
        'tools/amir_af_trial.py','tools/amir_af_report.py','tools/amir_af_verify.py','tools/amir_af_plot.py','tools/test_af_pose_hold.py'])]
    manifest=[];patch=[]
    for f in files:
        source=ROOT/f;old=r/'before'/f
        dest=r/'source_snapshot'/f;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read_bytes())
        manifest.append({'path':str(f),'status':'modified' if old.exists() else 'added','sha256':sha(source),
                         'before_sha256':sha(old) if old.exists() else None})
        patch.extend(difflib.unified_diff(old.read_text().splitlines(True) if old.exists() else [],source.read_text().splitlines(True),fromfile='a/'+str(f) if old.exists() else '/dev/null',tofile='b/'+str(f)))
    (r/'changes.json').write_text(json.dumps(manifest,indent=2));(r/'implementation.patch').write_text(''.join(patch))
    checks={k:bool(v) for k,v in checks.items()}
    report={'all_passed':all(checks.values()),'checks':checks,'cases':cases,'limitations':limitations}
    (r/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps({'checks':len(checks),'all_passed':report['all_passed'],'failed':[k for k,v in checks.items() if not v]},ensure_ascii=False))
    return 0 if report['all_passed'] else 1
if __name__=='__main__':raise SystemExit(main())
