#!/usr/bin/env python3
"""Offline completion-first comparison; original experiment verdicts retained."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml
from amir_cp8r1_analyze import analyze
from amir_cp8r2_analyze import kinematic_budget


def metrics(folder):
    result=json.loads((folder/'result.json').read_text())
    rows=[json.loads(s) for s in (folder/'samples.jsonl').open()]
    moving=[r for r in rows if r['phase'] in ('rotation','hold')]
    before=[r for r in rows if r['phase']=='baseline']
    out=analyze(folder)
    out.update({k:result.get(k) for k in ('settling_time_s','final_follower_speed_m_s','grasp_loss_events',
        'follower_input_boundary_passed','arm_command_writer_counts','last_progress','both_grasps_retained_after_stop')})
    out['source_directory']=str(folder)
    out['native_contact']=result['condition'].startswith('native_')
    out['angle_error_is_completion_gate']=False
    out['center_drift_limit_m']=result.get('center_drift_limit_m')
    out['center_drift_criterion_passed']=result.get('center_drift_criterion_passed')
    out['completion_and_center_accuracy_passed']=bool(result.get('passed') and result.get('center_drift_criterion_passed'))
    out['motion_completed_in_own_odom']=result.get('states',{}).get('leader',{}).get('completed_goals')==1
    out['payload_mass_kg']=result.get('static_grasp_check',{}).get('mass_kg')
    out['follower_goals_received']=result.get('states',{}).get('follower',{}).get('accepted_goals')
    out['control_stop_reason']=result.get('states',{}).get('follower',{}).get('reason')
    out['grasp_readiness_loss_events']=result.get('grasp_readiness_loss_events',[])
    out['qualification_contact_loss_events']=result.get('qualification_contact_loss_events')
    rotation=[r for r in rows if r['phase']=='rotation']
    out['rotation_duration_observed_s']=rotation[-1]['t_s']-rotation[0]['t_s'] if rotation else None
    out['minimum_progress_rate']=min((r.get('progress_rate',1.) for r in moving),default=None)
    stopped=[r for r in rows if r['phase']=='safe_stop']
    tail_stop=[r for r in stopped if r['t_s']>stopped[0]['t_s']+.3] if stopped else []
    for name in ('leader_GT','follower_GT','object_center_GT'):
        positions=[[r[name+'_'+axis] for axis in 'xyz'] for r in tail_stop
                   if all(name+'_'+axis in r for axis in 'xyz')]
        out[name+'_post_stop_position_span_m']=float(np.linalg.norm(np.ptp(positions,axis=0))) if len(positions)>1 else None
    if before and moving:out['kinematic_center_budget']=kinematic_budget(before[-1],moving[-1])
    if result.get('object_GT_delta_yaw_rad') is not None:
        out['angle_error_deg_diagnostic']=math.degrees(result['object_GT_delta_yaw_rad'])-result['delta_deg']
    policy=yaml.safe_load((folder/'follower_policy.yaml').read_text())
    params=next(iter(policy.values()))['ros__parameters']
    out['follower_policy']=params
    for axis in ('x','y','yaw'):
        out['follower_peak_local_'+axis]=max((abs(r.get('follower_local_error_'+axis,0.)) for r in moving),default=None)
    for kind in ('raw_tared_tcp_wrench','filtered_tcp_wrench'):
        for offset,label in ((0,'force_N'),(3,'moment_Nm')):
            values=[[r['follower_'+kind+'_'+str(i)] for i in range(offset,offset+3)] for r in moving
                    if 'follower_'+kind+'_'+str(offset+2) in r]
            out[kind+'_peak_'+label]=float(max(np.linalg.norm(values,axis=1))) if values else None
    violation=(result.get('last_follower_debug') or {}).get('wrench_violation')
    out['latched_wrench_violation']=violation
    # A callback can trip and clear the active raw bias between evaluator
    # samples. Never hide that latched peak behind the 20 Hz sampled maxima.
    for label,field in (('force_N','force_norm_N'),('moment_Nm','torque_norm_Nm')):
        peaks=[out.get(kind+'_peak_'+label) for kind in ('raw_tared_tcp_wrench','filtered_tcp_wrench')]
        peaks.append((violation or {}).get(field))
        out['maximum_sampled_or_latched_'+label]=max((v for v in peaks if v is not None),default=None)
    for robot in ('leader','follower'):
        if out['native_contact']:
            out[robot+'_all_motion_contacts_present']=all(r.get(robot+'_native_contacts') for r in moving) if moving else None
            forces=[r[robot+'_finger_'+side+'_N'] for r in moving for side in ('left','right')
                    if r.get(robot+'_finger_'+side+'_N') is not None]
            out[robot+'_finger_force_min_N']=min(forces) if forces else None
            out[robot+'_finger_force_max_N']=max(forces) if forces else None
    # A secondary, explicitly named reassessment for the previous CP-8R-2
    # records, without overwriting their original centre-accuracy FAIL.
    out['completion_only_reassessment']=bool(out['motion_completed_in_own_odom'] and moving and
        result.get('both_outputs_zero_after_stop') and result.get('final_follower_speed_m_s',1.)<1e-5 and
        not result.get('grasp_loss_events',[]) and all(r.get(robot+'_grasp_state')=='attached' for r in moving for robot in ('leader','follower'))) if folder.parent.name=='cp8r2' else None
    out['grasp_event_semantics']='original record; native readiness loss may follow a shared stop; physical contact columns are independent'
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory',type=Path,required=True)
    a=p.parse_args()
    folders=[f.parent for f in sorted(a.audit_directory.glob('*/result.json')) if (f.parent/'samples.jsonl').exists() and (f.parent/'follower_policy.yaml').exists()]
    previous=a.audit_directory.parent/'cp8r2'
    folders += [previous/name for name in ('plus5_01','minus5_01')]
    data=[metrics(f) for f in folders]
    (a.audit_directory/'comparison.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    keys=sorted({k for r in data for k in r})
    with (a.audit_directory/'comparison.csv').open('w',newline='') as out:
        writer=csv.DictWriter(out,fieldnames=keys);writer.writeheader();writer.writerows(data)
    events=[]
    for record in data:
        logfile=Path(record['source_directory'])/'follower.log'
        marker='First whole-body stop context: '
        contexts=[json.loads(line.split(marker,1)[1]) for line in logfile.read_text().splitlines() if marker in line] if logfile.exists() else []
        violation=record['latched_wrench_violation']
        if not contexts and violation:
            contexts=[dict(sim_time=violation['sim_time'],reason=record.get('control_stop_reason'))]
        for context in contexts:
            event=dict(trial=record['trial'],trial_passed=record['passed'],**context)
            if violation:
                event.update(force_norm_N=violation['force_norm_N'],torque_norm_Nm=violation['torque_norm_Nm'])
                event.update(zip(('fx_N','fy_N','fz_N','tx_Nm','ty_Nm','tz_Nm'),violation['wrench']))
            events.append(event)
    with (a.audit_directory/'stop_events.csv').open('w',newline='') as out:
        writer=csv.DictWriter(out,fieldnames=sorted({k for e in events for k in e}))
        writer.writeheader();writer.writerows(events)
    print(json.dumps([{k:r.get(k) for k in ('trial','condition','passed','error','object_GT_delta_deg','grasp_loss_events')} for r in data],indent=2))


if __name__=='__main__':main()
