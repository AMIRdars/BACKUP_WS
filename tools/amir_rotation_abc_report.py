#!/usr/bin/env python3
"""Source-time aligned A/B/C/D diagnostics. Evaluation GT never enters control."""
import argparse
import bisect
import csv
import json
from pathlib import Path
import numpy as np
import yaml
from amir_compliance_control.rotation_observer import ProjectedYaw,wrap_pi


def read(file):
    return [json.loads(line) for line in file.read_text().splitlines()]


def analyze(run):
    result=json.loads((run/'comparison_result.json').read_text())
    rows=read(run/'comparison_local.jsonl');stream=sorted(read(run/'observer.jsonl'),key=lambda o:o['timestamp'])
    states=[]
    # Retain duplicate simulation timestamps: a wall watchdog can stop the
    # robots while simulation time is paused. Cleared allocator debug must
    # not erase those stop/hold acknowledgements from the output CSV.
    for row in rows:
        d=row['debug'];value=dict(t_s=row['timestamp'],wall_s=row['wall_s'],phase=row['phase'],
            follower_debug_state=d['state'],follower_debug_reason=d['reason'])
        for robot in ('leader','follower'):
            state=row.get('states',{}).get(robot,{})
            value[robot+'_state']=state.get('state');value[robot+'_reason']=state.get('reason')
            for axis,u in zip(('vx_m_s','vy_m_s','wz_rad_s'),row['outputs'].get(robot,[None]*3)):
                value[robot+'_received_output_'+axis]=u
            value[robot+'_grasp']=row['grasp'].get(robot)
        states.append(value)
    if states:
        with (run/'state_timeseries.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(states[0]));writer.writeheader();writer.writerows(states)
    stamps=[o['timestamp'] for o in stream];gt=read(run/'evaluation_only.jsonl')
    by_sequence={o['publication_sequence']:o for o in stream if 'publication_sequence' in o}
    params=next(iter(yaml.safe_load((run/'follower_policy.yaml').read_text()).values()))['ros__parameters']
    g=ProjectedYaw((run/'generated/follower.urdf').read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    data=[];last=None
    for row in rows:
        d=row['debug'];t=row['timestamp']
        if t==last or 'recentering_position_footprint_m' not in d:continue
        last=t
        consumed=d.get('consumed_observation',{})
        sequence=consumed.get('publication_sequence')
        o=by_sequence[sequence] if sequence in by_sequence else stream[max(0,bisect.bisect_right(stamps,t)-1)]
        u=np.array(d['base_twist']);p=np.array(d['recentering_position_footprint_m'])
        low=np.array(d.get('low_error_xy_yaw') or [0.]*3)
        translation=np.array(params['base_gains'][:2])*low[:2]
        residual=np.r_[u[:2]-translation,0.]+np.cross([0.,0.,u[2]],p)
        raw=d.get('raw_tared_tcp_wrench') or [0.]*6
        m=g.measure(o['q_rad'],compute_jacobian=False) if 'q_rad' in o else None
        age=consumed['source_age_ns']*1e-9 if 'source_age_ns' in consumed else t-o.get('joint_stamp',t)
        value=dict(t_s=t,phase=row['phase'],state=d['state'],mode=result['mode'],
            e_xy_m=float(np.linalg.norm(d['measured_error_xy_yaw'][:2])),e_yaw_rad=d['measured_error_xy_yaw'][2],
            internal_arm_yaw_rad=d.get('internal_yaw_target_rad'),vx_m_s=u[0],vy_m_s=u[1],wz_rad_s=u[2],
            rotary_coupling_residual_m_s=float(np.linalg.norm(residual)),
            force_N=float(np.linalg.norm(raw[:3])),moment_Nm=float(np.linalg.norm(raw[3:])),
            yaw_valid=None if sequence is not None and sequence not in by_sequence else
                bool(o['yaw_valid'] and -20_000_000<=round(age*1e9)<=50_000_000),
            observer_source_age_s=age,observer_joint_alignment_ns=d.get('observer_joint_alignment_ns'),
            consumed_publication_sequence=sequence,
            consumed_snapshot_recorded=sequence in by_sequence,
            grasp_leader=row['grasp'].get('leader'),grasp_follower=row['grasp'].get('follower'),
            joint_margin_rad=None if m is None else m['joint_margin_rad'])
        residual_twist=np.array(d['residual_twist'])
        value.update(task_translation_residual_m_s=float(np.linalg.norm(residual_twist[:3])),
            task_rotation_residual_rad_s=float(np.linalg.norm(residual_twist[3:])),
            explicit_compensation_norm_m_s=float(np.linalg.norm(d['explicit_compensation_xy'])))
        if result['mode']=='WHOLE_BODY':
            value['allocation_base_output_difference']=float(np.linalg.norm(u-np.array(d['velocity'][5:])))
            value['allocation_variables']=len(d['velocity'])
        # Twist has no source header: these are the latest actual interlock
        # outputs received by the evaluator at this debug callback.
        for robot in ('leader','follower'):
            value[robot+'_state']=row.get('states',{}).get(robot,{}).get('state')
            for axis,u_actual in zip(('vx_m_s','vy_m_s','wz_rad_s'),row['outputs'].get(robot,[None]*3)):
                value[robot+'_received_output_'+axis]=u_actual
        if 'follower_odom_pose' in row:
            x,y,yaw=row['follower_odom_pose'];c,s=np.cos(yaw),np.sin(yaw)
            value.update(measured_base_yaw_rad=yaw,tcp_odom_x_m=x+c*p[0]-s*p[1],tcp_odom_y_m=y+s*p[0]+c*p[1])
        data.append(value)
    active=[r for r in data if r['phase']!='safe_stop'];moving=[r for r in active if r['phase'] in ('positive','negative')]
    base=[r for r in active if r['phase']=='baseline']
    report=dict(run=str(run),mode=result['mode'],passed=result['passed'],samples=len(data),
        goal_count=len(result['leader_goal_values_rad']),completed_goals=result.get('completed_goals',0),
        angle_accuracy_is_diagnostic=True,both_fixed_grasps_retained=result['both_fixed_grasps_retained'],
        exact_consumed_observation_join=bool(data and all(r['consumed_publication_sequence'] in by_sequence for r in data)),
        missing_consumed_snapshot_rows=sum(r['consumed_publication_sequence'] is not None and not r['consumed_snapshot_recorded'] for r in data),
        stop_and_hold_confirmed=result['stop_and_hold_confirmed'],reason=result.get('reason'))
    if active:
        report.update(maximum_e_xy_m=max(r['e_xy_m'] for r in active),maximum_e_yaw_rad=max(abs(r['e_yaw_rad']) for r in active),
            maximum_force_N=max(r['force_N'] for r in active),maximum_moment_Nm=max(r['moment_Nm'] for r in active),
            force_RMS_N=float(np.sqrt(np.mean([r['force_N']**2 for r in active]))),
            moment_RMS_Nm=float(np.sqrt(np.mean([r['moment_Nm']**2 for r in active]))),
            maximum_commanded_wz_rad_s=max(abs(r['wz_rad_s']) for r in active),
            minimum_joint_margin_rad=min(r['joint_margin_rad'] for r in active if r['joint_margin_rad'] is not None),
            maximum_rotary_coupling_residual_m_s=max(r['rotary_coupling_residual_m_s'] for r in active),
            maximum_task_translation_residual_m_s=max(r['task_translation_residual_m_s'] for r in active),
            maximum_task_rotation_residual_rad_s=max(r['task_rotation_residual_rad_s'] for r in active),
            maximum_explicit_compensation_norm_m_s=max(r['explicit_compensation_norm_m_s'] for r in active),
            maximum_observer_joint_alignment_ns=max(r['observer_joint_alignment_ns'] for r in active if r['observer_joint_alignment_ns'] is not None),
            nonzero_yaw_samples=int(sum(abs(r['wz_rad_s'])>0. for r in moving)))
        if result['mode']=='WHOLE_BODY':
            report.update(allocation_variables=sorted(set(r['allocation_variables'] for r in active)),
                maximum_allocation_base_output_difference=max(r['allocation_base_output_difference'] for r in active))
    if moving:
        window=[o for o in stream if moving[0]['t_s']<=o['timestamp']<=moving[-1]['t_s']]
        report.update(rotation_observer_samples=len(window),rotation_invalid_samples=sum(o['yaw_valid'] is not True for o in window))
    if base and all('measured_base_yaw_rad' in r for r in active):
        zero=np.mean([r['measured_base_yaw_rad'] for r in base]);home=np.mean([[r['tcp_odom_x_m'],r['tcp_odom_y_m']] for r in base],axis=0)
        report.update(maximum_measured_base_yaw_rad=max(abs(wrap_pi(r['measured_base_yaw_rad']-zero)) for r in active),
            maximum_odom_fk_point_xy_drift_m=max(float(np.linalg.norm(np.array([r['tcp_odom_x_m'],r['tcp_odom_y_m']])-home)) for r in active))
    static=[r for r in gt if r['phase']=='baseline'];gtactive=[r for r in gt if r['phase']!='safe_stop']
    if static:
        zero=np.array(static[-1]['object_pose_GT']);yaw0=np.arctan2(zero[1,0],zero[0,0])
        with (run/'evaluation_only.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=['t_s','phase','object_x_m','object_y_m','object_z_m',
                'object_yaw_change_rad','center_drift_m','base_surface_gap_m'])
            writer.writeheader()
            for row in gt:
                T=np.array(row['object_pose_GT'])
                writer.writerow(dict(t_s=row['timestamp'],phase=row['phase'],object_x_m=T[0,3],object_y_m=T[1,3],object_z_m=T[2,3],
                    object_yaw_change_rad=wrap_pi(np.arctan2(T[1,0],T[0,0])-yaw0),
                    center_drift_m=float(np.linalg.norm(T[:3,3]-zero[:3,3])),base_surface_gap_m=row['base_surface_gap_m']))
        report.update(maximum_object_center_drift_m=max(float(np.linalg.norm(np.array(r['object_pose_GT'])[:3,3]-zero[:3,3])) for r in gtactive),
                      minimum_base_surface_gap_m=min(r['base_surface_gap_m'] for r in gtactive))
        for phase in ('positive','negative','hold'):
            group=[r for r in gtactive if r['phase']==phase]
            if group:
                T=np.array(group[-1]['object_pose_GT']);report[phase+'_object_yaw_from_initial_rad']=wrap_pi(np.arctan2(T[1,0],T[0,0])-yaw0)
        transforms=[]
        for row in gtactive:
            idx=max(0,bisect.bisect_right(stamps,row['timestamp'])-1);o=stream[idx]
            if 'q_rad' not in o or abs(o['joint_stamp']-row['timestamp'])>.02:continue
            position,sensor=g.model.frames(o['q_rad']);tcp=sensor@g.tcp_offset;tcp[:3,3]=position
            T=np.linalg.inv(np.array(row['object_pose_GT']))@np.array(row['follower_base_pose_GT'])@tcp
            transforms.append((row['phase'],T))
        home=next((T for phase,T in reversed(transforms) if phase=='baseline'),None)
        if home is not None:
            errors=[np.linalg.inv(home)@T for phase,T in transforms]
            report.update(maximum_fixed_grasp_translation_change_m=max(float(np.linalg.norm(T[:3,3])) for T in errors),
                maximum_fixed_grasp_rotation_change_rad=max(float(np.arccos(np.clip((np.trace(T[:3,:3])-1)/2,-1,1))) for T in errors))
    (run/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
    if data:
        keys=list(dict.fromkeys(k for row in data for k in row))
        with (run/'measurements.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(data)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();reports=[analyze(run.resolve()) for run in a.runs]
    if a.output:
        keys=list(dict.fromkeys(k for r in reports for k in r))
        with a.output.open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(reports)
    print(json.dumps(reports,indent=2))


if __name__=='__main__':main()
