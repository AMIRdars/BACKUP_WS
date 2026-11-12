#!/usr/bin/env python3
"""Bounded-memory comparison reports from full, streamed experimental logs.

The consumed publication sequence joins controller data to observer data.
Missing evaluator copies stay unknown; the controller's source age is still
available. Ground truth is processed separately and never returned to control.
"""
import argparse,csv,json,math
from pathlib import Path
import numpy as np,yaml
from amir_compliance_control.rotation_observer import ProjectedYaw,wrap_pi


def lines(path):
    with path.open() as f:
        for line in f:
            if line.strip():yield json.loads(line)


class SequenceJoin:
    def __init__(self,path):self.rows=iter(lines(path));self.current=None
    def get(self,sequence):
        if sequence is None:return None
        while self.current is None or self.current.get('publication_sequence',-1)<sequence:
            self.current=next(self.rows,None)
            if self.current is None:return None
        return self.current if self.current.get('publication_sequence')==sequence else None


class TimeJoin:
    def __init__(self,path):self.rows=iter(lines(path));self.current=None;self.future=next(self.rows,None)
    def get(self,t):
        while self.future is not None and self.future['timestamp']<=t:
            self.current=self.future;self.future=next(self.rows,None)
        return self.current


class CsvStream:
    def __init__(self,path):self.file=path.open('w',newline='');self.writer=None
    def write(self,row):
        if self.writer is None:self.writer=csv.DictWriter(self.file,fieldnames=list(row));self.writer.writeheader()
        self.writer.writerow(row)
    def close(self):self.file.close()


def baseline_reference(run):
    count=0;yaw_sum=0.;point_sum=np.zeros(2);last=None
    for row in lines(run/'comparison_local.jsonl'):
        if row['phase']!='baseline':break
        d=row['debug'];t=row['timestamp']
        if t==last or 'recentering_position_footprint_m' not in d or 'follower_odom_pose' not in row:continue
        last=t;p=np.array(d['recentering_position_footprint_m']);x,y,yaw=row['follower_odom_pose'];c,s=np.cos(yaw),np.sin(yaw)
        count+=1;yaw_sum+=yaw;point_sum += [x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]]
    return count,yaw_sum,point_sum


def analyze(run):
    result=json.loads((run/'comparison_result.json').read_text());params=next(iter(yaml.safe_load((run/'follower_policy.yaml').read_text()).values()))['ros__parameters']
    g=ProjectedYaw((run/'generated/follower.urdf').read_text(),orientation_envelope_rad=params['rotation_error_limit_rad'],joint_margin_rad=params['joint_margin_rad'],heading_frame='grasp_heading')
    report=dict(run=str(run),mode=result['mode'],delta_deg=result['delta_deg'],passed=result['passed'],
        goal_count=len(result['leader_goal_values_rad']),completed_goals=result.get('completed_goals',result.get('final_states',{}).get('leader',{}).get('completed_goals',0)),
        angle_accuracy_is_diagnostic=True,both_fixed_grasps_retained=result['both_fixed_grasps_retained'],
        stop_and_hold_confirmed=result['stop_and_hold_confirmed'],reason=result.get('reason'),analysis_memory_bounded=True)
    states=CsvStream(run/'state_timeseries.csv');measurements=CsvStream(run/'measurements.csv');timing=CsvStream(run/'consumed_observation.csv')
    join=SequenceJoin(run/'observer.jsonl');last=None;count=0;force2=moment2=0.;maximum={};minimum={};missing=0;nonzero=0;moving_first=moving_last=None
    baseline_count,baseline_yaw,baseline_point=baseline_reference(run)
    max_yaw=max_point=0.;allocation_counts=set();age_hist={};timing_count=0
    def high(key,value):maximum[key]=max(maximum.get(key,-math.inf),float(value))
    def low(key,value):minimum[key]=min(minimum.get(key,math.inf),float(value))
    for row in lines(run/'comparison_local.jsonl'):
        d=row['debug'];t=row['timestamp'];state=dict(t_s=t,wall_s=row['wall_s'],phase=row['phase'],follower_debug_state=d['state'],follower_debug_reason=d.get('reason',''))
        for robot in ('leader','follower'):
            s=row.get('states',{}).get(robot,{})
            state[robot+'_state']=s.get('state');state[robot+'_reason']=s.get('reason')
            for axis,u_actual in zip(('vx_m_s','vy_m_s','wz_rad_s'),row.get('outputs',{}).get(robot,[None]*3)):state[robot+'_received_output_'+axis]=u_actual
            state[robot+'_grasp']=row['grasp'].get(robot)
        states.write(state)
        consumed=d.get('consumed_observation')
        if consumed:
            timing.write(dict(phase=row['phase'],debug_sim_s=t,state=d['state'],reason=d.get('reason',''),**consumed))
            if row['phase']!='safe_stop' and d['state']!='SAFE_STOP':
                timing_count+=1;age=int(consumed['source_age_ns']);age_hist[age]=age_hist.get(age,0)+1
        if t==last or 'recentering_position_footprint_m' not in d:continue
        last=t
        if not consumed:raise ValueError('streaming report needs recorded controller consumption timestamps')
        o=join.get(consumed.get('publication_sequence'));position=np.array(d['recentering_position_footprint_m']);u=np.array(d['base_twist']);low_error=np.array(d.get('low_error_xy_yaw') or [0.]*3)
        translation=np.array(params['base_gains'][:2])*low_error[:2];coupling=np.r_[u[:2]-translation,0.]+np.cross([0.,0.,u[2]],position)
        raw=np.array(d.get('raw_tared_tcp_wrench') or [0.]*6);residual=np.array(d['residual_twist']);age=consumed['source_age_ns']*1e-9
        margin=(float(np.min(np.r_[np.array(o['q_rad'])-g.model.lower,g.model.upper-np.array(o['q_rad'])])) if o and 'q_rad' in o else d['joint_margin_rad']+params['joint_margin_rad'])
        value=dict(t_s=t,phase=row['phase'],state=d['state'],mode=result['mode'],e_xy_m=float(np.linalg.norm(d['measured_error_xy_yaw'][:2])),e_yaw_rad=d['measured_error_xy_yaw'][2],internal_arm_yaw_rad=d.get('internal_yaw_target_rad'),vx_m_s=u[0],vy_m_s=u[1],wz_rad_s=u[2],rotary_coupling_residual_m_s=float(np.linalg.norm(coupling)),force_N=float(np.linalg.norm(raw[:3])),moment_Nm=float(np.linalg.norm(raw[3:])),yaw_valid=None if o is None else bool(o['yaw_valid'] and -20_000_000<=consumed['source_age_ns']<=50_000_000),observer_source_age_s=age,observer_joint_alignment_ns=d.get('observer_joint_alignment_ns'),consumed_publication_sequence=consumed.get('publication_sequence'),consumed_snapshot_recorded=o is not None,grasp_leader=row['grasp'].get('leader'),grasp_follower=row['grasp'].get('follower'),joint_margin_rad=margin,task_translation_residual_m_s=float(np.linalg.norm(residual[:3])),task_rotation_residual_rad_s=float(np.linalg.norm(residual[3:])),explicit_compensation_norm_m_s=float(np.linalg.norm(d['explicit_compensation_xy'])),task_compliance_allocation_budget=d.get('task_compliance_allocation_budget'),measured_allocation_budget=d.get('measured_allocation_budget'))
        if result['mode']=='WHOLE_BODY':value.update(allocation_base_output_difference=float(np.linalg.norm(u-np.array(d['velocity'][5:]))),allocation_variables=len(d['velocity']))
        for robot in ('leader','follower'):
            value[robot+'_state']=row.get('states',{}).get(robot,{}).get('state')
            for axis,u_actual in zip(('vx_m_s','vy_m_s','wz_rad_s'),row.get('outputs',{}).get(robot,[None]*3)):value[robot+'_received_output_'+axis]=u_actual
        if 'follower_odom_pose' in row:
            x,y,yaw=row['follower_odom_pose'];c,s=np.cos(yaw),np.sin(yaw);point=np.array([x+c*position[0]-s*position[1],y+s*position[0]+c*position[1]])
            value.update(measured_base_yaw_rad=yaw,tcp_odom_x_m=point[0],tcp_odom_y_m=point[1])
            if baseline_count and row['phase']!='safe_stop':max_yaw=max(max_yaw,abs(wrap_pi(yaw-baseline_yaw/baseline_count)));max_point=max(max_point,float(np.linalg.norm(point-baseline_point/baseline_count)))
        measurements.write(value)
        if row['phase']=='safe_stop':continue
        count+=1;force2+=value['force_N']**2;moment2+=value['moment_Nm']**2;missing+=int(o is None)
        for key,field in [('maximum_e_xy_m','e_xy_m'),('maximum_force_N','force_N'),('maximum_moment_Nm','moment_Nm'),('maximum_rotary_coupling_residual_m_s','rotary_coupling_residual_m_s'),('maximum_task_translation_residual_m_s','task_translation_residual_m_s'),('maximum_task_rotation_residual_rad_s','task_rotation_residual_rad_s'),('maximum_explicit_compensation_norm_m_s','explicit_compensation_norm_m_s')]:high(key,value[field])
        high('maximum_e_yaw_rad',abs(value['e_yaw_rad']));high('maximum_commanded_wz_rad_s',abs(u[2]));low('minimum_joint_margin_rad',margin)
        if d.get('observer_joint_alignment_ns') is not None:high('maximum_observer_joint_alignment_ns',d['observer_joint_alignment_ns'])
        if result['mode']=='WHOLE_BODY':allocation_counts.add(value['allocation_variables']);high('maximum_allocation_base_output_difference',value['allocation_base_output_difference'])
        if row['phase'] in ('positive','negative'):
            if moving_first is None:moving_first=t
            moving_last=t;nonzero+=int(abs(u[2])>0.)
    for writer in (states,measurements,timing):writer.close()
    report.update(maximum);report.update(minimum);report.update(samples=count,missing_consumed_snapshot_rows=missing,exact_consumed_observation_join=not missing,nonzero_yaw_samples=nonzero)
    if count:report.update(force_RMS_N=math.sqrt(force2/count),moment_RMS_Nm=math.sqrt(moment2/count))
    if baseline_count:report.update(maximum_measured_base_yaw_rad=max_yaw,maximum_odom_fk_point_xy_drift_m=max_point)
    if allocation_counts:report['allocation_variables']=sorted(allocation_counts)
    if moving_first is not None:
        n=bad=0
        for o in lines(run/'observer.jsonl'):
            if moving_first<=o['timestamp']<=moving_last:n+=1;bad+=int(o['yaw_valid'] is not True)
        report.update(rotation_observer_samples=n,rotation_invalid_samples=bad)
    if age_hist:
        total=sum(age_hist.values());target=math.ceil(.99*total);acc=0;p99=None
        for age,n in sorted(age_hist.items()):
            acc+=n
            if acc>=target:p99=age;break
        report.update(maximum_consumed_yaw_age_ns=max(age_hist),p99_consumed_yaw_age_ns=p99,consumed_yaw_timing_rows=timing_count)
    evaluate(run,g,report)
    (run/'metrics.json').write_text(json.dumps(report,indent=2)+'\n');return report


def evaluate(run,g,report):
    join=TimeJoin(run/'observer.jsonl');zero=None;home=None;max_center=0.;min_gap=math.inf;max_trans=max_rot=0.;max_height=max_tilt=0.;gt_rows=0;endpoints={};writer=CsvStream(run/'evaluation_only.csv')
    reference_join=TimeJoin(run/'observer.jsonl')
    for row in lines(run/'evaluation_only.jsonl'):
        if row['phase']!='baseline':break
        zero=np.array(row['object_pose_GT']);o=reference_join.get(row['timestamp'])
        if o and 'q_rad' in o and abs(o['joint_stamp']-row['timestamp'])<=.02:
            position,sensor=g.model.frames(o['q_rad']);tcp=sensor@g.tcp_offset;tcp[:3,3]=position
            home=np.linalg.inv(zero)@np.array(row['follower_base_pose_GT'])@tcp
    for row in lines(run/'evaluation_only.jsonl'):
        t=row['timestamp'];T=np.array(row['object_pose_GT']);o=join.get(t)
        local=None
        if o and 'q_rad' in o and abs(o['joint_stamp']-t)<=.02:
            position,sensor=g.model.frames(o['q_rad']);tcp=sensor@g.tcp_offset;tcp[:3,3]=position
            local=np.linalg.inv(T)@np.array(row['follower_base_pose_GT'])@tcp
        if zero is None:continue
        change=wrap_pi(float(np.arctan2(T[1,0],T[0,0])-np.arctan2(zero[1,0],zero[0,0])));drift=float(np.linalg.norm(T[:3,3]-zero[:3,3]))
        c,s=np.cos(change),np.sin(change);rz=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        tilt=float(np.arccos(np.clip((np.trace((rz@zero[:3,:3]).T@T[:3,:3])-1)/2,-1,1)))
        height=abs(float(T[2,3]-zero[2,3]))
        writer.write(dict(t_s=t,phase=row['phase'],object_x_m=T[0,3],object_y_m=T[1,3],object_z_m=T[2,3],object_yaw_change_rad=change,center_drift_m=drift,object_height_change_m=height,object_roll_pitch_change_rad=tilt,base_surface_gap_m=row['base_surface_gap_m']))
        if row['phase']=='safe_stop':continue
        gt_rows+=1;max_center=max(max_center,drift);max_height=max(max_height,height);max_tilt=max(max_tilt,tilt);min_gap=min(min_gap,row['base_surface_gap_m']);endpoints[row['phase']]=change
        if home is not None and local is not None:
            error=np.linalg.inv(home)@local;max_trans=max(max_trans,float(np.linalg.norm(error[:3,3])));max_rot=max(max_rot,float(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1))))
    writer.close()
    if gt_rows:
        report.update(maximum_object_center_drift_m=max_center,maximum_object_height_change_m=max_height,
            maximum_object_roll_pitch_change_rad=max_tilt,minimum_base_surface_gap_m=min_gap,maximum_fixed_grasp_translation_change_m=max_trans,maximum_fixed_grasp_rotation_change_rad=max_rot)
        for phase in ('positive','negative','hold'):
            if phase in endpoints:report[phase+'_object_yaw_from_initial_rad']=endpoints[phase]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path);a=p.parse_args();reports=[analyze(r.resolve()) for r in a.runs]
    if a.output:
        keys=list(dict.fromkeys(k for r in reports for k in r))
        with a.output.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(reports)
    print(json.dumps(reports,indent=2))

if __name__=='__main__':main()
