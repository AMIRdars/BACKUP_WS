#!/usr/bin/env python3
"""Offline-only CP-8 same-physics metrics and time-series figures."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from amir_compliance_control.wrench import LocalModel


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--audit-directory',type=Path,required=True);p.add_argument('--batch',default='final_comparison_batch.json');p.add_argument('--extra-batch');p.add_argument('--output-prefix',default='');a=p.parse_args()
    trials=json.loads((a.audit_directory/a.batch).read_text());metrics=[]
    if a.extra_batch:trials+=json.loads((a.audit_directory/a.extra_batch).read_text())
    labels=[]
    for record in trials:
        r=json.loads((a.audit_directory/record['trial']/'result.json').read_text());label=record['mode']+(' '+r.get('yaw_profile','initial') if record['mode']=='WHOLE_BODY' and r.get('yaw_profile','initial')!='initial' else '')
        if label not in labels:labels.append(label)
    fig,axes=plt.subplots(len(labels),5,figsize=(16,3*len(labels)),squeeze=False)
    forcefig,forceaxes=plt.subplots(len(labels),5,figsize=(16,3*len(labels)),squeeze=False)
    for record in trials:
        folder=a.audit_directory/record['trial'];report=json.loads((folder/'result.json').read_text())
        rows=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
        settle_rows=[v for v in rows if v['phase']=='settle']
        # A failed trial waits briefly for an already-triggered safety stop
        # before cleanup. That tail is not extra observation time for its
        # performance comparison. JSON goal deltas are captured at the exact
        # online end; CSV/plot cuts have the saved sample resolution.
        cutoff=(settle_rows[0]['sim_time']+report.get('settle_seconds',6.)) if settle_rows else float('inf')
        active=[v for v in rows if v['phase'] in ('baseline','motion','settle') and v['sim_time']<=cutoff]
        baseline=[v for v in active if v['phase']=='baseline']
        motion=[v for v in active if v['phase']=='motion']
        if not baseline or not motion:continue
        mode=record['mode'];profile=report.get('yaw_profile','initial');label=mode+(' '+profile if mode=='WHOLE_BODY' and profile!='initial' else '');axis=record['axis'];i=labels.index(label);j=['forward','backward','lateral','yaw','pivot'].index(axis)
        ax=axes[i,j];fax=forceaxes[i,j]
        start=motion[0]['sim_time'];t=np.array([v['sim_time']-start for v in active])
        component='yaw' if axis in ('yaw','pivot') else 'y' if axis=='lateral' else 'x'
        unit='mrad' if component=='yaw' else 'mm'
        for robot in ('leader','follower','payload'):
            key=robot+'_gt_'+component if robot!='payload' else 'payload_'+component
            def read(v):
                if robot=='payload' and component=='yaw':
                    x,y,z,w=[v['payload_q'+k] for k in 'xyzw'];return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                return v[key]
            initial=read(baseline[-1]);values=np.array([read(v)-initial for v in active])
            if component=='yaw':values=np.arctan2(np.sin(values),np.cos(values))
            ax.plot(t,values*1000,label=robot)
        settle=[v for v in active if v['phase']=='settle' and not v.get('fault')]
        late=[v for v in settle if v['sim_time']>=settle[-1]['sim_time']-1.] if settle else []
        outputs=[json.loads(l) for l in (folder/'command_outputs.jsonl').read_text().splitlines() if 'arm_request' not in l]
        late_outputs=[v for v in outputs if v['robot']=='follower' and late and late[0]['sim_time']<=v['sim_time']<=late[-1]['sim_time']]
        final_speed=max((float(np.linalg.norm([v['vx'],v['vy'],.1*v['wz']])) for v in late_outputs),default=None)
        final_angular_speed=max((abs(v['wz']) for v in late_outputs),default=None)
        final_linear_speed=max((float(np.hypot(v['vx'],v['vy'])) for v in late_outputs),default=None)
        final_arm_angle=max((abs(v['measured_error_yaw']) for v in late),default=None)
        final_arm=max((float(np.hypot(v['measured_error_x'],v['measured_error_y'])) for v in late),default=None)
        settled=final_speed is not None and final_arm is not None and final_speed<1e-5 and final_arm<.0001
        outcome='TRACKED' if report['passed'] and settled else 'SAFE_STOP' if report.get('safe_stop_observed') else 'NOT_TRACKED'
        ax.set_title(label+' '+axis+'\n'+outcome);ax.set_xlabel('sim time from motion (s)');ax.set_ylabel(unit);ax.grid(alpha=.3)
        biases={r:np.mean([[v[r+'_'+k] for k in ('fx','fy','fz')] for v in baseline if v['sim_time']>=baseline[-1]['sim_time']-1],axis=0) for r in ('leader','follower')}
        peaks=[];rms=[]
        for robot in ('leader','follower'):
            f=np.array([[v[robot+'_'+k] for k in ('fx','fy','fz')] for v in active]);norm=np.linalg.norm(f-biases[robot],axis=1)
            fax.plot(t,norm,label=robot)
            moving=np.array([[v[robot+'_'+k] for k in ('fx','fy','fz')] for v in motion]);d=np.linalg.norm(moving-biases[robot],axis=1)
            peaks.append(float(d.max()));rms.append(float(np.sqrt(np.mean(d*d))))
        fax.set_title(label+' '+axis+'\n'+outcome);fax.set_xlabel('sim time from motion (s)');fax.set_ylabel('wrist delta force norm (N)');fax.grid(alpha=.3)
        allocations=[json.loads(l) for l in (folder/'allocations.jsonl').read_text().splitlines()]
        times=[v['calculation_wall_ms'] for v in allocations if v.get('enabled') and v.get('calculation_wall_ms') is not None]
        iteration=[v[k] for v in allocations for k in ('iterations','candidate_iterations') if k in v]
        dt=[v.get('step_dt_s',0) for v in allocations if v.get('enabled')]
        bounded=True;previous=np.zeros(8);last_stamp=None
        scale=np.array([.03]*5+[.0005,.0005,.001]);acceleration=np.array([.15]*5+[.001,.001,.002])
        for v in allocations:
            if not v.get('enabled') or 'velocity' not in v:continue
            stamp=v['last_step_stamp_s']
            if last_stamp is not None and stamp<=last_stamp:continue
            u=np.array(v['velocity']);interval=v['step_dt_s'] if last_stamp is None else stamp-last_stamp
            bounded=bounded and bool(np.all(abs(u)<=scale+1e-10)) and bool(np.all(abs(u-previous)<=acceleration*interval+1e-10))
            previous,last_stamp=u,stamp
        pivot=None
        if axis=='pivot':
            models={r:LocalModel((a.audit_directory.parent/'cp6/generated'/(r+'.urdf')).read_text()) for r in ('leader','follower')}
            def rot(theta):return np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
            def anchor(v,r):
                q=[v[r+'_q'+str(i)+'_rad'] for i in range(1,6)]
                point=models[r].frames(q)[1][:2,3]
                return np.array([v[r+'_gt_x'],v[r+'_gt_y']])+rot(v[r+'_gt_yaw'])@point
            base=baseline[-1];end=late[-1] if late else active[-1];centre=anchor(base,'leader')
            theta=report['relative_goal_xy_yaw'][2]
            follower_target=centre+rot(theta)@(anchor(base,'follower')-centre)
            payload_target=centre+rot(theta)@(np.array([base['payload_x'],base['payload_y']])-centre)
            pivot={'centre_world_xy_m':centre.tolist(),
                'max_leader_anchor_drift_m':max(float(np.linalg.norm(anchor(v,'leader')-centre)) for v in active),
                'final_follower_anchor_target_circle_error_m':float(np.linalg.norm(anchor(end,'follower')-follower_target)),
                'final_payload_target_circle_error_m':float(np.linalg.norm(np.array([end['payload_x'],end['payload_y']])-payload_target))}

        metric={'trial':record['trial'],'mode':mode,'yaw_profile':profile,'trial_report_passed':report['passed'],'settle_seconds':report.get('settle_seconds',6),'axis':axis,'outcome':outcome,'distance_m':record['distance_m'],
            'peak_delta_force_N':max(peaks),'rms_delta_force_N':max(rms),'follower_input_boundary':report.get('follower_input_boundary_passed'),
            'leader_goals':report['endpoint_states']['leader']['accepted_goals'],'follower_goals':report['endpoint_states']['follower']['accepted_goals'],
            'offline_settled':settled,'offline_final_speed_metric_m_s':final_speed,'final_angular_speed_rad_s':final_angular_speed,'final_linear_speed_m_s':final_linear_speed,'final_arm_yaw_error_rad':final_arm_angle,'offline_final_arm_offset_xy_m':final_arm,
            'allocation_speed_acceleration_verified':bounded if mode=='WHOLE_BODY' else None,'pivot_metrics':pivot,
            'observation_cutoff_sim_s':cutoff,'sampled_cutoff_resolution_s':.06,
            'static_weight_N':report.get('static_check',{}).get('measured_weight_N'),
            'calculation_ms_p95':float(np.percentile(times,95)) if times else None,
            'calculation_ms_max':max(times) if times else None,'qp_iterations_max':max(iteration) if iteration else None,
            'integration_dt_max_s':max(dt) if dt else None,
            **{k:report.get(k) for k in ('gt_delta_xy_yaw','tracking_ratios','final_arm_offset_xy_m','final_follower_speed_m_s','error','error_phase')}}
        metrics.append(metric)
    for collection in (axes,forceaxes):
        for column in range(5):
            values=[float(y) for row in range(len(labels)) for line in collection[row,column].lines for y in line.get_ydata()]
            if values:
                low,high=min(values),max(values);padding=max((high-low)*.05,1e-6)
                for row in range(len(labels)):collection[row,column].set_ylim(low-padding,high+padding)
    for ax in (axes[0,0],forceaxes[0,0]):ax.legend(fontsize=8)
    for f,name in [(fig,'comparison_motion.png'),(forcefig,'comparison_forces.png')]:f.tight_layout();f.savefig(a.audit_directory/(a.output_prefix+name),dpi=160);plt.close(f)
    (a.audit_directory/(a.output_prefix+'comparison_metrics.json')).write_text(json.dumps(metrics,indent=2)+'\n')
    keys=['trial','mode','yaw_profile','axis','outcome','distance_m','peak_delta_force_N','rms_delta_force_N','calculation_ms_p95','calculation_ms_max','qp_iterations_max','integration_dt_max_s','final_angular_speed_rad_s','final_linear_speed_m_s','final_arm_yaw_error_rad','follower_input_boundary','leader_goals','follower_goals','error','error_phase']
    with (a.audit_directory/(a.output_prefix+'comparison.csv')).open('w') as f:
        w=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore');w.writeheader();w.writerows(metrics)
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':main()
