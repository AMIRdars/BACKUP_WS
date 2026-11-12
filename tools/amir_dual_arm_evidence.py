#!/usr/bin/env python3
"""Measured acceptance for physical approach and both-bases-stopped arm transport."""
import argparse
import csv
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from amir_p0_evidence import evaluate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=Path,required=True);parser.add_argument('--held',action='store_true')
    args=parser.parse_args();folder=args.case.resolve()
    result=json.loads((folder/('held_result.json' if args.held else 'result.json')).read_text())
    if not result.get('dual_arm_only'):raise ValueError('dual arm result required')
    summary,rows=evaluate(folder,'held_result.json' if args.held else 'result.json')
    active=[r for r in rows if r.get('admittance_enabled')=='True' and r.get('adm_p_actual_m_0')]
    motion=[r for r in active if r['phase']=='leader_translation']
    def values(key,size,selected=active):
        return np.array([[float(r[key+'_'+str(i)]) for i in range(size)] for r in selected])
    checks=summary['checks'];metrics={}
    checks['physical_approach_stop_before_grasp']=result['physical_base_approach']['passed'] is True and result['physical_base_approach']['stop_before_grasp'] is True
    checks['no_pose_service_placement']=result['physical_base_approach']['pose_service_calls']==0 and not (folder/'fixture_placement.jsonl').exists()
    checks['no_world_base_fixture']=not result.get('follower_base_fixture') and not (folder/'generated/base_fixture_proof.json').exists()
    checks['velocity_admittance_K_zero']=json.loads((folder/'experiment_policy.json').read_text())['controller_parameters']['virtual_stiffness']==[0.,0.]
    checks['base_assist_disabled']=all(r.get('adm_enable_base_assist')=='False' for r in active)
    events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
    stopped_at=result['physical_base_approach']['sim_stopped_s']
    after=[r for r in rows if float(r['sim_time'])>=stopped_at and r.get('leader_gt_position_0')]
    graph=json.loads((folder/'graph.json').read_text())
    object_path=values('payload_model_gt_position',3,motion)
    for robot in ('leader','follower'):
        for kind in ('base_cmd','rover_twist','base_final_reference'):
            samples=[e for e in events if e['sim_time']>=stopped_at and e['robot']==robot and e['kind']==kind]
            checks[robot+'_'+kind+'_zero_after_BASE_STOP']=bool(samples) and all(v==0. for e in samples for v in e['value'])
        body=values(robot+'_gt_position',3,after)
        drift=float(np.max(np.linalg.norm(body[:,:2]-body[0,:2],axis=1)))
        yaw=np.array([float(r[robot+'_gt_yaw_rad']) for r in after])
        yaw_drift=float(np.max(abs(np.arctan2(np.sin(yaw-yaw[0]),np.cos(yaw-yaw[0])))))
        odom_speed=float(np.max(abs(values(robot+'_odom_twist',3))))
        active_body=values(robot+'_gt_position',3)
        active_drift=float(np.max(np.linalg.norm(active_body[:,:2]-active_body[0,:2],axis=1)))
        arm=values(robot+'_p_actual',3,motion);delta=arm[-1]-arm[0]
        world_tcp=values(robot+'_tcp_world_evaluation',3,motion)
        metrics[robot]={'body_drift_from_stop_peak_m':drift,'body_yaw_drift_peak_rad':yaw_drift,
            'body_drift_during_ARM_ONLY_peak_m':active_drift,
            'odom_speed_peak':odom_speed,'local_arm_displacement_m':delta.tolist(),
            'world_TCP_displacement_m':(world_tcp[-1]-world_tcp[0]).tolist()}
        grasp_translation_residual=float(np.max(np.linalg.norm(
            (world_tcp-world_tcp[0])-(object_path-object_path[0]),axis=1)))
        metrics[robot]['TCP_object_translation_residual_peak_m']=grasp_translation_residual
        checks[robot+'_grasp_translation_residual_below_existing_2mm']=grasp_translation_residual<.002
        for suffix in ('base_cmd','rover_twist','mecanum_drive_controller/reference_unstamped','arm_controller/joint_trajectory'):
            checks[robot+'_single_writer_'+suffix]=len(graph['/'+robot+'/'+suffix])==1
        # The inherited 1 mm guard is latched when admittance is enabled.
        # BASE_STOP precedes grip seating; section 1.2 of the current request
        # gives a separate provisional 10 mm bound from that earlier epoch.
        # Neither control guard nor measured displacement is changed here.
        checks[robot+'_body_drift_from_BASE_STOP_below_spec_10mm']=drift<.01
        checks[robot+'_body_drift_from_ARM_ONLY_below_existing_1mm_guard']=active_drift<.001
        checks[robot+'_body_yaw_drift_below_existing_2mrad_guard']=yaw_drift<.002
        checks[robot+'_actual_arm_translation_matches_requested']=.9*result['requested_distance_m']<=abs(delta[0])<=1.1*result['requested_distance_m']
        checks[robot+'_actual_odom_speed_below_existing_guard']=odom_speed<.001
        for key in ('base_zero_only',):
            checks[robot+'_final_'+key]=result['final_interlock_debug'][robot].get(key) is True
        for side in ('left','right'):
            checks[robot+'_'+side+'_contact_through_motion']=all(r.get(robot+'_native_'+side+'_contact')=='True' for r in motion)
    checks['opposed_local_arm_directions']=metrics['leader']['local_arm_displacement_m'][0]>0 and metrics['follower']['local_arm_displacement_m'][0]<0
    checks['matching_world_TCP_direction']=all(v['world_TCP_displacement_m'][0]>0 for v in metrics.values())
    checks['leader_goal_reached']=result.get('leader_completed') is True
    orientation=values('payload_model_gt_orientation_xyzw',4,motion)
    dots=np.clip(abs(orientation@orientation[0]),0.,1.)
    rotation_peak=float(np.max(2*np.arccos(dots)))
    checks['object_orientation_change_below_existing_0_01rad']=rotation_peak<.01
    checks={k:bool(v) for k,v in checks.items()}
    summary.update(passed=all(checks.values()),checks=checks,dual_arm_metrics=metrics,
                   base_fixture=False,physical_approach=result['physical_base_approach'])
    summary['object_rotation_peak_rad']=rotation_peak
    (folder/'dual_arm_evidence.json').write_text(json.dumps(summary,indent=2))
    # The original CSV retains source timestamps and every acquisition channel.
    # A compact shared-time view is provided without fabricating simultaneity.
    selected=[r for r in rows if r.get('leader_p_actual_0') and r.get('follower_p_actual_0')]
    fields=['sim_time','phase','experiment_state','stop_reason','follower_reason','admittance_enabled']
    prefixes=['leader_gt_position','follower_gt_position','leader_odom_twist','follower_odom_twist',
        'leader_base_final_reference','follower_base_final_reference','leader_q_actual','follower_q_actual',
        'leader_qdot_actual','follower_qdot_actual','leader_p_actual','follower_p_actual','leader_comp_p_target_m',
        'adm_p_ref_xy_m','wrench_filtered','adm_delta_p_dot_adm_m_s','payload_model_gt_position',
        'leader_tcp_world_evaluation','follower_tcp_world_evaluation']
    fields += sorted({k for r in selected for k in r if any(k.startswith(prefix+'_') for prefix in prefixes)})
    with (folder/'dual_arm_timeseries.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(selected)
    def val(key):return np.array([float(r.get(key) or 'nan') for r in active])
    t=val('sim_time');t-=t[0]
    fig,axes=plt.subplots(8,1,figsize=(12,22),sharex=True,constrained_layout=True)
    for robot in ('leader','follower'):
        xy=values(robot+'_gt_position',3)
        axes[0].plot(t,np.linalg.norm(xy[:,:2]-xy[0,:2],axis=1)*1e6,label=robot+' physical body GT (evaluation)')
        for i,axis in enumerate(('vx','vy','wz')):
            axes[1].plot(t,val(robot+'_base_final_reference_'+str(i)),label=robot+' final '+axis)
            axes[1].plot(t,val(robot+'_odom_twist_'+str(i)),'--',label=robot+' odom '+axis)
        sign=1 if robot=='leader' else -1
        x=val(robot+'_p_actual_0');axes[2].plot(t,sign*(x-x[0])*1000,label=robot+' measured TCP projected world x')
        for i in range(5):
            q=val(robot+'_q_actual_'+str(i));axes[5].plot(t,q-q[0],label=robot+' Joint_'+str(i+1))
            axes[6].plot(t,val(robot+'_qdot_actual_'+str(i)),label=robot+' Joint_'+str(i+1))
        tree=ET.parse(folder/'generated'/(robot+'.urdf'))
        limits=[tree.find(f"joint[@name='Joint_{i}']/limit") for i in range(1,6)]
        lower=np.array([float(limit.get('lower')) for limit in limits])
        upper=np.array([float(limit.get('upper')) for limit in limits])
        q=values(robot+'_q_actual',5)
        axes[7].plot(t,np.min(np.minimum(q-lower,upper-q),axis=1),label=robot+' measured minimum joint margin')
    x=val('leader_comp_p_target_m_0');axes[2].plot(t,(x-x[0])*1000,'--',label='leader own-arm reference')
    x=val('adm_p_ref_xy_m_0');axes[2].plot(t,-(x-x[0])*1000,':',label='follower local F/T integrated reference')
    for i,axis in enumerate(('Fx','Fy')):
        axes[3].plot(t,val('wrench_filtered_'+str(i)),label=axis+' incremental force [N]')
    axv=axes[3].twinx();axv.plot(t,val('adm_delta_p_dot_adm_m_s_0'),color='k',alpha=.4,label='v_adm_x');axv.set_ylabel('Admittance velocity [m/s]')
    for key,label in [('payload_model_gt_position_0','object'),('leader_tcp_world_evaluation_0','leader grasp TCP'),('follower_tcp_world_evaluation_0','follower grasp TCP')]:
        x=val(key);axes[4].plot(t,(x-x[0])*1000,label=label+' world x change (evaluation only)')
    labels=['Body drift [um]','Final base command [m/s; rad/s]','Arm TCP travel [mm]',
            'Incremental F/T [N]','World displacement [mm]','Joint change [rad]','Measured joint speed [rad/s]','Joint margin [rad]']
    for ax,label in zip(axes,labels):ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend(fontsize=7)
    axes[-1].set_xlabel('Simulation time from first active admittance sample [s]')
    fig.suptitle('Dual ARM_ONLY: physical approach, both bases stopped, native friction contact\nMeasured acceptance passed='+str(summary['passed'])+'; no world base restraint fixture')
    for suffix in ('png','pdf'):fig.savefig(folder/('dual_arm_measured.'+suffix),dpi=140)
    plt.close(fig)
    states={'INIT':0,'BASE_APPROACH':1,'BASE_STOP':2,'DUAL_GRASP':3,'SETTLE_AND_BIAS':4,'ARM_ONLY':5,'HOLD':6,'ABORT':7}
    complete=[r for r in rows if r.get('experiment_state')]
    timeline=[]
    for row in complete:
        if not timeline or row['experiment_state']!=timeline[-1]['state']:
            timeline.append({'state':row['experiment_state'],'sim_time_s':float(row['sim_time'])})
    (folder/'experiment_fsm.json').write_text(json.dumps(timeline,indent=2))
    fig,ax=plt.subplots(figsize=(13,5),constrained_layout=True)
    ax.step([float(r['sim_time']) for r in complete],[states[r['experiment_state']] for r in complete],where='post')
    ax.set_yticks(list(states.values()),list(states));ax.set_xlabel('Simulation time [s]');ax.grid(alpha=.25)
    ax.set_title('Observed experiment states: physical approach and grasp before ARM_ONLY')
    fig.savefig(folder/'dual_arm_fsm.png',dpi=140);plt.close(fig)
    print(json.dumps(summary,indent=2));return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
