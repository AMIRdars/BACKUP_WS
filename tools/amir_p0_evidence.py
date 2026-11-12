#!/usr/bin/env python3
"""Export P0 measured-state checks and standalone figures (evaluation only)."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def evaluate(folder, result_name='result.json'):
    result=json.loads((folder/result_name).read_text())
    rows=list(csv.DictReader((folder/'timeseries.csv').open()))
    graph=json.loads((folder/'graph.json').read_text())
    active=[r for r in rows if r.get('admittance_enabled')=='True' and r.get('adm_p_actual_m_0')]
    if not active:raise ValueError('no actual arm feedback: '+str(folder))
    def array(key,size):
        return np.array([[float(r[key+'_'+str(i)]) for i in range(size)] for r in active])
    actual,home,ref=array('adm_p_actual_m',3),array('adm_p_home_m',3),array('adm_p_ref_xy_m',2)
    drift=array('adm_base_drift_xy_yaw',3)
    checks={'result_passed':result.get('passed') is True,
        'safe_stop_verified':result.get('safe_stop_verified') is True,
        'base_cmd_all_zero':result.get('follower_max_abs_base_cmd')==0.,
        'home_latched_once':np.max(np.ptp(home,axis=0))<1e-12,
        'e_arm_is_actual_minus_home':np.max(abs(array('adm_e_arm_m',3)-(actual-home)))<1e-12,
        'e_track_is_reference_minus_actual':np.max(abs(array('adm_e_track_xy_m',2)-(ref-actual[:,:2])))<1e-12,
        'adm_is_reference_minus_home':np.max(abs(array('adm_delta_p_adm_m',2)-(ref-home[:,:2])))<1e-12,
        'base_odom_drift_below_existing_1mm_guard':np.max(np.linalg.norm(drift[:,:2],axis=1))<.001,
        'all_active_ARM_ONLY':all(r.get('adm_mode')=='ARM_ONLY' for r in active),
        'all_active_rotation_tracking_disabled':all(r.get('adm_rotation_tracking_enabled')=='False' for r in active)}
    for suffix,node in [('base_cmd','follower_arm_only'),('rover_twist','actuator_interlock'),
        ('mecanum_drive_controller/reference_unstamped','rover_twist_relay_ign'),
        ('arm_controller/joint_trajectory','actuator_interlock')]:
        writers=graph['/follower/'+suffix]
        checks['single_writer:'+suffix]=len(writers)==1 and writers[0]['node']=='/follower/'+node
    for name in ('follower_arm_only','wrench_preprocessor'):
        topics=[t[0] for t in graph['/follower/'+name+'/subscriptions']]
        checks['local_only:'+name]=not any(t.startswith(('/leader/','/world/','/experiment/')) or 'goal' in t for t in topics)
    events=[json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
    final=[e for e in events if e['robot']=='follower' and e['kind']=='base_final_reference']
    if result.get('dual_arm_only'):
        final=[e for e in final if e['sim_time']>=result['physical_base_approach']['sim_stopped_s']]
    checks['actual_final_reference_all_zero']=bool(final) and all(v==0. for e in final for v in e['value'])
    if result.get('stage') in ('AF-2','AF-3'):
        checks['grasp_held_after_stop']=result.get('grasp_held_after_stop') is True
        checks['gripper_hold_verified']=result.get('gripper_hold_verified') is True
    wrench=list(csv.DictReader((folder/'wrench_samples.csv').open()))
    force=[sum(float(r['wrench_'+str(i)])**2 for i in range(3))**.5 for r in wrench
           if r['kind']=='filtered' and (r.get('admittance_enabled')=='True' or r['phase']=='fault_injected')]
    controls=[json.loads(s)['value'] for s in (folder/'control_samples.jsonl').read_text().splitlines()]
    force += [np.linalg.norm(v['wrench_at_fault'][:3]) for v in controls if v.get('wrench_at_fault')]
    dt=[v['dt_control_s'] for v in controls if v.get('enabled') and 'dt_control_s' in v]
    if dt:checks['source_dt_inside_existing_guard']=min(dt)>0 and max(dt)<=.05
    summary={'folder':str(folder),'result_source':result_name,'passed':all(checks.values()),
        'checks':{k:bool(v) for k,v in checks.items()},
        'stage':result.get('stage'),'error':result.get('error'),'fault':result.get('fault'),
        'endpoint_stop_reasons':{r:v.get('reason') for r,v in result.get('final_endpoints',{}).items()},
        'leader_distance_m':result.get('leader_odom_distance_m'),
        'leader_peak_speed_m_s':result.get('leader_odom_peak_m_s'),
        'arm_actual_displacement_peak_m':float(np.max(np.linalg.norm(actual[:,:2]-home[:,:2],axis=1))),
        'track_error_peak_m':float(np.max(np.linalg.norm(array('adm_e_track_xy_m',2),axis=1))),
        'base_odom_drift_peak_m':float(np.max(np.linalg.norm(drift[:,:2],axis=1))),
        'base_odom_speed_peak_m_s':max(np.hypot(float(r['follower_odom_twist_0']),float(r['follower_odom_twist_1'])) for r in active),
        'force_increment_peak_N':max(force,default=None),
        'dt_control_range_s':[min(dt),max(dt)] if dt else None,
        'stop_latency_wall_s':result.get('stop_latency_wall_s'),
        'scope':('native friction grasp, half-stroke leader arm compensation' if result.get('native_arm_stroke')
                 else 'simulation micro fixed grasp')+'; experimental boundaries, not calibrated load ratings'}
    if result.get('native_arm_stroke'):
        leader=np.array([float(r['leader_p_actual_0']) for r in active])
        leader_gt=array('leader_gt_position',3)
        leader_world_delta=leader_gt[-1]-leader_gt[0]
        gt=np.array([[float(r['follower_gt_position_'+str(i)]) for i in range(3)] for r in active])
        summary.update(leader_arm_actual_retraction_m=float(leader[0]-leader[-1]),
            leader_world_base_displacement_xyz_m=leader_world_delta.tolist(),
            leader_world_base_distance_xy_m=float(np.linalg.norm(leader_world_delta[:2])),
            follower_world_base_drift_peak_m=float(np.max(np.linalg.norm(gt[:,:2]-gt[0,:2],axis=1))),
            object_world_displacement_xyz_m=result.get('payload_model_gt_displacement_xyz_m'),
            follower_base_fixture=result.get('follower_base_fixture',False),
            follower_joint_actual_change_rad=(array('follower_q_actual',5)[-1]-array('follower_q_actual',5)[0]).tolist(),
            minimum_joint_margin_rad=min(float(r['adm_min_joint_margin_rad']) for r in active),
            minimum_task_singular_value_m=min(float(r['adm_min_task_singular_value_m']) for r in active),
            maximum_height_error_m=float(np.max(abs(actual[:,2]-home[:,2]))),
            maximum_orientation_error_rad=float(np.max(np.linalg.norm(array('adm_orientation_error_rad',3),axis=1))))
        checks['body_world_drift_below_existing_1mm_guard']=summary['follower_world_base_drift_peak_m']<.001
        if result.get('dual_arm_only'):
            checks['leader_world_base_drift_below_1mm']=float(np.max(np.linalg.norm(leader_gt[:,:2]-leader_gt[0,:2],axis=1)))<.001
            summary['scope']='native friction grasp; both bases stopped; leader arm drives payload; no base restraint fixture'
        else:
            checks['leader_world_travel_in_existing_distance_band']=(
                .9*result['requested_distance_m']<=leader_world_delta[0]<=1.1*result['requested_distance_m'])
        if result.get('leader_base_distance_calibrated'):
            summary['leader_calibrated_distance_m']=result['leader_odom_distance_m']*result['leader_distance_metres_per_raw_odom_metre']
            summary['leader_distance_metres_per_raw_odom_metre']=result['leader_distance_metres_per_raw_odom_metre']
            checks['calibrated_leader_actual_world_x_reaches_requested_distance']=leader_world_delta[0]>=result['requested_distance_m']
        if result.get('follower_base_fixture'):
            checks['base_fixture_locked_after_grasp']=all(r.get('follower_base_fixture_state')=='locked' for r in active)
            anchors=[i for i,r in enumerate(rows) if r.get('phase')=='lock_follower_base_after_grasp' and r.get('follower_gt_position_0')]
            if not anchors:raise ValueError('missing post-grasp fixture anchor measurements')
            after=[r for r in rows[anchors[0]:] if r.get('follower_gt_position_0')]
            body=np.array([[float(r['follower_gt_position_'+str(i)]) for i in range(2)] for r in after])
            summary['follower_world_base_drift_from_grasp_peak_m']=float(np.max(np.linalg.norm(body-body[0],axis=1)))
            checks['body_world_drift_from_grasp_below_1mm']=summary['follower_world_base_drift_from_grasp_peak_m']<.001
        for robot in ('leader','follower'):
            for side in ('left','right'):
                forces=np.array([float(r[robot+'_native_'+side+'_force_N']) for r in active])
                checks[f'native_contact:{robot}/{side}']=all(r.get(robot+'_native_'+side+'_contact')=='True' for r in active)
                checks[f'native_force_band:{robot}/{side}']=bool(np.min(forces)>=8. and np.max(forces)<=26.)
                summary[f'{robot}_{side}_normal_force_range_N']=[float(np.min(forces)),float(np.max(forces))]
        summary['checks']={k:bool(v) for k,v in checks.items()};summary['passed']=all(checks.values())
    (folder/'p0_evidence.json').write_text(json.dumps(summary,indent=2))
    return summary,rows


def plot(folder,summary,rows):
    selected=[r for r in rows if r.get('adm_p_actual_m_0')]
    def val(key):return np.array([float(r.get(key) or 'nan') for r in selected])
    def norm(key,size):return np.linalg.norm(np.column_stack([val(key+'_'+str(i)) for i in range(size)]),axis=1)
    absolute=val('sim_time');t=absolute-absolute[0]
    fig,axes=plt.subplots(5,2,figsize=(15,17),sharex=True,constrained_layout=True)
    ax=axes[0,0];lead=val('leader_odom_pose_0');ax.plot(t,(lead-lead[0])*1000,label='leader raw wheel odom x')
    if summary.get('leader_distance_metres_per_raw_odom_metre'):
        ax.plot(t,(lead-lead[0])*summary['leader_distance_metres_per_raw_odom_metre']*1000,'--',label='leader calibrated own-odom estimate')
    ax.plot(t,-val('adm_e_arm_m_0')*1000,label='follower measured FK projected to world x')
    ax.plot(t,-val('adm_delta_p_adm_m_0')*1000,':',label='admittance projected to world x')
    if 'native friction' in summary['scope']:
        arm=val('leader_p_actual_0');ax.plot(t,(arm[0]-arm)*1000,'-.',label='leader measured own-arm retraction')
        body=val('leader_gt_position_0');ax.plot(t,(body-body[0])*1000,':',label='leader body GT (evaluation only)')
    if np.isfinite(val('payload_model_gt_position_0')).any():
        p=val('payload_model_gt_position_0');ax.plot(t,(p-p[0])*1000,'--',label='object GT (evaluation only)')
    ax.set_ylabel('Travel [mm]')
    ax=axes[0,1]
    for i in (0,1):ax.plot(t,val('wrench_filtered_'+str(i)),label=['Fx','Fy'][i]+' in base_footprint')
    forces=norm('wrench_filtered',3);ax.plot(t,forces,label='force norm')
    policy=json.loads((folder/'experiment_policy.json').read_text())
    limit=policy['controller_parameters']['force_increment_stop_N']
    if limit<max(1.,float(np.nanmax(forces))*2):ax.axhline(limit,color='r',linestyle=':',label='experimental guard, not rating')
    else:ax.text(.02,.95,f'Existing guard: {limit:g} N (not a load rating)',transform=ax.transAxes,va='top',fontsize=7)
    ax.set_ylabel('Incremental F/T [N]')
    ax=axes[1,0]
    for i in (0,1):ax.plot(t,val('adm_e_track_xy_m_'+str(i))*1e6,label=['e_track x','e_track y'][i])
    ax.set_ylabel('Tracking error [um]')
    ax=axes[1,1]
    for key,label in [('base_cmd','request'),('rover_twist','interlock'),('base_final_reference','final controller input')]:
        for i in range(3):ax.plot(t,val('follower_'+key+'_'+str(i)),label=label+' '+['vx [m/s]','vy [m/s]','wz [rad/s]'][i],alpha=.7)
    ax.set_ylabel('Follower base commands')
    ax=axes[2,0]
    for i in (0,1):ax.plot(t,val('adm_base_drift_xy_yaw_'+str(i))*1e6,label='odom '+['x','y'][i])
    if any(r.get('follower_gt_drift_since_grasp_m') for r in selected):
        ax.plot(t,val('follower_gt_drift_since_grasp_m')*1e6,label='body world GT drift (evaluation only)')
    ax.set_ylabel('Measured base drift [um]')
    ax=axes[2,1]
    for i in range(5):
        q=val('follower_q_actual_'+str(i));line=ax.plot(t,(q-q[0])*1000,label=f'Joint_{i+1} actual')[0]
        ax.plot(t,(val('follower_q_cmd_'+str(i))-q[0])*1000,'--',color=line.get_color(),alpha=.7)
    ax.set_ylabel('Joint change [mrad]; dashed = command')
    ax=axes[3,0];ax.plot(t,val('adm_min_joint_margin_rad'),label='min measured joint margin');ax.axhline(.01,color='r',linestyle=':');ax.set_ylabel('Joint margin [rad]')
    ax=axes[3,1];ax.plot(t,val('adm_min_task_singular_value_m'),label='sigma_min(J_xy)');ax.axhline(.001,color='r',linestyle=':');ax.set_ylabel('Task singular value [m]')
    ax=axes[4,0]
    for i in range(5):ax.plot(t,val('follower_reaction_torque_Nm_'+str(i)),label=f'Joint_{i+1} load')
    ax.set_ylabel('Transmitted joint load [Nm]')
    ax=axes[4,1]
    if 'native friction' in summary['scope']:
        z=val('leader_p_actual_2');ax.plot(t,(z-z[0])*1000,label='leader measured height change')
        ax.plot(t,val('adm_e_arm_m_2')*1000,label='follower measured height change')
        ax.set_ylabel('Measured FK height change [mm]')
    else:
        for robot in ('leader','follower'):ax.plot(t,val(robot+'_constraint_error_m')*1000,label=robot+' constraint error')
        ax.plot(t,val('adm_e_arm_m_2')*1000,label='follower height deviation');ax.set_ylabel('Grasp/height error [mm]')
    stops=[i for i,r in enumerate(selected) if r.get('follower_state')=='SAFE_STOP']
    for ax in axes.flat:
        if stops:ax.axvline(t[stops[0]],color='k',linestyle=':',alpha=.6,label='safe stop')
        ax.grid(alpha=.25);ax.legend(fontsize=6,loc='best')
    for ax in axes[-1]:ax.set_xlabel('Simulation time from first arm sample [s]')
    grasp='native friction grasp' if 'native friction' in summary['scope'] else 'fixed grasp'
    fig.suptitle(f'{folder.name}: ARM_ONLY {summary["stage"]}, passed={summary["passed"]}; {grasp} simulation')
    for suffix in ('png','pdf'):fig.savefig(folder/('p0_measured.'+suffix),dpi=140)
    plt.close(fig)
    if 'native friction' in summary['scope']:
        fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,constrained_layout=True)
        for robot,ax in zip(('leader','follower'),axes):
            for side in ('left','right'):
                ax.plot(t,val(robot+'_native_'+side+'_force_N'),label=side+' finger measured normal force')
                lost=val(robot+'_native_'+side+'_force_N').copy()
                lost[:]=[float(r.get(robot+'_native_'+side+'_contact')!='True') for r in selected]
                missing=lost>0
                if missing.any():ax.scatter(t[missing],np.zeros(np.count_nonzero(missing)),marker='x',color='r',label=side+' contact absent')
            ax.axhline(8.,color='r',linestyle=':',label='inherited simulation monitor 8 / 26 N')
            ax.axhline(26.,color='r',linestyle=':');ax.set_ylabel(robot+' grip [N]');ax.legend(fontsize=8);ax.grid(alpha=.25)
        axes[-1].set_xlabel('Simulation time from first arm sample [s]')
        fig.suptitle('Native friction contact: measured finger forces and contact monitoring')
        for suffix in ('png','pdf'):fig.savefig(folder/('native_grip.'+suffix),dpi=140)
        plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--held',action='store_true');a=parser.parse_args()
    folder=a.case.resolve();summary,rows=evaluate(folder,'held_result.json' if a.held else 'result.json')
    plot(folder,summary,rows)
    print(json.dumps(summary,indent=2));return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
