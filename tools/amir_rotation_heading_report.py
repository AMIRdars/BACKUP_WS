#!/usr/bin/env python3
"""Offline evaluation of local yaw calibration. GT never enters the follower."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from amir_compliance_control.rotation_observer import ProjectedYaw,wrap_pi


def analyze(folder):
    rows=[json.loads(line) for line in (folder/'passive_local.jsonl').read_text().splitlines()]
    if not rows:return dict(local_samples=0)
    stationary=[r for r in rows if r['phase']=='static']
    if len(stationary)<2:return dict(local_samples=len(rows),static_samples=len(stationary))
    geometry=ProjectedYaw((folder/'generated/follower.urdf').read_text(),orientation_envelope_rad=.01,
                          joint_margin_rad=.01,heading_frame='grasp_heading')
    psi0=float(np.mean(np.unwrap([r['psi_ee_B'] for r in stationary])))
    noise=float(np.std(np.unwrap([r['psi_ee_B'] for r in stationary]),ddof=1))
    w0=np.mean([r['wrench_tcp'] for r in stationary],axis=0)
    summary=dict(local_samples=len(rows),static_samples=len(stationary),psi_static_mean_rad=psi0,
        yaw_static_std_rad=noise,W0=w0.tolist(),phases={},
        minimum_joint_margin_rad=min(geometry.measure(r['q_rad'])['joint_margin_rad'] for r in rows),
        maximum_follower_base_twist=float(max(np.max(abs(np.array(r['base_twist']))) for r in rows)),
        all_recorded_grasps_attached=all(r['grasp_attached'] for r in rows),
        maximum_incremental_force_N=0.,maximum_incremental_moment_Nm=0.)
    for name in ('static','positive','negative'):
        group=[r for r in rows if r['phase']==name]
        if not group:continue
        tail=group[-max(4,len(group)//4):]
        signal=float(np.median([wrap_pi(r['psi_ee_B']-psi0) for r in tail]))
        summary['phases'][name]=dict(samples=len(group),duration_s=group[-1]['timestamp']-group[0]['timestamp'],
            local_signal_rad=signal,signal_to_static_noise=None if noise==0. else signal/noise)
    with (folder/'calibration_measurements.csv').open('w',newline='') as f:
        keys=['t_s','phase','psi_local_rad','e_psi_static_rad','force_delta_norm_N','moment_delta_norm_Nm',
              'tau_z_delta_Nm','base_speed_norm','grasp_attached','yaw_valid',
              'p_B_x_m','p_B_y_m','p_B_z_m','projection_norm','joint_margin_rad']
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
        for row in rows:
            dw=np.array(row['wrench_tcp'])-w0;force=float(np.linalg.norm(dw[:3]));moment=float(np.linalg.norm(dw[3:]))
            measured=geometry.measure(row['q_rad'])
            summary['maximum_incremental_force_N']=max(summary['maximum_incremental_force_N'],force)
            summary['maximum_incremental_moment_Nm']=max(summary['maximum_incremental_moment_Nm'],moment)
            writer.writerow(dict(t_s=row['timestamp'],phase=row['phase'],psi_local_rad=row['psi_ee_B'],
                e_psi_static_rad=wrap_pi(row['psi_ee_B']-psi0),force_delta_norm_N=force,
                moment_delta_norm_Nm=moment,tau_z_delta_Nm=dw[5],
                base_speed_norm=float(np.linalg.norm(row['base_twist'])),grasp_attached=row['grasp_attached'],
                yaw_valid=row['yaw_valid'],p_B_x_m=measured['p_ee_B'][0],p_B_y_m=measured['p_ee_B'][1],
                p_B_z_m=measured['p_ee_B'][2],projection_norm=measured['projection_norm'],
                joint_margin_rad=measured['joint_margin_rad']))
    gt=[json.loads(line) for line in (folder/'evaluation_only.jsonl').read_text().splitlines()]
    static=[r for r in gt if r['phase']=='static']
    if static:
        initial=np.array(static[-1]['object_pose_GT']);center=initial[:3,3]
        summary['maximum_center_drift_m']=max(float(np.linalg.norm(np.array(r['object_pose_GT'])[:3,3]-center)) for r in gt)
        for name in ('positive','negative'):
            group=[r for r in gt if r['phase']==name]
            if not group:continue
            tail=group[-max(4,len(group)//4):]
            angles=[]
            for row in tail:
                delta=np.array(row['object_pose_GT'])[:3,:3]@initial[:3,:3].T
                angles.append(math.atan2(delta[1,0],delta[0,0]))
            summary['phases'][name]['object_yaw_GT_rad_offline']=float(np.median(angles))
        # Fixed-grasp compliance error is evaluated against the loaded static
        # object-to-TCP transform; the local observer consumes no GT.
        if all('follower_base_pose_GT' in r for r in gt):
            g=ProjectedYaw((folder/'generated/follower.urdf').read_text(),orientation_envelope_rad=.01,
                           joint_margin_rad=.01,heading_frame='grasp_heading')
            stamps=np.array([r['timestamp'] for r in rows]);transforms=[]
            for row in gt:
                i=int(np.argmin(abs(stamps-row['timestamp'])))
                if abs(stamps[i]-row['timestamp'])>.02:continue
                position,sensor=g.model.frames(rows[i]['q_rad']);tcp=sensor@g.tcp_offset;tcp[:3,3]=position
                relative=np.linalg.inv(np.array(row['object_pose_GT']))@np.array(row['follower_base_pose_GT'])@tcp
                transforms.append((row['phase'],relative))
            home=next((T for phase,T in reversed(transforms) if phase=='static'),None)
            if home is not None:
                errors=[np.linalg.inv(home)@T for phase,T in transforms]
                summary['maximum_follower_constraint_translation_change_m']=max(float(np.linalg.norm(T[:3,3])) for T in errors)
                summary['maximum_follower_constraint_rotation_change_rad']=max(math.acos(float(np.clip((np.trace(T[:3,:3])-1)/2,-1,1))) for T in errors)
    (folder/'offline_analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    a=p.parse_args();print(json.dumps(analyze(a.folder.resolve()),indent=2))


if __name__=='__main__':main()
