#!/usr/bin/env python3
"""Measured speed/grasp comparison for saved dual-ARM_ONLY trials (offline)."""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def measurements(case):
    result_path = case / ('held_result.json' if (case/'held_result.json').exists() else 'result.json')
    result = json.loads(result_path.read_text())
    sources = {robot: {} for robot in ('leader', 'follower')}
    force_samples = []
    bases = {robot: [] for robot in sources}
    contacts = {robot: {side: True for side in ('left', 'right')} for robot in sources}
    force_ranges = {robot: {side: [float('inf'), -float('inf')] for side in ('left', 'right')} for robot in sources}
    base_zero = True
    active_count = 0
    with (case/'timeseries.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row.get('admittance_enabled') != 'True':
                continue
            active_count += 1
            stamp = float(row['sim_time'])
            forces = []
            for robot in sources:
                bases[robot].append([stamp, *[float(row[robot+'_gt_position_'+str(i)]) for i in range(2)]])
                for i in range(3):
                    base_zero &= float(row.get(robot+'_base_final_reference_'+str(i)) or 0.) == 0.
                for side in ('left', 'right'):
                    force = float(row[robot+'_native_'+side+'_force_N'])
                    contacts[robot][side] &= row.get(robot+'_native_'+side+'_contact') == 'True'
                    limits = force_ranges[robot][side]
                    limits[0], limits[1] = min(limits[0], force), max(limits[1], force)
                    forces.append(force)
                if row['phase'] == 'leader_translation' and row.get(robot+'_joint_stamp_s'):
                    source = float(row[robot+'_joint_stamp_s'])
                    sources[robot][source] = [*[float(row[robot+'_p_actual_'+str(i)]) for i in range(3)],
                        *[float(row[robot+'_qdot_actual_'+str(i)]) for i in range(5)]]
            wrench = np.array([float(row.get('wrench_filtered_'+str(i)) or 0.) for i in range(3)])
            track = np.array([float(row.get('adm_e_track_xy_m_'+str(i)) or 0.) for i in range(2)])
            force_samples.append([stamp, np.linalg.norm(wrench), min(forces), np.linalg.norm(track)])
    info = {'case': str(case), 'motion_completed': bool(result.get('passed') and result.get('leader_completed')),
        'error': result.get('error'), 'safe_stop_verified': result.get('safe_stop_verified', False),
        'grasp_held_after_stop': result.get('grasp_held_after_stop', False),
        'endpoint_stop_reasons': {r: v.get('reason') for r,v in result.get('final_endpoints',{}).items()},
        'active_rows': active_count, 'active_final_base_commands_zero': bool(active_count and base_zero),
        'contacts_continuous': bool(active_count and all(all(v.values()) for v in contacts.values())),
        'contacts': contacts, 'normal_force_ranges_N': force_ranges if active_count else {}, 'arms': {}}
    evidence_path = case/'dual_arm_evidence.json'
    if evidence_path.exists():
        evidence = json.loads(evidence_path.read_text())
        info.update(strict_acceptance_passed=evidence['passed'],
            strict_pass_count=sum(evidence['checks'].values()), strict_check_count=len(evidence['checks']),
            failed_checks=[k for k,v in evidence['checks'].items() if not v])
        info['non_force_band_checks_passed']=all(v for k,v in evidence['checks'].items()
            if not k.startswith('native_force_band:'))
    series = {}
    for robot, sign in (('leader',1), ('follower',-1)):
        samples = sources[robot]
        if len(samples) < 2:
            continue
        t = np.array(sorted(samples))
        v = np.array([samples[k] for k in t])
        speed = sign*np.diff(v[:,0])/np.diff(t)
        body = np.array(bases[robot])
        drift = np.linalg.norm(body[:,1:]-body[0,1:], axis=1)
        xml=ET.parse(case/'generated'/(robot+'.urdf'))
        limits=np.array([float(xml.find(f"joint[@name='Joint_{i}']/limit").get('velocity')) for i in range(1,6)])
        info['arms'][robot] = {'source_samples': len(t), 'source_duration_s': float(t[-1]-t[0]),
            'TCP_displacement_mm': (1000*(v[-1,:3]-v[0,:3])).tolist(),
            'FK_interval_speed_peak_mm_s': float(1000*np.max(speed)),
            'FK_interval_speed_p99_5_mm_s': float(1000*np.quantile(speed,.995)),
            'actual_joint_speed_peak_rad_s': float(np.max(abs(v[:,3:]))),
            'actual_URDF_joint_velocity_ratio_peak':float(np.max(abs(v[:,3:])/limits)),
            'active_base_GT_drift_peak_mm': float(1000*np.max(drift))}
        series[robot] = {'t':t-t[0], 'speed_mm_s':1000*speed, 'x_mm':1000*sign*(v[:,0]-v[0,0]),
                         'body_t':body[:,0]-body[0,0], 'body_drift_mm':drift*1000}
    if force_samples:
        force_samples = np.array(force_samples)
        info['filtered_increment_force_peak_N'] = float(np.max(force_samples[:,1]))
        info['tracking_error_peak_mm'] = float(1000*np.max(force_samples[:,3]))
        series['force'] = force_samples
    info['transport_and_grasp_passed'] = bool(info['motion_completed'] and info['safe_stop_verified']
        and info['grasp_held_after_stop'] and info['contacts_continuous'] and info['active_final_base_commands_zero']
        and info.get('non_force_band_checks_passed',False)
        and all(v['actual_URDF_joint_velocity_ratio_peak']<=1. for v in info['arms'].values()))
    (case/'speed_metrics.json').write_text(json.dumps(info,indent=2,allow_nan=False))
    return info,series


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();root=args.root.resolve()
    results=[]
    fig,axes=plt.subplots(6,1,figsize=(13,19),constrained_layout=True)
    csv_rows=[]
    cases={p.parent for pattern in ('*/trial/result.json','*/trial/held_result.json') for p in root.glob(pattern)}
    for case in sorted(cases):
        info,series=measurements(case);results.append(info)
        label=case.parent.name + (' [completed]' if info['motion_completed'] else ' [stopped]')
        for j,robot in enumerate(('leader','follower')):
            if robot not in series:
                continue
            s=series[robot]
            axes[j].plot(s['t'][1:],s['speed_mm_s'],label=label)
            axes[2].plot(s['t'],s['x_mm'],label=label+' '+robot)
            axes[5].plot(s['body_t'],s['body_drift_mm'],label=label+' '+robot)
            for i in range(1,len(s['t'])):
                csv_rows.append([case.parent.name,robot,s['t'][i],s['speed_mm_s'][i-1],s['x_mm'][i]])
        if 'force' in series:
            f=series['force'];t=f[:,0]-f[0,0]
            axes[3].plot(t,f[:,1],label=label)
            axes[4].plot(t,f[:,2],label=label)
    labels=['Leader FK speed [mm/s]','Follower FK speed [mm/s]','Arm TCP displacement [mm]',
        'Incremental filtered force [N]','Minimum of four finger forces [N]','Base GT drift from active [mm]']
    axes[4].axhline(8.,color='red',ls='--',label='inherited 8 N lower band')
    axes[5].axhline(1.,color='red',ls='--',label='existing 1 mm guard')
    for ax,label in zip(axes,labels):
        ax.set_ylabel(label);ax.set_xlabel('Own source time since motion start / observer time since active [s]')
        ax.grid(alpha=.25);ax.legend(fontsize=6,ncol=2)
    fig.suptitle('Unchanged URDF, both bases stopped: measured speed and native friction grip')
    for ext in ('png','pdf'):fig.savefig(root/('speed_comparison.'+ext),dpi=140)
    plt.close(fig)
    (root/'comparison.json').write_text(json.dumps(results,indent=2,allow_nan=False))
    with (root/'measured_speed.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['case','robot','joint_source_elapsed_s','FK_interval_speed_mm_s','TCP_displacement_mm']);writer.writerows(csv_rows)
    print(json.dumps([{'case':Path(r['case']).parent.name,'completed':r['motion_completed'],
        'transport_grasp_passed':r['transport_and_grasp_passed'], 'strict':r.get('strict_acceptance_passed'),
        'leader_peak_mm_s':r['arms'].get('leader',{}).get('FK_interval_speed_peak_mm_s'),
        'error':r['error']} for r in results],indent=2))


if __name__=='__main__':main()
