#!/usr/bin/env python3
"""Offline CP-6 evidence and plots; never publishes control inputs."""
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
from amir_compliance_control.admittance import rotation_error
from amir_compliance_control.recentering import BaseRecentering


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-directory', type=Path, required=True)
    parser.add_argument('--trials', nargs='+', required=True)
    args = parser.parse_args()
    root = args.audit_directory
    model = LocalModel((root/'generated/follower.urdf').read_text())
    summary, signs = [], []
    for name in args.trials:
        path = root/name
        result = json.loads((path/'result.json').read_text())
        if not result['passed']:
            raise ValueError('selected trial failed: '+name)
        rows = [json.loads(line) for line in (path/'samples.jsonl').open()]
        graph = json.loads((path/'graph.json').read_text())
        nodes = ['/follower/follower_arm_base', '/follower/wrench_preprocessor', '/follower/safety_gate']
        for node in nodes:
            topics = [t for t, _ in graph[node]['subscriptions']]
            if any(t.startswith(('/leader/', '/world/')) or 'transport_goal' in t
                   or t.endswith(('/tf','/tf_static','/odom')) for t in topics):
                raise ValueError('forbidden follower input: '+str(topics))
        active = [r for r in rows if r['phase'] in ('baseline','motion','settle') and r['admittance_enabled']]
        metrics = {'trial': name, 'passed': True, 'rows': len(rows),
                   'follower_boundary_passed': True, 'fault': result['fault_test']}
        for key in ('dual_zero_latency_wall_s', 'final_arm_offset_x_m', 'final_follower_speed_m_s'):
            metrics[key] = result.get(key)
        if result['fault_test'] != 'none':
            if not result['no_automatic_restart'] or any(abs(result['twists_at_injection'][r][0]) <= 5e-6
                    for r in ('leader','follower')):
                raise ValueError('fault was not injected while both robots were moving: '+name)
            late = [r for r in rows if r['phase']=='fault_injected' and r['wall_time']>=rows[-1]['wall_time']-.2]
            metrics['post_fault_max_encoder_translation_m_s'] = max(abs(r[robot+'_encoder_v'+axis])
                for r in late for robot in ('leader','follower') for axis in 'xy')
            metrics['post_fault_max_encoder_wz_rad_s'] = max(abs(r[robot+'_encoder_wz'])
                for r in late for robot in ('leader','follower'))
            if (metrics['post_fault_max_encoder_translation_m_s']>1e-6
                    or metrics['post_fault_max_encoder_wz_rad_s']>1e-5):
                raise ValueError('post-fault wheel odometry did not settle: '+name)
        for robot, distance in result.get('gt_distance_m', {}).items():
            metrics[robot+'_distance_m'] = distance
        if result['condition'] == 'transport':
            if any(r['applied_fx_N'] != 0 or r['applied_fy_N'] != 0 for r in rows):
                raise ValueError('transport included external excitation: '+name)
            if result['endpoint_states']['follower']['accepted_goals'] != 0:
                raise ValueError('follower received goal: '+name)
            metrics['zero_external_excitation'] = True
            if active:
                q = lambda r: [r[f'follower_q{i}_rad'] for i in range(1,6)]
                orientation = model.frames(q(active[0]))[1][:3,:3]
                metrics['max_local_orientation_error_rad'] = float(max(
                    np.linalg.norm(rotation_error(model.frames(q(r))[1][:3,:3], orientation)) for r in active))
                metrics['max_measured_arm_offset_x_m'] = max(abs(r['measured_error_x']) for r in active)
                metrics['max_filtered_fx_N'] = max(abs(r['filtered_fx_N']) for r in active)
                metrics['max_payload_height_change_m'] = max(abs(r['payload_z']-active[0]['payload_z']) for r in active)
                quaternion = lambda r: np.array([r['payload_q'+k] for k in 'xyzw'])
                origin = quaternion(active[0]); origin /= np.linalg.norm(origin)
                metrics['max_payload_rotation_change_rad'] = float(max(2*np.arccos(np.clip(
                    abs(quaternion(r)@origin)/np.linalg.norm(quaternion(r)),0.,1.)) for r in active))
                metrics['saturated_samples'] = sum(r['saturated'] for r in active)
        for test in result.get('sign_tests', []):
            phase = test['phase']
            selected = [r for r in rows if r['phase']==phase and 'follower_gt_x' in r]
            first,last = selected[0],selected[-1]
            yaw = first['follower_gt_yaw']
            dx,dy = (last['follower_gt_'+k]-first['follower_gt_'+k] for k in 'xy')
            axis = 'x' if phase.startswith('fx') else 'y' if phase.startswith('fy') else 'yaw'
            gt = (math.cos(yaw)*dx+math.sin(yaw)*dy if axis=='x' else
                  -math.sin(yaw)*dx+math.cos(yaw)*dy if axis=='y' else
                  math.atan2(math.sin(last['follower_gt_yaw']-yaw),math.cos(last['follower_gt_yaw']-yaw)))
            direction = -1 if '-1.0' in phase else 1
            if direction*gt<=1e-5:
                raise ValueError('ground truth sign failed: '+name+' '+phase)
            early = min(selected, key=lambda r: abs(r['sim_time']-first['sim_time']-.1))
            item = dict(test, trial=name, ground_truth_delta=gt, ground_truth_passed=True,
                        initial_interval_s=early['sim_time']-first['sim_time'],
                        initial_arm_error=early['measured_error_'+axis]-first['measured_error_'+axis],
                        initial_encoder_delta=early['follower_encoder_'+axis]-first['follower_encoder_'+axis])
            signs.append(item)
        summary.append(metrics)
        if name.startswith('transport_1mm'):
            data = active
            t = np.array([r['sim_time']-data[0]['sim_time'] for r in data])
            fig, axes = plt.subplots(4,1,figsize=(10,10),sharex=True,layout='constrained')
            for robot in ('leader','follower'):
                axes[0].plot(t, [(r[robot+'_gt_x']-data[0][robot+'_gt_x'])*1e3 for r in data],label=robot)
            axes[0].plot(t,[(r['payload_x']-data[0]['payload_x'])*1e3 for r in data],label='payload')
            axes[0].set_ylabel('World travel [mm]')
            for key,label in [('measured_error_x','measured arm'),('low_error_x','0.2 Hz lowpass')]:
                axes[1].plot(t,[r[key]*1e6 for r in data],label=label)
            axes[1].set_ylabel('Local error [um]')
            axes[2].plot(t,[r['leader_output_vx']*1e6 for r in data],label='leader')
            axes[2].plot(t,[-r['follower_output_vx']*1e6 for r in data],label='follower (world forward)')
            axes[2].set_ylabel('Base command [um/s]')
            axes[3].plot(t,[r['filtered_fx_N'] for r in data],label='follower local Fx')
            axes[3].set_ylabel('Tared force [N]');axes[3].set_xlabel('Simulation time since baseline [s]')
            for ax in axes: ax.grid(True);ax.legend(loc='best')
            fig.suptitle(name+' | leader-only 1 mm goal, zero external excitation')
            fig.savefig(root/(name+'.png'),dpi=160);plt.close(fig)
    bands=[]
    grasp = model.frames([0.,1.4,-1.6,.2,1.570796])[0]
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    for freq in (.05,2.):
        control=BaseRecentering(.2,[.5,.5,1.],[.0005,.0005,.001],[.001,.001,.002])
        time=np.arange(.01,100.01,.01);signal=1e-4*np.sin(2*np.pi*freq*time);filtered=[]
        for value in signal:
            control.step([value,0,0],grasp,.01);filtered.append(control.low[0])
        tail=time>60
        gain=float(np.sqrt(np.mean(np.asarray(filtered)[tail]**2)/np.mean(signal[tail]**2)))
        bands.append({'input_frequency_hz':freq,'measured_lowpass_gain':gain})
        ax.plot(time[-2000:]-80,np.array(filtered)[-2000:]*1e6,label=f'{freq:g} Hz, gain={gain:.3f}')
    ax.set(xlabel='Time [s]',ylabel='Base-path filtered error [um]',title='Software lowpass response (synthetic input)')
    ax.grid(True);ax.legend();fig.savefig(root/'band_separation.png',dpi=160);plt.close(fig)
    for filename,items in [('comparison',summary),('signs',signs),('bands',bands)]:
        (root/(filename+'.json')).write_text(json.dumps(items,indent=2)+'\n')
        keys=sorted({k for item in items for k in item})
        with (root/(filename+'.csv')).open('w') as file:
            writer=csv.DictWriter(file,fieldnames=keys);writer.writeheader();writer.writerows(items)
    print(json.dumps({'trials':len(summary),'signed_axes':len(signs),'bands':bands},indent=2))


if __name__=='__main__':
    main()
