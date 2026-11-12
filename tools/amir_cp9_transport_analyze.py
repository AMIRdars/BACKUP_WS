#!/usr/bin/env python3
"""Generate shareable charts and CSV from saved native carry evidence only."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def save_csv(path, rows):
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w') as out:
        writer=csv.DictWriter(out,fieldnames=keys);writer.writeheader();writer.writerows(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--trial',required=True);a=p.parse_args();root=a.audit_directory
    selected={v['trial'] for v in json.loads((root/'selected_trials.json').read_text())['trials']} if (root/'selected_trials.json').exists() else set()
    inventory=[];normal=[];faults=[]
    for f in sorted(root.glob('*/result.json')):
        r=json.loads(f.read_text());item={'trial':f.parent.name,'passed':r.get('passed'),'final_selected':f.parent.name in selected,
            'phase':r.get('error_phase'),'error':r.get('error'),'fault':r.get('fault_test'),'distance_m':r.get('distance_m')}
        inventory.append(item)
        if r.get('cooperative_contact_transport_verified'):
            normal.append({'trial':f.parent.name,'final_selected':f.parent.name in selected,'distance_m':r['distance_m'],'speed_m_s':r['speed_m_s'],
                **{k+'_dx_m':v[0] for k,v in r['gt_delta_xy_yaw'].items()},
                'final_arm_offset_xy_m':r['final_arm_offset_xy_m'],'final_follower_speed_m_s':r['final_follower_speed_m_s']})
        if r.get('fault_injected'):
            faults.append({'trial':f.parent.name,'final_selected':f.parent.name in selected,'fault':r['fault_test'],'passed':r['passed'],
                'dual_zero_latency_wall_s':r.get('dual_zero_latency_wall_s'),
                'both_arm_base_inactive':r.get('both_arm_base_inactive'),'no_automatic_restart':r.get('no_automatic_restart'),
                **{k+'_encoder_vx_m_s':v for k,v in r.get('measured_encoder_vx_at_injection',{}).items()},
                **{k+'_gt_travel_pre_injection_m':v for k,v in r.get('gt_travel_before_injection_m',{}).items()}})
    (root/'experiment_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    save_csv(root/'normal_metrics.csv',normal);save_csv(root/'moving_fault_metrics.csv',faults)
    folder=root/a.trial;report=json.loads((folder/'result.json').read_text())
    rows=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
    move=[v for v in rows if v['phase']=='motion'];start=move[0]['sim_time']
    baseline=[v for v in rows if v['phase']=='baseline'][-1]
    rows=[v for v in rows if v['phase'] in ('baseline','motion','settle')]
    t=[v['sim_time']-start for v in rows]
    fig,axes=plt.subplots(4,1,figsize=(11,11),sharex=True,constrained_layout=True)
    for r in ('leader','follower'):
        axes[0].plot(t,[(v[r+'_gt_x']-baseline[r+'_gt_x'])*1000 for v in rows],label=r+' base (GT)')
    axes[0].plot(t,[(v['leader_encoder_x']-baseline['leader_encoder_x'])*1000 for v in rows],
        linestyle='--',alpha=.65,label='leader encoder')
    axes[0].plot(t,[(v['payload_x']-baseline['payload_x'])*1000 for v in rows],label='native payload (GT)')
    axes[0].axhline(report['distance_m']*1000,color='black',linestyle=':',label='leader-only goal')
    axes[0].set_ylabel('world displacement [mm]');axes[0].legend(loc='best',ncol=2)
    axes[0].set_title('CP-9 native contact: guarded low-speed linear carry (simulation only)')
    for key,label in [('measured_error_x','measured local arm X'),('low_error_x','0.2 Hz arm bias')]:
        axes[1].plot(t,[v[key]*1e6 for v in rows],label=label)
    axes[1].set_ylabel('local deflection [um]');axes[1].legend(loc='best')
    axes[2].plot(t,[v['follower_output_vx']*1000 for v in rows],label='follower final command, body X')
    axes[2].plot(t,[v['follower_encoder_vx']*1000 for v in rows],label='follower encoder, body X',alpha=.65)
    axes[2].set_ylabel('base velocity [mm/s]');axes[2].legend(loc='best')
    for r in ('leader','follower'):
        for s in ('left','right'):
            axes[3].plot(t,[v['native_grasp_monitor'].get(r,{}).get('own_forces_N',{}).get(s,math.nan) for v in rows],label=r+' '+s)
    for y in (8.,26.):axes[3].axhline(y,color='gray',linestyle=':')
    axes[3].set_ylabel('finger normal reaction [N]');axes[3].legend(loc='best',ncol=2)
    axes[3].set_xlabel('simulation time from motion start [s]')
    end=move[-1]['sim_time']-start
    for ax in axes:
        ax.axvline(0.,color='gray',linewidth=.7);ax.axvline(end,color='gray',linewidth=.7)
        ax.grid(alpha=.25)
    fig.savefig(root/'native_linear_carry.png',dpi=160)
    print(json.dumps({'experiments':len(inventory),'normal':len(normal),'faults':len(faults),
        'figure':str(root/'native_linear_carry.png')},indent=2))


if __name__=='__main__':main()
