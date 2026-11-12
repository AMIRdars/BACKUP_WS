#!/usr/bin/env python3
"""Offline evidence index and figures for native contact commissioning."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--trial',required=True);a=p.parse_args();index=[]
    for f in sorted(a.audit_directory.glob('*/result.json')):
        r=json.loads(f.read_text());index.append({'trial':f.parent.name,'kind':r.get('kind'),'passed':r['passed'],
            'fault':r.get('fault','none'),'fault_injected':r.get('fault_injected',False),
            'unsupported_static_hold_verified':r.get('unsupported_static_hold_verified',False),
            'base_arm_inactive_after_fault':r.get('base_arm_inactive_after_fault'),
            'both_fault_latency_wall_s':r.get('both_local_grippers_fault_latency_wall_s'),
            'no_automatic_gripper_restart':r.get('no_automatic_gripper_restart'),
            'error_phase':r.get('error_phase'),'error':r.get('error')})
    (a.audit_directory/'experiment_inventory.json').write_text(json.dumps(index,indent=2)+'\n')
    with (a.audit_directory/'experiments.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(index[0]));w.writeheader();w.writerows(index)
    faults=[r for r in index if r['fault_injected']]
    with (a.audit_directory/'faults.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(index[0]));w.writeheader();w.writerows(faults)
    folder=a.audit_directory/a.trial;rows=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
    first=rows[0]['sim'];t=np.array([r['sim']-first for r in rows]);fig,ax=plt.subplots(4,1,figsize=(12,11),sharex=True)
    positions=np.array([r['payload_xyz'] for r in rows]);ax[0].plot(t,(positions-positions[0])*1000);ax[0].legend(['x','y','z']);ax[0].set_ylabel('payload change (mm)')
    for robot in ('leader','follower'):
        for side in ('left','right'):
            ax[1].plot(t,[r.get('grasp',{}).get(robot,{}).get('own_sensors',{}).get(side+'_force',np.nan) for r in rows],label=robot+'/'+side)
        ax[2].plot(t,[r.get('grasp',{}).get(robot,{}).get('q',np.nan) for r in rows],label=robot)
        errors=[]
        for row in rows:
            q=row.get(robot+'_q',{});master=q.get('Gripper',np.nan)
            differences=[abs(v-master) for n,v in q.items() if n.endswith('_mimic')]
            errors.append(max(differences,default=np.nan))
        ax[3].plot(t,errors,label=robot)
    ax[1].axhline(26,color='r',ls='--',label='simulation stop policy (not hardware rating)');ax[1].set_ylabel('finger raw normal axis (N)');ax[1].legend(ncol=3)
    ax[2].set_ylabel('measured Gripper (rad)');ax[2].legend()
    ax[3].axhline(.01,color='r',ls='--',label='simulation tracking policy');ax[3].set_ylabel('max mimic error (rad)');ax[3].set_xlabel('simulation time from first sample (s)');ax[3].legend()
    events=[('close start',next((r['sim']-first for r in rows if any(v.get('state')=='CLOSING' for v in r.get('grasp',{}).values())),None)),
        ('support removed',next((r['sim']-first for r in rows if r['phase']=='unsupported_closed_gripper'),None))]
    for label,stamp in events:
        if stamp is None:continue
        for axis in ax:axis.axvline(stamp,color='purple',ls=':',alpha=.6)
        ax[0].text(stamp,.96,label,transform=ax[0].get_xaxis_transform(),ha='right',va='top',fontsize=9,rotation=90)
    for axis in ax:axis.grid(alpha=.3)
    fig.suptitle(a.trial+' — native contact, no weld or payload force assistance');fig.tight_layout()
    fig.savefig(a.audit_directory/'contact_grasp.png',dpi=160);plt.close(fig)
    print(json.dumps(index,indent=2))


if __name__=='__main__':main()
