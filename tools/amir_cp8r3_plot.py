#!/usr/bin/env python3
"""Saved-evidence figures. No ROS imports, actuator client or simulator input."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--trials',nargs='+',required=True)
    p.add_argument('--output-stem',required=True)
    a=p.parse_args()
    fig,axes=plt.subplots(3,2,figsize=(13,10),constrained_layout=True)
    native=False
    for name in a.trials:
        folder=a.audit_directory/name
        result=json.loads((folder/'result.json').read_text())
        native=result['condition'].startswith('native_')
        rows=[json.loads(line) for line in (folder/'samples.jsonl').open()]
        initial=next(r for r in reversed(rows) if r['phase']=='baseline')
        moving=[r for r in rows if r['phase'] in ('rotation','hold')]
        t=np.array([r['t_s']-moving[0]['t_s'] for r in moving])
        angle=np.unwrap([r['object_yaw_GT_rad'] for r in moving])-initial['object_yaw_GT_rad']
        line,=axes[0,0].plot(t,np.degrees(angle),label=name)
        axes[0,0].plot(t,np.degrees([r.get('phi_cmd_rad') or 0. for r in moving]),'--',color=line.get_color(),alpha=.6)
        axes[0,1].plot(t,[r.get('progress_rate',1.) for r in moving],label=name)
        values=[[r['follower_raw_tared_tcp_wrench_'+str(i)] for i in (3,4,5)] for r in moving]
        axes[1,0].plot(t,np.linalg.norm(values,axis=1),label=name)
        axes[1,1].plot(t,[1e3*np.hypot(r['follower_local_error_x'],r['follower_local_error_y']) for r in moving],label=name)
        if native:
            for robot in ('leader','follower'):
                for side in ('left','right'):
                    axes[2,0].plot(t,[r[robot+'_finger_'+side+'_N'] for r in moving],label=name+': '+robot[0]+side[0])
            contacts=[all(r.get(robot+'_native_contacts') for robot in ('leader','follower')) for r in moving]
        else:
            c0=np.array([initial['object_center_GT_'+k] for k in 'xyz'])
            centers=np.array([[r['object_center_GT_'+k] for k in 'xyz'] for r in moving])
            axes[2,0].plot(t,1e3*np.linalg.norm(centers-c0,axis=1),label=name)
            contacts=[all(r.get(robot+'_grasp_state')=='attached' for robot in ('leader','follower')) for r in moving]
        axes[2,1].plot(t,np.array(contacts,int),label=name)
    titles=['Object rotation (solid) / leader reference (dashed)','Monotone leader progress rate',
            'Follower unfiltered incremental TCP moment','Follower local xy deflection',
            'Own finger forces' if native else 'Centre drift (diagnostic only)',
            'Both bilateral contacts present' if native else 'Both fixed grasps attached']
    units=['Angle [deg]','Progress rate','Moment norm [Nm]','Deflection [mm]',
           'Finger reaction [N]' if native else 'Drift [mm]','Present [0/1]']
    axes[1,0].axhline(.05,color='#b92b27',linestyle=':',label='Experimental stop envelope')
    axes[2,1].set_ylim(-.1,1.1)
    if native:
        for cap in (8.,26.):axes[2,0].axhline(cap,color='#b92b27',linestyle=':')
    for ax,title,unit in zip(axes.flat,titles,units):
        ax.set(title=title,xlabel='Time since rotation start [s]',ylabel=unit)
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('CP-8R-3: grasp-preserving completion / '+('native contact' if native else 'fixed fixture'),fontsize=14)
    for extension in ('svg','png'):fig.savefig(a.audit_directory/(a.output_stem+'.'+extension),dpi=150)
    plt.close(fig)


if __name__=='__main__':main()
