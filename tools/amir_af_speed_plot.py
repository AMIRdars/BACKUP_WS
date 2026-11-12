#!/usr/bin/env python3
"""Export measured AF motion evidence; use distro Python with PYTHONNOUSERSITE=1."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args()
    folder=a.case.resolve();result=json.loads((folder/'result.json').read_text())
    rows=[r for r in csv.DictReader((folder/'timeseries.csv').open())
          if r['phase'].startswith('leader_translation') or r['phase'] in ('hold','fault_injected')]
    if not rows:raise ValueError('motion CSV required')
    def values(key):return np.array([float(r.get(key) or 'nan') for r in rows])
    def norm(key,n):return np.linalg.norm(np.column_stack([values(key+'_'+str(i)) for i in range(n)]),axis=1)
    absolute_t=values('sim_time');origin=absolute_t[0];t=absolute_t-origin
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,constrained_layout=True)
    ax=axes[0,0];lead=values('leader_odom_pose_0');lead-=lead[0]
    ax.plot(t,lead*1e3,label='leader odom');ax.plot(t,-values('adm_e_arm_m_0')*1e3,label='follower FK (-x)')
    if any(r.get('payload_model_gt_position_0') for r in rows):
        payload=values('payload_model_gt_position_0');payload-=payload[0]
        ax.plot(t,payload*1e3,'--',label='payload model GT world x (receipt clock)')
    ax.plot(t,-values('adm_delta_p_adm_m_0')*1e3,':',label='admittance target (-x)');ax.set_ylabel('Travel [mm]')
    # Deactivated base controllers may stop odom publication. Never display a
    # frozen, nonzero odom value as fresh post-stop velocity.
    odom=values('leader_odom_twist_0');odom[absolute_t-values('leader_odom_stamp_s')>.05]=np.nan
    ax=axes[0,1];ax.plot(t,odom*1e3,label='leader actual odom (fresh only)')
    ax.plot(t,values('leader_base_cmd_0')*1e3,':',label='leader request');ax.axhline(5.,color='r',linestyle='--',label='5 mm/s target');ax.set_ylabel('Speed [mm/s]')
    ax=axes[1,0];ax.plot(t,norm('wrench_filtered',3),label='filtered incremental force')
    high_rate=folder/'wrench_samples.csv'
    if high_rate.exists():
        received=[r for r in csv.DictReader(high_rate.open()) if r['kind']=='filtered'
                  and (r['phase'].startswith('leader_translation') or r['phase'] in ('hold','fault_injected'))]
        ax.plot([float(r['source_stamp_s'])-origin for r in received],
                [sum(float(r['wrench_'+str(i)])**2 for i in range(3))**.5 for r in received],
                linewidth=.7,alpha=.7,label='all received F/T samples (source time)')
    ax.axhline(.5,color='r',linestyle='--',label='unchanged stop boundary');ax.set_ylabel('Force norm [N]')
    ax=axes[1,1];ax.plot(t,norm('adm_e_track_xy_m',2)*1e6,label='XY tracking error norm')
    ax.plot(t,values('adm_e_arm_m_2')*1e6,label='height displacement');ax.set_ylabel('Arm error [um]')
    ax=axes[2,0];ax.plot(t,values('adm_base_drift_xy_yaw_0')*1e6,label='follower odom drift x')
    ax.plot(t,values('adm_base_drift_xy_yaw_1')*1e6,label='follower odom drift y');ax.set_ylabel('Base drift [um]')
    ax=axes[2,1]
    for i in range(5):ax.plot(t,values('follower_qdot_actual_'+str(i)),label=f'Joint_{i+1}')
    for sign in (-1,1):ax.axhline(sign*.03,color='r',linestyle='--',alpha=.5)
    ax.set_ylabel('Measured joint velocity [rad/s]')
    for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=7,loc='best')
    for ax in axes[-1]:ax.set_xlabel('Simulation time from goal [s]')
    fig.suptitle(f"{folder.name}: {result['stage']} passed={result['passed']}; "
                 f"peak odom={result.get('leader_odom_peak_m_s',0)*1000:.3f} mm/s")
    for suffix in ('png','pdf'):fig.savefig(folder/('measured_motion.'+suffix),dpi=160)
    plt.close(fig)


if __name__=='__main__':main()
