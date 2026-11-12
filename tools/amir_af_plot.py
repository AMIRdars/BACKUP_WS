#!/usr/bin/env python3
"""Standalone scientific plot of the preserved AF carry CSV."""
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
 p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 rows=[r for r in csv.DictReader((a.case/'timeseries.csv').open()) if r['phase'] in ('baseline','leader_translation','hold') and r.get('adm_p_actual_m_0')]
 def col(key):return np.array([float(r.get(key) or 0) for r in rows])
 t=col('sim_time');t-=t[0];leader=col('leader_odom_pose_0');leader-=leader[0]
 fig,axs=plt.subplots(4,1,figsize=(10,10),sharex=True,layout='constrained')
 axs[0].plot(t,leader*1000,label='Leader odom displacement')
 axs[0].plot(t,-col('adm_e_arm_m_0')*1000,label='Follower arm FK (-local X)')
 axs[0].plot(t,-col('adm_delta_p_adm_m_0')*1000,'--',label='Follower admittance target (-local X)')
 axs[0].set_ylabel('Travel (mm)');axs[0].legend(loc='upper left')
 axs[1].plot(t,np.hypot(col('adm_e_track_xy_m_0'),col('adm_e_track_xy_m_1'))*1000,label='XY tracking error')
 axs[1].plot(t,np.abs(col('adm_e_arm_m_2'))*1000,label='Height error')
 axs[1].set_ylabel('Error (mm)');axs[1].legend()
 force=np.linalg.norm(np.array([col('wrench_filtered_'+str(i)) for i in range(3)]),axis=0)
 torque=np.linalg.norm(np.array([col('wrench_filtered_'+str(i)) for i in range(3,6)]),axis=0)
 axs[2].plot(t,force,color='tab:blue',label='Incremental force');axs[2].axhline(.5,color='tab:blue',ls=':',label='Force experiment stop')
 axs[2].set_ylabel('Force (N)',color='tab:blue');other=axs[2].twinx();other.plot(t,torque,color='tab:orange',label='Incremental torque');other.axhline(.05,color='tab:orange',ls=':');other.set_ylabel('Torque (Nm)',color='tab:orange')
 axs[2].legend(loc='upper left')
 axs[3].plot(t,np.hypot(col('adm_base_drift_xy_yaw_0'),col('adm_base_drift_xy_yaw_1'))*1e6,label='Follower base odom drift')
 axs[3].plot(t,np.hypot(col('follower_base_cmd_0'),col('follower_base_cmd_1'))*1000,label='Follower base command (mm/s)')
 axs[3].set_ylabel('Drift (um) / cmd (mm/s)');axs[3].legend();axs[3].set_xlabel('Simulation time from baseline (s)')
 for ax in axs:ax.grid(alpha=.25)
 fig.suptitle('ARM_ONLY: fixed grasp, base zero, XY DLS + nullspace pose hold')
 a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160);plt.close(fig)
if __name__=='__main__':main()
