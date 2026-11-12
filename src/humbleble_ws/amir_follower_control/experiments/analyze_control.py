"""Export controller evidence and unit-separated metrics; no synthetic runs."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from amir_follower_control.core import ArmURDF


def input_end_metrics(timestamps, position, omega_integral, input_end):
    """Require measured samples bracketing input end; never extrapolate a stop."""
    covered=bool(len(timestamps) and timestamps[0]<=input_end<=timestamps[-1])
    result={'requested_input_end_sim_time':float(input_end),
            'input_end_covered_by_log':covered,
            'input_end_sample_sim_time':None,
            'actual_G_B_delta_at_input_end_m':None,
            'spatial_omega_z_integral_at_input_end_rad':None}
    if covered:
        index=int(np.flatnonzero(timestamps<=input_end)[-1])
        result.update({'input_end_sample_sim_time':float(timestamps[index]),
                       'actual_G_B_delta_at_input_end_m':(position[index]-position[0]).tolist(),
                       'spatial_omega_z_integral_at_input_end_rad':float(omega_integral[index])})
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('log')
    p.add_argument('--urdf',required=True)
    p.add_argument('--output-directory',required=True)
    p.add_argument('--trial')
    a=p.parse_args()
    out=Path(a.output_directory)
    out.mkdir(parents=True,exist_ok=True)
    records=[json.loads(line) for line in Path(a.log).open()]
    trial=None
    if a.trial:
        trial=json.loads(Path(a.trial).read_text())
        if 'follower_enable_sim_time' in trial:
            start=trial['follower_enable_sim_time']-.03
            records=[r for r in records if r['sim_time']>=start]
            end=trial.get('end_sim_time',trial['follower_enable_sim_time']-trial.get('delay_s',0)+
                          trial.get('sim_duration_s',trial['duration_s']+1.))
            records=[r for r in records if r['sim_time']<=end]
    rows=[r for r in records if 'q_des' in r]
    result={'records':len(rows),'faults':[r.get('fault') for r in records if r.get('state')=='FAULT']}
    if trial is not None:
        result['supervisor_result']=trial
    if not rows:
        (out/'metrics.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        return
    arm=ArmURDF(Path(a.urdf).read_text())
    q=np.array([r['q'] for r in rows])
    desired=np.array([r['q_des'] for r in rows])
    position=np.array([arm.evaluate(row)[0][:3,3] for row in q])
    target_position=np.array([arm.evaluate(row)[0][:3,3] for row in desired])
    timestamps=np.array([r['sim_time'] for r in rows])
    actual_velocity=np.array([arm.evaluate(row)[2]@np.array(r['dq']) for row,r in zip(q,rows)])
    omega_integral=np.r_[0.,np.cumsum(.5*(actual_velocity[1:,3]+actual_velocity[:-1,3])*np.diff(timestamps))]
    error=np.array([r['error'] for r in rows])
    raw=np.array([r['raw'] for r in rows])
    measured_torque=[abs(r['sensor_joint_transmitted_torque_Nm']) for r in rows
                     if 'sensor_joint_transmitted_torque_Nm' in r]
    static_loads=[r['quasistatic_joint_load_Nm'] for r in rows if 'quasistatic_joint_load_Nm' in r]
    result.update({
        'duration_s':float(timestamps[-1]-timestamps[0]),
        'error_axis_units':['N','N','N','Nm'],
        'error_rms':np.sqrt(np.mean(error**2,axis=0)).tolist(),
        'error_peak_abs':np.max(np.abs(error),axis=0).tolist(),
        'tail_window_s':float(min(.5,timestamps[-1]-timestamps[0])),
        'tail_error_peak_to_peak':np.ptp(error[timestamps>=timestamps[-1]-.5],axis=0).tolist(),
        'raw_peak_abs':np.max(np.abs(raw),axis=0).tolist(),
        'tracking_peak_rad':np.max(np.abs(q-desired),axis=0).tolist(),
        'actual_G_B_delta_m':(position[-1]-position[0]).tolist(),
        'actual_spatial_omega_z_integral_rad':float(np.trapz(actual_velocity[:,3],timestamps)),
        'sensor_joint_torque_peak_Nm':max(measured_torque) if measured_torque else None,
        'quasistatic_joint_load_peak_abs_Nm':np.max(np.abs(static_loads),axis=0).tolist() if static_loads else None,
        'quasistatic_effort_margin_min_Nm':np.min(arm.effort-np.abs(static_loads),axis=0).tolist() if static_loads else None,
        'quasistatic_note':'Model gravity of links upstream of sensor plus measured downstream wrist wrench; excludes upstream inertia and friction. Not measured actuator torque.',
        'actual_G_B_span_m':np.ptp(position,axis=0).tolist(),
        'target_G_B_span_m':np.ptp(target_position,axis=0).tolist(),
        'max_admittance_velocity':np.max(np.abs([r['velocity'] for r in rows]),axis=0).tolist(),
        'min_joint_margin_rad':min(r['joint_margin'] for r in rows),
        'max_control_dt_s':max(r['dt'] for r in rows),
        'state_age_p99_s':np.quantile([r['age'] for r in rows],.99,axis=0).tolist(),
        'states':sorted(set(r['state'] for r in rows))})
    if trial is not None and 'excitation_start_sim_time' in trial:
        input_end=trial['excitation_start_sim_time']+trial['duration_s']
        result.update(input_end_metrics(timestamps,position,omega_integral,input_end))
    (out/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    columns=['sim_time','state',*[f'error_{x}' for x in ('Fx','Fy','Fz','tz')],
             *[f'q{i}' for i in range(1,6)],*[f'q_des{i}' for i in range(1,6)],
             'base_vx','base_vy','base_wz','G_B_x','G_B_y','G_B_z','margin_rad']
    with (out/'control.csv').open('w') as file:
        writer=csv.writer(file)
        writer.writerow(columns)
        for row,xyz in zip(rows,position):
            writer.writerow([row['sim_time'],row['state'],*row['error'],*row['q'],*row['q_des'],
                             *row['base_cmd'],*xyz,row['joint_margin']])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(7,1,sharex=True,figsize=(10,12))
    t=timestamps-timestamps[0]
    for index,label in enumerate(('Fx [N]','Fy [N]','Fz [N]','tz [Nm]')):
        axes[index].plot(t,error[:,index])
        axes[index].set_ylabel(label)
    axes[4].plot(t,position-position[0],label=['x','y','z'])
    axes[4].set_ylabel('G in body [m]')
    axes[4].legend()
    axes[5].plot(t,[r['base_cmd'] for r in rows],label=['vx','vy','wz'])
    axes[5].set_ylabel('base cmd [m/s,rad/s]')
    axes[5].legend()
    axes[6].plot(t,omega_integral)
    axes[6].set_ylabel('integral omega_z [rad]')
    axes[6].set_xlabel('simulation time since enable [s]')
    fig.tight_layout()
    fig.savefig(out/'response.png',dpi=150)
    plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
