"""Summarize real local sensor/excitation logs; never fabricate trial rows."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from amir_follower_control.core import ArmURDF, transform_wrench


def stamp(msg):
    t=msg['header']['stamp']
    return t['sec']+t['nanosec']*1e-9


def sensor_summary(path):
    topics={}
    latest={}
    skews=[]
    raw=[]
    for line in path.open():
        record=json.loads(line)
        msg=record['message']
        name=record['topic']
        t=stamp(msg)
        topics.setdefault(name,[]).append((t,record['sim_receipt'],record['wall_elapsed']))
        latest[name]=t
        if len(latest)==3:
            skews.append(np.ptp(list(latest.values())))
        if name=='ft_sensor':
            w=msg['wrench']
            raw.append([w[k][a] for k in ('force','torque') for a in ('x','y','z')])
    result={'topics':{},'raw_axis_order':['Fx','Fy','Fz','tx','ty','tz']}
    for name,samples in topics.items():
        a=np.array(samples)
        periods=np.diff(a[:,0])
        ages=(a[:,1]-a[:,0])[a[:,1]>0]
        result['topics'][name]={
            'count':len(a),'sim_period_median_s':float(np.median(periods)),
            'sim_period_max_s':float(np.max(periods)),
            'stamp_nondecreasing':bool(np.all(periods>=0)),
            'receipt_age_p99_s':float(np.quantile(ages,.99)) if len(ages) else None,
            'measured_rtf':float((a[-1,0]-a[0,0])/(a[-1,2]-a[0,2]))}
    if raw:
        result['raw_mean']=np.mean(raw,axis=0).tolist()
        result['raw_std']=np.std(raw,axis=0).tolist()
        result['raw_peak_abs']=np.max(np.abs(raw),axis=0).tolist()
    if skews:
        result['latest_state_skew_p99_s']=float(np.quantile(skews,.99))
    return result


def pulses(path,urdf,outdir):
    arm=ArmURDF(Path(urdf).read_text())
    q=None
    qstamp=None
    rows=[]
    for line in path.open():
        record=json.loads(line)
        value=record['value']
        if record['kind'].endswith('joint_states'):
            mapping=dict(zip(value['name'],value['position']))
            q=np.array([mapping[n] for n in arm.names])
            qstamp=stamp(value)
        elif record['kind'].endswith('ft_sensor') and q is not None:
            t=stamp(value)
            if abs(t-qstamp)>.02:
                continue
            w=value['wrench']
            raw=np.array([w[k][a] for k in ('force','torque') for a in ('x','y','z')])
            tg,ts,_=arm.evaluate(q)
            wrench=transform_wrench(raw,ts,tg[:3,3],1.)
            rows.append([t,record['stage'],*record['input'],*wrench,*q])
    if not rows:
        raise ValueError('no aligned physical samples')
    names=['sim_time','stage','input_fx','input_fy','input_fz','input_tz',
           'Fx_G_B','Fy_G_B','Fz_G_B','tx_G_B','ty_G_B','tz_G_B',*arm.names]
    with (outdir/'calibration.csv').open('w') as out:
        writer=csv.writer(out)
        writer.writerow(names)
        writer.writerows(rows)
    groups={}
    for row in rows:
        groups.setdefault(row[1],[]).append(row)
    result={}
    for axis,name in enumerate(('fx','fy','fz','tz')):
        for sign in (1,-1):
            stage=name+str(sign)
            bias='bias_'+stage
            if stage not in groups or bias not in groups:
                continue
            def average(key):
                data=groups[key]
                return np.median([r[6:12] for r in data if r[0]>data[0][0]+.5],axis=0)
            response=average(stage)-average(bias)
            expected=sign*(.5 if axis<3 else .05)
            component=axis if axis<3 else 5
            result[stage]={'delta_wrench_G_B':response.tolist(),
                           'command':expected,'measured_gain_with_sign_plus1':float(response[component]/expected),
                           'samples':len(groups[stage])}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,sharex=True,figsize=(10,8))
    a=np.array([[r[0],*r[6:12],*r[2:6]] for r in rows],dtype=float)
    for index,component in enumerate((0,1,2,5)):
        axes[index].plot(a[:,0],a[:,1+component],label='measured, sign=+1')
        axes[index].plot(a[:,0],a[:,7+index],label='external command at G')
        axes[index].set_ylabel(['Fx [N]','Fy [N]','Fz [N]','tz [Nm]'][index])
    axes[0].legend()
    axes[-1].set_xlabel('simulation time [s]')
    fig.tight_layout()
    fig.savefig(outdir/'calibration.png',dpi=150)
    plt.close(fig)
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('log')
    p.add_argument('--urdf')
    p.add_argument('--output-directory',required=True)
    a=p.parse_args()
    directory=Path(a.output_directory)
    directory.mkdir(parents=True,exist_ok=True)
    result=pulses(Path(a.log),a.urdf,directory) if a.urdf else sensor_summary(Path(a.log))
    (directory/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
