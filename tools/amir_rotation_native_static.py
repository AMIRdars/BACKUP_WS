"""Gate-0 statistics from native-contact OWN data; never a yaw certificate.

The measured static noise is reported, not compared with invented ratings.
Gate-1/2 positive/negative evidence is still required before autonomous yaw.
"""
import math
import numpy as np
from amir_compliance_control.wrench import finite


class NativeStaticYaw:
    def __init__(self,geometry,*,force_limit_N,torque_limit_Nm):
        self.geometry=geometry
        self.force_limit=float(force_limit_N);self.torque_limit=float(torque_limit_Nm)
        if not np.isfinite([self.force_limit,self.torque_limit]).all() or min(self.force_limit,self.torque_limit)<=0.:
            raise ValueError('measured native six-axis excitation envelope required')
        self.records=[];self.baseline=None;self.previous=None
        self.maximum_force=0.;self.maximum_moment=0.

    def add(self,row):
        q=finite(row['q_rad'],(5,));qd=finite(row['qdot_rad_s'],(5,));w=finite(row['wrench_tcp'],(6,))
        v=finite(row['base_twist'],(3,));t=float(row['joint_stamp']);now=float(row['timestamp'])
        stamps=finite([t,row['wrench_stamp'],row['odom_stamp']],(3,))
        # Retain the existing chapter-23 past/future/skew policies.
        if not math.isfinite(now) or np.any(now-stamps > .05+1e-12) or np.any(now-stamps < -.02-1e-12) or np.ptp(stamps)>.02+1e-12:
            raise ValueError('native static own sources stale/future/skewed')
        if row['grasp_attached'] is not True:raise ValueError('native own grasp not retained')
        if np.max(abs(v))>=.0001 or np.max(abs(qd))>=.001:
            raise ValueError('native static base and arm must remain stationary')
        measured=self.geometry.measure(q,compute_jacobian=False)
        if not measured['geometry_valid']:raise ValueError(measured['valid_reason'])
        if self.baseline is None:self.baseline=w.copy()
        force=float(np.linalg.norm((w-self.baseline)[:3]));moment=float(np.linalg.norm((w-self.baseline)[3:]))
        self.maximum_force=max(self.maximum_force,force);self.maximum_moment=max(self.maximum_moment,moment)
        # Check EVERY raw F/T event, including repeated joint snapshots.
        if force>=self.force_limit or moment>=self.torque_limit:
            raise ValueError(f'native static incremental wrench envelope exceeded: {force:.9g} N, {moment:.9g} Nm')
        if self.previous is not None:
            old_t,old_q=self.previous
            if t==old_t:
                if not np.array_equal(q,old_q):raise ValueError('native static joint/FK discontinuity')
                return
            if not 0.<t-old_t<=.05+1e-12:raise ValueError('native static joint source gap/discontinuity')
            if np.any(abs(q-old_q)>.001*(t-old_t)+1e-12):
                raise ValueError('native joint displacement contradicts stationary qualification')
        self.previous=(t,q.copy())
        self.records.append(dict(timestamp=now,joint_stamp=t,q_rad=q.tolist(),qdot_rad_s=qd.tolist(),
            psi_ee_B=measured['psi_ee_B'],projection_norm=measured['projection_norm'],joint_margin_rad=measured['joint_margin_rad'],
            wrench_tcp=w.tolist(),base_twist=v.tolist(),grasp_attached=True))

    def finish(self):
        if len(self.records)<8:raise ValueError('at least eight measured native static samples required')
        angles=np.unwrap([x['psi_ee_B'] for x in self.records]);w=np.array([x['wrench_tcp'] for x in self.records])
        return dict(gate0_static_passed=True,autonomous_yaw_qualification_issued=False,
            gate1_gate2='NOT RUN: positive/negative native passive yaw evidence required',
            input_scope='own joints/FK, own odometry, own raw TCP F/T and own native grasp only',
            samples=len(self.records),measured_duration_s=self.records[-1]['joint_stamp']-self.records[0]['joint_stamp'],
            yaw_heading_frame=self.geometry.heading_frame,mean_yaw_rad=float(np.mean(angles)),yaw_std_rad=float(np.std(angles,ddof=1)),
            minimum_projection_norm=min(x['projection_norm'] for x in self.records),minimum_joint_margin_rad=min(x['joint_margin_rad'] for x in self.records),
            W_0=np.mean(w,axis=0).tolist(),wrench_std=np.std(w,axis=0,ddof=1).tolist(),
            maximum_incremental_force_N=self.maximum_force,maximum_incremental_moment_Nm=self.maximum_moment,
            force_limit_N=self.force_limit,torque_limit_Nm=self.torque_limit)


def export_csv(source,destination,geometry=None):
    """Preserve every raw event and fault, including duplicate joint stamps."""
    import csv,json
    scalars=['timestamp','joint_stamp','wrench_stamp','odom_stamp','grasp_attached',
        'psi_ee_B','projection_norm','geometry_valid','fault']
    vectors={'q_rad':[f'q{i}_rad' for i in range(1,6)],
        'qdot_rad_s':[f'qdot{i}_rad_s' for i in range(1,6)],
        'wrench_tcp':['fx_N','fy_N','fz_N','tx_Nm','ty_Nm','tz_Nm'],
        'base_twist':['vx_m_s','vy_m_s','wz_rad_s']}
    count=0
    with destination.open('w',newline='') as out,source.open() as raw:
        writer=csv.DictWriter(out,fieldnames=scalars+[name for names in vectors.values() for name in names]);writer.writeheader()
        for line in raw:
            row=json.loads(line);value={key:row.get(key) for key in scalars}
            for key,names in vectors.items():value.update(zip(names,row.get(key,[])))
            if geometry is not None and 'q_rad' in row:
                try:
                    measured=geometry.measure(row['q_rad'],compute_jacobian=False)
                    value.update({key:measured[key] for key in ('psi_ee_B','projection_norm','geometry_valid')})
                except (ValueError,KeyError,TypeError) as exc:value['fault']=value.get('fault') or str(exc)
            writer.writerow(value);count+=1
    return count
