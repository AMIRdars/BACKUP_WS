"""Offline URDF/own-joint diagnosis of uncommanded orientation in a 4D task.

Does not run ROS or change a certificate. A kinematic coupling is not proof of
the unique cause of a measured contact-force transient.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from amir_follower_control.core import ArmURDF, allocate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--urdf',required=True)
    parser.add_argument('--trial',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    trial=json.loads(Path(args.trial).read_text())
    begin=trial['follower_enable_sim_time']
    end=trial['end_sim_time']
    rows=[json.loads(line) for line in Path(args.log).open()]
    rows=[r for r in rows if 'q_des' in r and begin<=r['sim_time']<=end]
    if len(rows)<2:
        raise ValueError('at least two enabled own-joint records required')
    arm=ArmURDF(Path(args.urdf).read_text())
    q=np.asarray(rows[0]['q'])
    t,_,j4=arm.evaluate(q)
    j6=arm.evaluate(q,full_jacobian=True)[2]
    task=np.array([0.,.0001,0.,0.])
    exact=np.linalg.lstsq(j4,task,rcond=1e-5)[0]
    _,values,vh=np.linalg.svd(j4,full_matrices=True)
    null=vh[np.sum(values>1e-5):].T
    if null.shape[1]:
        exact+=null@np.linalg.lstsq(j6[3:5]@null,-j6[3:5]@exact,rcond=1e-5)[0]
    bounded=allocate(arm,q,np.zeros(5),j4,task,np.zeros(3),.01,t[:3,3])
    timestamps=np.array([r['sim_time'] for r in rows])
    actual=np.array([arm.evaluate(r['q'],full_jacobian=True)[2]@r['dq'] for r in rows])
    measured_omega=np.sum(.5*(actual[1:,3:]+actual[:-1,3:])*np.diff(timestamps)[:,None],axis=0)
    delta=arm.evaluate(rows[-1]['q'])[0][:3,3]-t[:3,3]
    output={
        'trial_success':trial['success'],
        'sample_window_sim_time_s':[float(timestamps[0]),float(timestamps[-1])],
        'reference_q_rad':q.tolist(),
        'J4_rank':int(np.linalg.matrix_rank(j4,tol=1e-5)),
        'J4_singular_values':values.tolist(),
        'requested_v4_mps_mps_mps_radps':task.tolist(),
        'exact_4D_with_minimum_available_angular_xy':{
            'qdot_radps':exact.tolist(),
            'actual_twist6_mps_mps_mps_radps_radps_radps':(j6@exact).tolist(),
            'task_residual':(task-j4@exact).tolist(),
            'uncommanded_omega_xy_per_vy_rad_per_m':((j6@exact)[3:5]/task[1]).tolist()},
        'runtime_bounded_DLS_damping_0p02':{
            'qdot_radps':bounded.qdot.tolist(),
            'actual_twist6_mps_mps_mps_radps_radps_radps':(j6@bounded.qdot).tolist()},
        'measured_own_FK_G_delta_m':delta.tolist(),
        'measured_spatial_omega_integral_xyz_rad':measured_omega.tolist(),
        'linearized_omega_xy_from_measured_y_delta_rad':((j6@exact)[3:5]/task[1]*delta[1]).tolist(),
        'limitations':'Offline kinematics using measured own joints and URDF. No force model, motor torque measurement, proof of contact slip, or unique causal attribution. Rank 4 does not imply zero angular x/y.'}
    Path(args.output).write_text(json.dumps(output,indent=2,allow_nan=False))
    print(json.dumps(output,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
