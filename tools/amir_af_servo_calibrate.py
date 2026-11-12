#!/usr/bin/env python3
"""Fit existing position servo from actual ungrasped JTC state samples."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from amir_compliance_control.arm_servo_response import model_digest,validate_response


def main():
    p=argparse.ArgumentParser();p.add_argument('--probe',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    probe=a.probe.resolve();result=json.loads((probe/'result.json').read_text())
    if not result['passed'] or result['stage']!='AF-1' or any(v!='detached' for v in result['final_grasp'].values()):
        raise ValueError('passing ungrasped AF-1 required')
    graph=json.loads((probe/'graph.json').read_text())['follower_servo_parameters']
    gain=graph['gz_ros2_control']['position_proportional_gain'];rate=graph['controller_manager']['update_rate']
    names=[f'Joint_{i}' for i in range(1,6)]
    rows=[json.loads(s) for s in (probe/'jtc_state_samples.jsonl').read_text().splitlines()]
    rows=[r for r in rows if r['phase'].startswith(('force_','release')) and r['joint_names']==names
          and len(r['reference_positions'])==5 and len(r['feedback_velocities'])==5]
    response={};tau=[]
    for i,name in enumerate(names):
        e=np.array([(r['output_positions'] if len(r['output_positions'])==5 else r['reference_positions'])[i]-r['feedback_positions'][i] for r in rows])
        v=np.array([r['feedback_velocities'][i] for r in rows]);mask=(np.abs(e)>2e-6)&(np.abs(v)>1e-5)
        n=int(mask.sum());k=float(e[mask]@v[mask]/(e[mask]@e[mask])) if n else None
        verified=n>=20 and k is not None and abs(k/(gain*rate)-1)<=.15
        response[name]={'samples':n,'velocity_gain_s_inv':k,'verified':bool(verified)}
        tau.append(1/(gain*rate) if verified else 0.)
    folder=probe/'generated';xml=(folder/'follower.urdf').read_text()
    cert={'scope':'simulation_position_servo_response_only','passed':sum(t>0 for t in tau)>=3,
        'ungrasped':True,'joint_names':names,'normalized_urdf_sha256':model_digest(xml,folder),
        'position_proportional_gain':gain,'controller_update_rate_hz':rate,
        'open_loop_control':graph['arm_controller']['open_loop_control'],
        'joint_response':response,'joint_compensation_s':tau,
        'probe':str(probe),'jtc_samples_sha256':hashlib.sha256((probe/'jtc_state_samples.jsonl').read_bytes()).hexdigest(),
        'plugin_library_sha256':hashlib.sha256(Path('/opt/ros/humble/lib/libgz_hardware_plugins.so').read_bytes()).hexdigest(),
        'source':'https://github.com/ros-controls/gz_ros2_control/blob/0.7.20/gz_ros2_control/src/gz_system.cpp',
        'limitations':'Unexcited joints have zero compensation; this is not a force/load certification.'}
    validate_response(cert,xml,folder)
    a.output.write_text(json.dumps(cert,indent=2));print(json.dumps({'passed':cert['passed'],'joint_response':response,'joint_compensation_s':tau},indent=2))
if __name__=='__main__':main()
