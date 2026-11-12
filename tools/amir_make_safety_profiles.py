#!/usr/bin/env python3
"""Validate measured joint reaction frames/loads and derive simulation profiles."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from amir_follower_control.core import ArmURDF
from amir_compliance_control.wrench import LocalModel


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe', type=Path, required=True)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--motion-evidence', type=Path, required=True)
    p.add_argument('--allow-ungrasped-base-test', action='store_true',
                   help='explicit simulation-only single-base sign test; never transport')
    a = p.parse_args()
    r = json.loads(a.probe.read_text())
    motion = json.loads(a.motion_evidence.read_text())
    results = {}
    for robot in ('leader', 'follower'):
        xml = (a.run_directory/(robot+'.urdf')).read_text()
        model, arm = LocalModel(xml), ArmURDF(xml)
        msg = r['latest']['/'+robot+'/joint_states']
        for moment in ('first', 'latest'):
            odom = r[moment]['/'+robot+'/mecanum_drive_controller/odometry']
            if odom['header']['frame_id'] != robot+'/odom' or odom['child_frame_id'] != robot+'/base_footprint':
                raise ValueError('encoder frame namespace not confirmed')
        mapping = dict(zip(msg['name'], msg['position']))
        q = np.array([mapping[n] for n in model.names])
        w = r['latest']['/'+robot+'/ft_sensor']['wrench']
        raw = np.array([w['force'][k] for k in 'xyz']+[w['torque'][k] for k in 'xyz'])
        expected = arm.quasistatic_joint_load(q, raw, 1.)
        projected = []
        for i, name in enumerate(model.names):
            topic = '/'+robot+('/ft_sensor' if i == 4 else '/joint_load/'+name)
            sample = r['latest'][topic]
            frame = (robot+'/Joint_5/wrist_ft_sensor' if i == 4 else robot+'/'+name+'/joint_load_'+name)
            if sample['header']['frame_id'] != frame or r['counts'][topic] < 100:
                raise ValueError('joint reaction sensor frame/data not confirmed')
            projected.append(float(model.axes[i]@np.array([sample['wrench']['torque'][k] for k in 'xyz'])))
        error = np.abs(np.array(projected)-expected)
        if np.any(error > .02+.02*np.abs(expected)):
            raise ValueError('reaction sensor projection disagrees with static model')
        speeds = []
        for axis in ('x', 'y', 'yaw'):
            case = next(v for v in motion if v['robot'] == robot and v['test'] == 'base_'+axis)
            if not case['passed']:
                raise ValueError('motion envelope lacks passing single-robot evidence')
            speeds.append(case['speed'])
        profile = {'robot': robot, 'simulation_only': True,
            'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
            'world_sha256': hashlib.sha256((a.run_directory/'world.sdf').read_bytes()).hexdigest(),
            'controller_yaml_file': str((a.run_directory/(robot+'_controllers.yaml')).resolve()),
            'controller_yaml_sha256': hashlib.sha256((a.run_directory/(robot+'_controllers.yaml')).read_bytes()).hexdigest(),
            'base_controller_yaml_file': str((a.run_directory/(robot+'_base_controller.yaml')).resolve()),
            'base_controller_yaml_sha256': hashlib.sha256((a.run_directory/(robot+'_base_controller.yaml')).read_bytes()).hexdigest(),
            'max_base_velocity': speeds, 'joint_margin_rad': .01,
            'effort_limits_source': 'selected URDF joint/limit/@effort',
            'joint_reaction_projection_verified': True,
            'measured_static_joint_torque_Nm': projected,
            'predicted_static_joint_torque_Nm': expected.tolist(),
            'max_static_projection_error_Nm': float(np.max(error)),
            'probe': str(a.probe.resolve()), 'motion_evidence': str(a.motion_evidence.resolve()),
            'require_grasp_for_base_motion': not a.allow_ungrasped_base_test,
            'ungrasped_base_test_only': a.allow_ungrasped_base_test,
            'scope': 'simulated transmitted joint torque; not certified hardware bearing/drive limits'}
        (a.run_directory/(robot+'_safety.json')).write_text(json.dumps(profile, indent=2))
        results[robot] = profile
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
