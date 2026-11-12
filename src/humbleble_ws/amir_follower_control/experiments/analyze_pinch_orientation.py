"""Offline frame diagnosis from recorded joints/GT; never a control input.

The hypothetical equal-load projection is a static illustration, not a contact
force reconstruction or a hardware load rating.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from amir_follower_control.core import ArmURDF, origin, rotation


def rotation_of_quaternion(q):
    x, y, z, w = [q[k] for k in ('x', 'y', 'z', 'w')]
    if abs(x*x + y*y + z*z + w*w - 1) > 1e-3:
        raise ValueError('non-unit evaluation quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--at-sim-time', type=float, required=True)
    args = parser.parse_args()
    directory = Path(args.directory)
    states, poses = {}, {}
    for line in (directory/'fx_positive/evaluation.jsonl').open():
        row = json.loads(line)
        if row['sim_time'] > args.at_sim_time:
            continue
        for robot in ('amir1', 'amir2'):
            if row['topic'] == f'/{robot}/joint_states':
                states[robot] = row
        if row['topic'] == 'world_GT':
            for transform in row['message']['transforms']:
                poses[transform['child_frame_id']] = (row['sim_time'], transform['transform'])
    metadata = json.loads((directory/'payload_initialization.json').read_text())
    result = {'scope': 'offline recorded frame diagnosis only; no runtime feedback',
              'requested_sim_time': args.at_sim_time,
              'load_projection_note': 'Equal payload weight / 2 projected on each finger sensor X; ignores moments, dynamics and non-planar contacts. Not an allowable force.',
              'candidate_note': 'Joint_5=0 is FK only. Payload orientation/clearance, sensor recalibration and load gates have NOT been tested.',
              'robots': {}}
    for robot in ('amir1', 'amir2'):
        row = states[robot]
        message = row['message']
        mapping = dict(zip(message['name'], message['position']))
        xml = (directory/f'{robot}.urdf').read_text()
        urdf = ET.fromstring(xml)
        parents = {joint.find('child').get('link'): joint for joint in urdf.findall('joint')}
        def frame(link):
            if link == 'base_footprint':
                return np.eye(4)
            joint = parents[link]
            transform = frame(joint.find('parent').get('link')) @ origin(joint.find('origin'))
            if joint.get('type') != 'fixed':
                name = joint.get('name')
                mimic = joint.find('mimic')
                if name in mapping:
                    value = mapping[name]
                elif name+'_mimic' in mapping:
                    value = mapping[name+'_mimic']
                elif mimic is not None:
                    value = float(mimic.get('multiplier', '1'))*mapping[mimic.get('joint')]+float(mimic.get('offset', '0'))
                else:
                    raise ValueError('missing measured joint '+name)
                movement = np.eye(4)
                movement[:3, :3] = rotation([float(v) for v in joint.find('axis').get('xyz').split()], value)
                transform = transform @ movement
            return transform
        pose_time, pose = poses[robot]
        base_rotation = rotation_of_quaternion(pose['rotation'])
        arm = ArmURDF(xml)
        q = np.array([mapping[name] for name in arm.names])
        original, _, _ = arm.evaluate(q)
        if not np.allclose(original, frame('tcp_link'), atol=1e-10):
            raise ValueError('offline full-tree FK disagrees with controller arm FK')
        sensors = {}
        for side in ('left', 'right'):
            sensor_name = f'finger_{side}_force_sensor'
            matches = [(g, g.find(f"sensor[@name='{sensor_name}']")) for g in urdf.findall('gazebo')
                       if g.find(f"sensor[@name='{sensor_name}']") is not None]
            if len(matches) != 1:
                raise ValueError('expected exactly one actual sensor '+sensor_name)
            gazebo, sensor = matches[0]
            if sensor.findtext('force_torque/frame') != 'child' or sensor.find('pose') is not None:
                raise ValueError('sensor requires a separately verified frame conversion')
            joint = urdf.find(f"joint[@name='{gazebo.get('reference')}']")
            link = joint.find('child').get('link')
            axis = base_rotation @ frame(link)[:3, 0]
            sensors[side] = {'joint': joint.get('name'), 'child_link': link,
                             'sensor_X_in_world': axis.tolist(),
                             'abs_sensor_X_dot_world_vertical': float(abs(axis[2])),
                             'equal_share_weight_projection_abs_N': float(metadata['total_model_mass_kg']*9.81/2*abs(axis[2]))}
        candidate_q = q.copy()
        candidate_q[4] = 0.
        candidate, _, candidate_jac = arm.evaluate(candidate_q)
        effort = urdf.find("joint[@name='Gripper']/limit").get('effort')
        result['robots'][robot] = {
            'joint_receipt_sim_time': row['sim_time'], 'joint_header': message['header']['stamp'],
            'GT_receipt_sim_time': pose_time, 'q_rad': q.tolist(), 'gripper_q_rad': mapping['Gripper'],
            'URDF_gripper_effort_Nm_not_hardware_rating': float(effort), 'sensors': sensors,
            'candidate_joint5_zero_FK_only': {
                'delta_G_B_m': (candidate[:3, 3]-original[:3, 3]).tolist(),
                'gripper_X_in_world': (base_rotation@candidate[:3, 0]).tolist(),
                'jacobian_rank': int(np.linalg.matrix_rank(candidate_jac, tol=1e-5)),
                'sigma_min': float(np.linalg.svd(candidate_jac, compute_uv=False)[-1])}}
    (directory/'pinch_orientation_analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
