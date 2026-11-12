"""Offline independent base metrology; GT is evaluation only.

Fits physics body velocity = M @ encoder body velocity using distinct x/y/yaw
experiments. It never changes a controller or issues transport permission.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


AXES = ('x', 'y', 'yaw')


def calibration_limits(follower, leader):
    """Use the smaller existing leader/follower caps, without new ratings."""
    speeds = np.asarray(follower['base_velocity_limits'], dtype=float)
    accelerations = np.asarray(follower['base_acceleration_limits'], dtype=float)
    leader_speeds = np.asarray([leader['rotation_max_base_speed']]*2+[leader['rotation_max_angular_speed']], dtype=float)
    leader_accelerations = np.asarray([leader['rotation_max_base_acceleration']]*2+[leader['rotation_max_angular_acceleration']], dtype=float)
    if any(v.shape != (3,) or not np.isfinite(v).all() or np.min(v) <= 0.
           for v in (speeds, accelerations, leader_speeds, leader_accelerations)):
        raise ValueError('existing leader/follower limits must be finite positive 3-vectors')
    return np.minimum(speeds, leader_speeds), np.minimum(accelerations, leader_accelerations)


def body_log(initial, current):
    """SE(2) logarithm in the initial base frame, including yaw unwrapping."""
    x, y = np.asarray(current[:2]) - np.asarray(initial[:2])
    c, s = math.cos(initial[2]), math.sin(initial[2])
    p = np.array([c*x+s*y, -s*x+c*y])
    theta = current[2] - initial[2]
    if abs(theta) < 1e-8:
        a = 1. - theta*theta/6.
        b = theta/2. - theta**3/24.
    else:
        a, b = math.sin(theta)/theta, (1.-math.cos(theta))/theta
    return np.r_[np.array([[a, b], [-b, a]]) @ p/(a*a+b*b), theta]


def fit_velocity(rows, source):
    """Use unique source timestamps, not callback/clock arrival timestamps."""
    unique = {}
    for row in rows:
        stamp = float(row[source+'_stamp_s'])
        pose = np.array([row[source+'_'+axis] for axis in AXES], dtype=float)
        if not math.isfinite(stamp) or not np.isfinite(pose).all():
            raise ValueError('nonfinite source pose/time')
        if stamp in unique and not np.array_equal(unique[stamp], pose):
            raise ValueError('changed pose with duplicate source timestamp')
        unique[stamp] = pose
    if len(unique) < 8:
        raise ValueError('at least eight unique source samples required')
    ordered = sorted(unique.items())
    t = np.array([x[0] for x in ordered])
    if np.any(np.diff(t) > .05+1e-9):
        raise ValueError('source interval exceeds existing 50 ms bound')
    poses = np.array([x[1] for x in ordered])
    poses[:, 2] = np.unwrap(poses[:, 2])
    values = np.array([body_log(poses[0], pose) for pose in poses])
    design = np.column_stack((t-t.mean(), np.ones(len(t))))
    fit = np.linalg.lstsq(design, values, rcond=None)[0]
    return dict(velocity=fit[0].tolist(), source_samples=len(t), duration_s=float(t[-1]-t[0]),
                maximum_fit_residual=np.max(abs(design@fit-values), axis=0).tolist())


def analyze(rows):
    groups = {}
    for row in rows:
        if row['phase'] != 'plateau':
            continue
        if row['robot'] not in ('leader', 'follower') or row['axis'] not in AXES:
            raise ValueError('unknown robot or excitation axis')
        sign = int(row['sign'])
        if sign not in (-1, 1):
            raise ValueError('positive and negative excitation required')
        age = [float(row['sim_time_s'])-float(row[s+'_stamp_s']) for s in ('encoder', 'gt')]
        if not np.isfinite(age).all() or min(age) < -.02-1e-9 or max(age) > .05+1e-9:
            raise ValueError('source age outside existing bounds')
        if abs(age[0]-age[1]) > .02+1e-9:
            raise ValueError('encoder/GT skew exceeds existing 20 ms bound')
        groups.setdefault((row['robot'], row['axis'], sign), []).append(row)
    report = dict(evaluation_only=True, applied_to_control=False,
                  transport_qualified=False, precision_qualified=False, robots={})
    for robot in ('leader', 'follower'):
        cases, encoded, physical = [], [], []
        for axis in AXES:
            for sign in (-1, 1):
                key = (robot, axis, sign)
                if key not in groups:
                    raise ValueError('independent positive/negative x/y/yaw coverage required')
                own, gt = fit_velocity(groups[key], 'encoder'), fit_velocity(groups[key], 'gt')
                index = AXES.index(axis)
                if own['velocity'][index]*sign <= 0. or gt['velocity'][index]*sign <= 0.:
                    raise ValueError('measured excitation sign is wrong or signal is zero')
                cases.append(dict(axis=axis, sign=sign, encoder=own, gt=gt))
                encoded.append(own['velocity']); physical.append(gt['velocity'])
        encoded, physical = np.array(encoded), np.array(physical)
        scale = np.max(abs(encoded), axis=0)
        if np.any(scale <= 0.):
            raise ValueError('unexcited base axis')
        normalized = encoded/scale
        solution, _, rank, singular = np.linalg.lstsq(normalized, physical, rcond=None)
        if rank != 3 or singular[-1]/singular[0] < 1e-6:
            raise ValueError('rank deficient independent excitation; arc-only fit forbidden')
        matrix = (solution/scale[:, None]).T
        residual = physical - encoded@matrix.T
        directional = {}
        for sign in (-1, 1):
            indices = [i for i, case in enumerate(cases) if case['sign'] == sign]
            x, y = normalized[indices], physical[indices]
            fit, _, sign_rank, _ = np.linalg.lstsq(x, y, rcond=None)
            if sign_rank != 3:
                raise ValueError('each sign must independently excite all three axes')
            directional[str(sign)] = (fit/scale[:, None]).T.tolist()
        report['robots'][robot] = dict(
            physics_from_encoder_matrix=matrix.tolist(), normalized_design_rank=int(rank),
            normalized_design_condition=float(singular[0]/singular[-1]), cases=cases,
            maximum_velocity_fit_residual=np.max(abs(residual), axis=0).tolist(),
            directional_matrices=directional,
            maximum_positive_negative_matrix_difference=float(np.max(abs(
                np.array(directional['1'])-np.array(directional['-1'])))))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.csv.open() as stream:
        report = analyze(list(csv.DictReader(stream)))
    args.output.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
