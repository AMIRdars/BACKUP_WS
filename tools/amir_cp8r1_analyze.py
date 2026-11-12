#!/usr/bin/env python3
"""Read-only CP-8R-1 metrics, including independent GT/encoder disagreement."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def rz2(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def virtual_center_drift(rows):
    """A rigid VIRTUAL centre implied by actual GT base pose; no real object."""
    before = [r for r in rows if r['phase'] == 'baseline']
    moving = [r for r in rows if r['phase'] in ('rotation', 'hold') and r.get('center_odom_x') is not None]
    if not before or not moving:
        return None
    initial = before[-1]
    center = np.array([moving[0]['center_odom_'+k] for k in 'xy'])
    b0 = np.array([initial['leader_encoder_'+k] for k in 'xy'])
    body_radius = rz2(initial['leader_encoder_yaw']).T@(center-b0)
    world_center = np.array([initial['leader_GT_'+k] for k in 'xy'])+rz2(initial['leader_GT_yaw'])@body_radius
    points = np.array([np.array([r['leader_GT_'+k] for k in 'xy'])+rz2(r['leader_GT_yaw'])@body_radius for r in moving])
    return float(np.max(np.linalg.norm(points-world_center, axis=1)))


def analyze(folder):
    result = json.loads((folder/'result.json').read_text())
    rows = [json.loads(line) for line in (folder/'samples.jsonl').open()]
    moving = [r for r in rows if r['phase'] in ('rotation', 'hold')]
    summary = {k: result.get(k) for k in ('condition', 'delta_deg', 'passed', 'error', 'error_phase',
        'leader_GT_delta_yaw_rad', 'gt_angle_ratio', 'object_GT_delta_yaw_rad', 'object_center_drift_m',
        'both_outputs_zero_after_stop', 'leader_input_boundary_passed', 'command_writer_counts')}
    summary['trial'] = folder.name
    summary['rows'] = len(rows)
    if result.get('leader_GT_delta_yaw_rad') is not None:
        summary['leader_GT_delta_deg'] = math.degrees(result['leader_GT_delta_yaw_rad'])
        summary['leader_GT_angle_error_deg'] = summary['leader_GT_delta_deg']-result['delta_deg']
    if result.get('condition') == 'leader_ungrasped':
        summary['virtual_center_GT_equivalent_max_drift_m'] = virtual_center_drift(rows)
    else:
        before = [r for r in rows if r['phase'] == 'baseline' and r.get('object_center_GT_z') is not None]
        if before and moving:
            b = before[-1]
            initial = np.array([b['object_center_GT_'+k] for k in 'xyz'])
            points = np.array([[r['object_center_GT_'+k] for k in 'xyz'] for r in moving])
            summary['object_center_max_drift_m'] = float(np.max(np.linalg.norm(points-initial, axis=1)))
            summary['object_height_max_error_m'] = float(np.max(abs(points[:, 2]-initial[2])))
            summary['object_max_tilt_from_vertical_rad'] = float(max(math.acos(np.clip(r['object_vertical_axis_dot_z'], -1., 1.)) for r in moving))
        if result.get('object_GT_delta_yaw_rad') is not None:
            summary['object_GT_delta_deg'] = math.degrees(result['object_GT_delta_yaw_rad'])
    for robot in ('leader', 'follower'):
        margins = [r[robot+'_joint_margin_rad'] for r in moving if robot+'_joint_margin_rad' in r]
        summary[robot+'_min_joint_margin_rad'] = min(margins) if margins else None
        forces = [[r[robot+'_wrist_raw_'+k] for k in ('fx', 'fy', 'fz')]
                  for r in moving if robot+'_wrist_raw_fx' in r]
        summary[robot+'_peak_raw_wrist_force_norm_N'] = float(np.max(np.linalg.norm(forces, axis=1))) if forces else None
        errors = [r[robot+'_grasp_constraint_position_error_m'] for r in moving if robot+'_grasp_constraint_position_error_m' in r]
        summary[robot+'_max_grasp_constraint_position_error_m'] = max(errors) if errors else None
        stopped = [r for r in rows if r['phase'] == 'safe_stop' and robot+'_q1_rad' in r]
        summary[robot+'_post_stop_max_joint_span_rad'] = float(max(
            np.ptp([r[robot+'_q'+str(i)+'_rad'] for r in stopped]) for i in range(1, 6))) if stopped else None
        summary[robot+'_fixed_grasp_retained_after_stop'] = all(
            r.get(robot+'_grasp_state') == 'attached' for r in stopped) if stopped and result['condition'] == 'synchronized_fixed' else None
        for component in ('vx', 'vy', 'wz'):
            values = [abs(r[robot+'_output_'+component]) for r in moving if r.get(robot+'_output_'+component) is not None]
            summary[robot+'_peak_output_'+component] = max(values) if values else None
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory', type=Path, required=True)
    a = p.parse_args()
    results = [analyze(f.parent) for f in sorted(a.audit_directory.glob('*/result.json'))]
    (a.audit_directory/'comparison.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    keys = sorted({k for r in results for k in r})
    with (a.audit_directory/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps(results, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
