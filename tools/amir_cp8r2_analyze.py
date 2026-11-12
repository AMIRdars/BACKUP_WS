#!/usr/bin/env python3
"""Offline local follower evidence. Evaluation ground truth is never feedback."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from amir_cp8r1_analyze import analyze, rz2


def kinematic_budget(initial, final):
    """Offline GT/encoder closure, using only the INITIAL measured lever.

    This explains geometric drift; it does not identify physical wheel/slip
    parameters and is never an input to a controller.
    """
    xy = lambda row, prefix: np.array([row[prefix+k] for k in 'xy'])
    r0 = rz2(initial['leader_GT_yaw'])
    radius = r0.T@(xy(initial, 'object_center_GT_')-xy(initial, 'leader_GT_'))
    actual = xy(final, 'leader_GT_')-xy(initial, 'leader_GT_')
    encoder = rz2(initial['leader_GT_yaw']-initial['leader_encoder_yaw'])@(
        xy(final, 'leader_encoder_')-xy(initial, 'leader_encoder_'))
    gt_yaw = final['leader_GT_yaw']-initial['leader_GT_yaw']
    enc_yaw = final['leader_encoder_yaw']-initial['leader_encoder_yaw']
    translation = actual-encoder
    yaw_contribution = r0@(rz2(gt_yaw)-rz2(enc_yaw))@radius
    closure = actual+r0@(rz2(gt_yaw)-np.eye(2))@radius
    return {'scope': 'evaluation only; initial measured rigid lever, no fitted robot parameter',
            'leader_encoder_delta_yaw_rad': enc_yaw, 'leader_GT_delta_yaw_rad': gt_yaw,
            'translation_disagreement_xy_m': translation.tolist(),
            'yaw_disagreement_contribution_xy_m': yaw_contribution.tolist(),
            'leader_rigid_center_closure_xy_m': closure.tolist(),
            'object_actual_delta_xy_m': (xy(final, 'object_center_GT_')-xy(initial, 'object_center_GT_')).tolist()}


def analyze_local(folder):
    summary = analyze(folder)
    result = json.loads((folder/'result.json').read_text())
    rows = [json.loads(line) for line in (folder/'samples.jsonl').open()]
    moving = [r for r in rows if r['phase'] in ('rotation', 'hold')]
    before = [r for r in rows if r['phase'] == 'baseline']
    if before and moving:
        summary['kinematic_center_budget'] = kinematic_budget(before[-1], moving[-1])
    summary.update({key: result.get(key) for key in (
        'follower_input_boundary_passed', 'final_follower_speed_m_s', 'final_follower_local_error_xy_yaw')})
    summary['follower_accepted_goals'] = result.get('states', {}).get('follower', {}).get('accepted_goals')
    summary['independent_unfiltered_guard_in_runtime_debug'] = 'raw_tared_tcp_wrench' in result.get('last_follower_debug', {})
    summary['wrench_violation'] = result.get('last_follower_debug', {}).get('wrench_violation')
    policy = yaml.safe_load((folder/'follower_policy.yaml').read_text())['/follower/follower_yaw_arm_base']['ros__parameters']
    for key in ('base_gains', 'arm_task_metric', 'base_force_acceleration_gains', 'base_force_velocity_gains',
                'wrench_force_limit_N', 'wrench_torque_limit_Nm'):
        summary[key] = policy.get(key)
    for key in ('local_following_passed', 'center_drift_criterion_passed', 'arm_command_writer_counts',
                'post_stop_observation_s', 'both_grasps_retained_after_stop'):
        summary[key] = result.get(key)
    for axis in ('x', 'y', 'yaw'):
        values = [abs(r['follower_local_error_'+axis]) for r in moving]
        summary['follower_peak_local_error_'+axis] = max(values) if values else None
    for kind in ('raw_tared_tcp_wrench', 'filtered_tcp_wrench'):
        for start, suffix in ((0, 'force_N'), (3, 'torque_Nm')):
            values = [[r['follower_'+kind+'_'+str(i)] for i in range(start, start+3)]
                      for r in moving if 'follower_'+kind+'_'+str(start+2) in r]
            summary['follower_peak_'+kind+'_'+suffix] = float(np.max(np.linalg.norm(values, axis=1))) if values else None
    for robot in ('leader', 'follower'):
        stopped = [r for r in rows if r['phase'] == 'safe_stop']
        summary[robot+'_fixed_grasp_retained_after_stop'] = bool(stopped) and all(
            r.get(robot+'_grasp_state') == 'attached' for r in stopped)
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory', type=Path, required=True)
    a = p.parse_args()
    results = [analyze_local(f.parent) for f in sorted(a.audit_directory.glob('*/result.json'))]
    (a.audit_directory/'comparison.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    keys = sorted({k for r in results for k in r})
    with (a.audit_directory/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps([{k: r[k] for k in ('trial', 'passed', 'error', 'object_GT_delta_deg', 'object_center_max_drift_m') if k in r}
                      for r in results], indent=2))


if __name__ == '__main__':
    main()
