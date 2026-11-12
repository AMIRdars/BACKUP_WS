"""Quantify measured posture recovery and the commanded base subtraction."""
import argparse
import json
from pathlib import Path

import numpy as np

from amir_follower_control.core import ArmURDF, base_jacobian


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log')
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--trial', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    trial = json.loads(Path(args.trial).read_text())
    start = trial['follower_enable_sim_time'] - .03
    end = trial['end_sim_time']
    rows = [r for line in Path(args.log).open()
            if 'q_des' in (r := json.loads(line)) and start <= r['sim_time'] <= end]
    if len(rows) < 2:
        raise RuntimeError('insufficient physical controller samples')
    arm = ArmURDF(Path(args.urdf).read_text())
    transforms = [arm.evaluate(r['q'])[0] for r in rows]
    positions = np.array([tg[:3, 3] for tg in transforms])
    relative_rotations = [tg[:3, :3] @ transforms[0][:3, :3].T for tg in transforms]
    relative_yaw = np.array([np.arctan2(r[1, 0], r[0, 0]) for r in relative_rotations])
    t = np.array([r['sim_time'] for r in rows])
    displacement = np.linalg.norm((positions - positions[0])[:, :2], axis=1)
    peak = int(np.argmax(displacement))
    active = [r for r in rows if r['state'] == 'RECENTERING']
    reconstruction = []
    measured_residual = []
    base_contributions = []
    for prev, row in zip(rows[:-1], rows[1:]):
        dt = row['dt']
        if abs(row['sim_time'] - prev['sim_time'] - dt) > 1e-8:
            continue
        tg, _, jac = arm.evaluate(row['q'])
        jb = base_jacobian(tg[:3, 3])
        qdot_command = (np.array(row['q_des']) - prev['q_des']) / dt
        base_part = jb @ row['base_cmd']
        # The allocator logs residual = (v_des - Jb*v_base) - J*qdot.
        reconstruction.append(jac @ qdot_command + base_part
                              - row['velocity'] + row['residual'])
        measured_residual.append(np.array(row['velocity'])
                                 - jac @ row['dq'] - jb @ row['base_odom_twist'])
        base_contributions.append(base_part)
    result = {
        'supervisor_success': trial['success'],
        'reference_note': 'Displacement from first measured enabled FK sample; not an external pose.',
        'first_sample_sim_time': float(t[0]),
        'peak_horizontal_displacement_m': float(displacement[peak]),
        'peak_at_sim_time': float(t[peak]),
        'final_horizontal_displacement_m': float(displacement[-1]),
        'peak_to_final_reduction_fraction': float(1 - displacement[-1] / displacement[peak])
            if displacement[peak] > 0 else None,
        'relative_yaw_note': 'Same small-angle local orientation metric as recenter; not the spatial omega_z integral.',
        'peak_abs_relative_yaw_rad': float(np.max(np.abs(relative_yaw))),
        'final_abs_relative_yaw_rad': float(abs(relative_yaw[-1])),
        'yaw_peak_to_final_reduction_fraction': float(1 - abs(relative_yaw[-1]) / np.max(np.abs(relative_yaw)))
            if np.max(np.abs(relative_yaw)) > 1e-9 else None,
        'first_recenter_sim_time': active[0]['sim_time'] if active else None,
        'recenter_samples': len(active),
        'final_control_state': rows[-1]['state'],
        'peak_abs_base_cmd_mps_mps_radps': np.max(np.abs([r['base_cmd'] for r in rows]), axis=0).tolist(),
        'peak_abs_encoder_twist_mps_mps_radps': np.max(np.abs([r['base_odom_twist'] for r in rows]), axis=0).tolist(),
        'allocation_identity_max_abs_mps_mps_mps_radps': np.max(np.abs(reconstruction), axis=0).tolist(),
        'measured_task_residual_rms_mps_mps_mps_radps': np.sqrt(np.mean(np.array(measured_residual)**2, axis=0)).tolist(),
        'peak_abs_base_G_contribution_mps_mps_mps_radps': np.max(np.abs(base_contributions), axis=0).tolist(),
    }
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
