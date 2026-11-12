#!/usr/bin/env python3
"""Offline Phase 1 acceptance and figures; never a control input."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from amir_p0_evidence import evaluate, plot


def phase_checks(rows):
    active = [r for r in rows if r.get('admittance_enabled') == 'True' and r.get('adm_p_actual_m_0')]
    if not active:
        raise ValueError('Phase 1 requires measured active arm samples')
    def values(key, size):
        return np.array([[float(r[key+'_'+str(i)]) for i in range(size)] for r in active])
    actual, home = values('adm_p_actual_m', 3), values('adm_p_home_m', 3)
    r_actual = np.linalg.norm((actual-home)[:, :2], axis=1)
    r_logged = np.array([float(r['adm_r_arm_xy_m']) for r in active])
    q = values('follower_q_actual', 5)
    # Require a real movement above the existing 1 mm base-drift guard,
    # rather than interpreting a nonzero arm command as successful following.
    checks = {
        'explicit_base_assist_disabled': all(r.get('adm_enable_base_assist') == 'False' for r in active),
        'real_sensor_mode_only': all(r.get('adm_wrench_source_mode') == 'sensor' for r in active),
        'active_phase_mode_ARM_ONLY': all(r.get('adm_phase_mode') == 'ARM_ONLY' for r in active),
        'r_from_measured_FK': bool(np.max(abs(r_logged-r_actual)) < 1e-12),
        'arm_measured_travel_exceeds_1mm': bool(np.max(r_actual) > .001),
        'joint_actual_position_changes': bool(np.max(np.ptp(q, axis=0)) > .001),
        'IK_residual_finite': bool(np.isfinite(values('adm_ik_residual_xy_m_s', 2)).all()),
        'measured_joint_velocities_available': bool(np.isfinite(values('follower_qdot_actual', 5)).all()),
        'raw_FT_samples_available': bool(np.isfinite(values('follower_wrench_raw', 6)).all()),
    }
    return checks, active


def export(folder, rows, active, reference_on, reference_off):
    """Keep recorder and all source timestamps; do not label asynchronous data synchronous."""
    output = []
    previous_source = None
    for row in rows:
        if not row.get('adm_p_actual_m_0'):
            continue
        source = float(row['adm_source_stamp_s'])
        current = row.get('admittance_enabled') == 'True' and source != previous_source
        previous_source = source
        item = {'t_sim': row['sim_time'], 't_control_source': source,
                'fresh_control_sample': current, 'mode': row.get('adm_phase_mode'),
                'enable_base_assist': row.get('adm_enable_base_assist'),
                'wrench_source_mode': row.get('adm_wrench_source_mode'),
                'r_m': row['adm_r_arm_xy_m'],
                'safety_reason': row.get('stop_reason') or row.get('follower_reason'),
                'FT_source_stamp': row.get('follower_wrench_stamp_s'),
                'filtered_FT_source_stamp': row.get('filtered_wrench_stamp_s'),
                'joint_source_stamp': row.get('follower_joint_stamp_s'),
                'odom_source_stamp': row.get('follower_odom_stamp_s')}
        for prefix, source_prefix, size in [
            ('p_ee_B', 'adm_p_actual_m', 3), ('p_home_B', 'adm_p_home_m', 3),
            ('p_ref_B', 'adm_p_ref_xy_m', 2), ('delta_p_adm', 'adm_delta_p_adm_m', 2),
            ('e_track', 'adm_e_track_xy_m', 2), ('e_arm', 'adm_e_arm_m', 3),
            ('force_raw', 'follower_wrench_raw', 6), ('force_filtered', 'wrench_filtered', 6),
            ('q_actual', 'follower_q_actual', 5), ('qdot_meas', 'follower_qdot_actual', 5),
            ('qdot_cmd', 'adm_qdot_cmd_rad_s', 5), ('base_cmd', 'follower_base_final_reference', 3),
            ('base_odom_velocity', 'follower_odom_twist', 3), ('object_GT_evaluation_only', 'payload_model_gt_position', 3),
            ('ik_residual_xy_m_s', 'adm_ik_residual_xy_m_s', 2)]:
            for i in range(size):
                item[prefix+'_'+str(i)] = row.get(source_prefix+'_'+str(i))
        for key in ('min_joint_margin_rad', 'min_task_singular_value_m'):
            item[key] = row.get('adm_'+key)
        for robot in ('leader', 'follower'):
            for side in ('left', 'right'):
                for kind in ('contact', 'force_N'):
                    key = robot+'_native_'+side+'_'+kind
                    item[key] = row.get(key)
        output.append(item)
    with (folder/'phase1_timeseries.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output[0]))
        writer.writeheader(); writer.writerows(output)
    def val(key):
        return np.array([float(r.get(key) or 'nan') for r in active])
    t = val('sim_time'); t -= t[0]
    fig, axes = plt.subplots(6, 1, figsize=(12, 17), sharex=True, constrained_layout=True)
    axes[0].plot(t, val('adm_r_arm_xy_m')*1000, label='measured r = ||p_actual - p_home||xy')
    for value, label in [(reference_on, 'r_on'), (reference_off, 'r_off')]:
        axes[0].axhline(value*1000, linestyle=':', label=label+' reference only; Phase 2 disabled')
    axes[0].set_ylabel('Arm displacement [mm]')
    for i, axis in enumerate(('x', 'y')):
        home = val('adm_p_home_m_'+str(i))
        axes[1].plot(t, (val('adm_p_actual_m_'+str(i))-home)*1000, label='actual '+axis+' in follower/base_footprint')
        axes[1].plot(t, (val('adm_p_ref_xy_m_'+str(i))-home)*1000, '--', label='reference '+axis)
    axes[1].set_ylabel('TCP relative to home [mm]')
    for i, axis in enumerate(('vx', 'vy', 'wz')):
        axes[2].plot(t, val('follower_base_final_reference_'+str(i)), label='final cmd '+axis)
        axes[2].plot(t, val('follower_odom_twist_'+str(i)), '--', label='actual odom '+axis)
    axes[2].set_ylabel('Base [m/s; rad/s]')
    for i, axis in enumerate(('Fx', 'Fy', 'Fz')):
        axes[3].plot(t, val('wrench_filtered_'+str(i)), label=axis+' incremental, base axes')
    axes[3].set_ylabel('Filtered F/T force [N]')
    for i in range(5):
        q = val('follower_q_actual_'+str(i)); axes[4].plot(t, q-q[0], label='Joint_'+str(i+1)+' measured')
    axes[4].set_ylabel('Actual joint change [rad]')
    all_times = np.array([float(r['t_sim']) for r in output])-float(active[0]['sim_time'])
    modes = {'INIT': 0, 'ARM_ONLY': 1, 'BASE_ASSIST': 2, 'SAFE_STOP': 3}
    axes[5].step(all_times, [modes[r['mode']] for r in output], where='post', label='observed phase state')
    axes[5].set_yticks(list(modes.values()), list(modes)); axes[5].set_xlabel('Simulation time from first active sample [s]')
    for ax in axes:
        ax.grid(alpha=.25); ax.legend(fontsize=8, loc='best')
    fig.suptitle('Phase 1: native friction grasp, local measured F/T, base assist disabled\nGazebo post-grasp base restraint fixture; no claim of unfixtured brake performance')
    for suffix in ('png', 'pdf'):
        fig.savefig(folder/('phase1_measured.'+suffix), dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--held', action='store_true')
    parser.add_argument('--reference-r-on-m', type=float, default=.08)
    parser.add_argument('--reference-r-off-m', type=float, default=.05)
    args = parser.parse_args()
    if not 0 < args.reference_r_off_m < args.reference_r_on_m:
        parser.error('plot references require 0 < r_off < r_on; these never enable base assistance')
    folder = args.case.resolve()
    summary, rows = evaluate(folder, 'held_result.json' if args.held else 'result.json')
    checks, active = phase_checks(rows)
    summary.update(phase=1, phase2_enabled=False, phase1_checks=checks,
                   passed=summary['passed'] and all(checks.values()),
                   plot_threshold_scope='specification examples only; not control or safety thresholds',
                   source_alignment='recorder time plus original source stamps; no synthetic synchronization')
    (folder/'phase1_evidence.json').write_text(json.dumps(summary, indent=2))
    export(folder, rows, active, args.reference_r_on_m, args.reference_r_off_m)
    plot(folder, summary, rows)
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
