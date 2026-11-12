#!/usr/bin/env python3
"""Offline CP-5 plots and graph checks; no command publication."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from amir_compliance_control.wrench import LocalModel


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trials', type=Path, nargs='+', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--urdf', type=Path, required=True)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    model = LocalModel(a.urdf.read_text())
    summaries = []
    for path in a.trials:
        result = json.loads((path/'result.json').read_text())
        rows = [json.loads(line) for line in (path/'samples.jsonl').read_text().splitlines()]
        active = [r for r in rows if 'reference_x' in r]
        graph = json.loads((path/'graph.json').read_text())
        outputs = [json.loads(line) for line in (path/'command_outputs.jsonl').read_text().splitlines()]
        nodes = ['/follower/follower_arm_admittance', '/follower/wrench_preprocessor', '/follower/safety_gate']
        inputs = {n: [t[0] for t in graph[n]['subscriptions']] for n in nodes}
        record = {'case':path.name, 'passed':result['passed'], 'fault':result['fault_test'],
            'shared_stop_latency_wall_s':result.get('dual_zero_latency_wall_s'),
            'direction_tests':result.get('direction_tests'),
            'recovery_mean_delta_xyz_m':result.get('recovery_mean_delta_xyz_m'),
            'max_virtual_offset_m':result.get('max_virtual_offset_m'),
            'max_abs_base_output':max(abs(o[k]) for o in outputs for k in ('vx', 'vy', 'wz')),
            'max_measured_z_error_m':max(abs(r['follower_tcp_z']-r['reference_z']) for r in active),
            'peak_joint_reaction_torque_Nm':result['peak_joint_reaction_torque_Nm'], 'follower_inputs':inputs}
        record['boundary_passed'] = not any(t.startswith('/leader/') or 'goal' in t or 'ground_truth' in t
            or '/world/' in t or t.endswith('/tf') or t.endswith('/tf_static') for topics in inputs.values() for t in topics)
        record['max_measured_xy_offset_m'] = max(abs(r['follower_tcp_'+k]-r['reference_'+k]) for r in active for k in 'xy')
        reference_rotation = model.frames([active[0][f'follower_q{i}_rad'] for i in range(1, 6)])[1][:3, :3]
        angles = []
        for row in active:
            r = model.frames([row[f'follower_q{i}_rad'] for i in range(1, 6)])[1][:3, :3]
            angles.append(float(np.arccos(np.clip((np.trace(r@reference_rotation.T)-1)/2, -1., 1.))))
        record['max_tool_rotation_angle_rad'] = max(angles)
        if all(robot+'_gt_y' in active[0] for robot in ('leader', 'follower')):
            record['base_gt_planar_span_m'] = {robot:float(np.linalg.norm([
                np.ptp([r[robot+'_gt_'+k] for r in active]) for k in 'xy'])) for robot in ('leader', 'follower')}
            record['max_encoder_speed_components'] = {robot:max(abs(r[robot+'_encoder_'+k]) for r in active for k in ('vx', 'vy', 'wz'))
                                                        for robot in ('leader', 'follower')}
        summaries.append(record)
        if result['fault_test'] == 'none':
            t = np.array([r['sim_time']-active[0]['sim_time'] for r in active])
            fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
            for i, k in enumerate('xy'):
                axes[0].plot(t, [r['applied_f'+k+'_N'] for r in active], label='Applied '+k)
                if 'filtered_f'+k+'_N' in active[0]:
                    axes[0].plot(t, [r['filtered_f'+k+'_N'] for r in active], '--', label='Filtered '+k)
                actual = [(r['follower_tcp_'+k]-r['reference_'+k])*1000 for r in active]
                virtual = [r.get('admittance_'+k, 0.)*1000 for r in active]
                axes[1+i].plot(t, actual, label='Measured joint FK '+k)
                axes[1+i].plot(t, virtual, '--', label='Virtual '+k)
                axes[1+i].axhline(.5, color='gray', linestyle=':'); axes[1+i].axhline(-.5, color='gray', linestyle=':')
                axes[1+i].set_ylabel(k+' offset [mm]')
            axes[0].set_ylabel('Force [N]')
            axes[3].plot(t, [(r['follower_tcp_z']-r['reference_z'])*1e6 for r in active], label='Measured joint FK z error')
            axes[3].set_ylabel('Height error [um]'); axes[3].set_xlabel('Simulation time after ARM_ONLY enable [s]')
            for axis in axes:
                axis.legend(fontsize=8); axis.grid(True, alpha=.3)
            fig.suptitle('CP-5 ARM_ONLY: free orientation, ungrasped; no mobile following')
            fig.tight_layout(); fig.savefig(a.output/(path.name+'.png'), dpi=150); plt.close(fig)
    (a.output/'summary.json').write_text(json.dumps(summaries, indent=2))
    with (a.output/'summary.csv').open('w') as f:
        keys = [k for k in summaries[0] if k != 'follower_inputs']
        writer = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore'); writer.writeheader(); writer.writerows(summaries)
    print(json.dumps(summaries, indent=2))
    assert all(r['passed'] and r['boundary_passed'] and r['max_abs_base_output'] == 0. for r in summaries)


if __name__ == '__main__': main()
