#!/usr/bin/env python3
"""Offline CP-3 comparison. GT and logged traces never flow back to control."""
import argparse
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trials', type=Path, nargs='+', required=True)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    model = ET.parse(a.run_directory/'leader.urdf')
    limits = [model.find(f"joint[@name='Joint_{i}']/limit") for i in range(1, 6)]
    effort = np.array([float(e.get('effort')) for e in limits])
    lower, upper = [np.array([float(e.get(key)) for e in limits]) for key in ('lower', 'upper')]
    records = []
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    for path in a.trials:
        result = json.loads((path/'result.json').read_text())
        rows = [json.loads(line) for line in (path/'samples.jsonl').read_text().splitlines()]
        motion = [r for r in rows if r['phase'] in ('motion', 'fault_injected', 'settle')]
        hold = [r for r in rows if r['phase'] == 'hold']
        record = {'case': path.name, 'passed': result['passed'], 'fault_test': result['fault_test'],
            'delay_s': result['delay_s'], 'speed_m_s': result['speed_m_s'],
            'follower_speed_limit_m_s': result.get('follower_speed_limit_m_s', result['speed_m_s']),
            'dual_zero_latency_wall_s': result.get('dual_zero_latency_wall_s')}
        output = [json.loads(line) for line in (path/'command_outputs.jsonl').read_text().splitlines()]
        onset = {r: next((v['sim_time'] for v in output if v['robot'] == r and abs(v['vx']) > .1*result['speed_m_s']), None)
                 for r in ('leader', 'follower')}
        if all(v is not None for v in onset.values()):
            record['measured_output_delay_s'] = onset['follower']-onset['leader']
        if motion:
            q = np.array([[r[f'{robot}_q{i}_rad'] for i in range(1, 6)]
                          for r in motion for robot in ('leader', 'follower')])
            record['minimum_joint_margin_rad'] = float(np.min(np.minimum(q-lower, upper-q)))
            record['motion_RTF'] = (motion[-1]['sim_time']-motion[0]['sim_time'])/(motion[-1]['wall_time']-motion[0]['wall_time'])
            record['payload_height_range_m'] = max(r['payload_z'] for r in motion)-min(r['payload_z'] for r in motion)
        if result['fault_test'] == 'none' and result['passed'] and motion and hold:
            time = np.array([r['sim_time']-motion[0]['sim_time'] for r in motion])
            distance = np.array([r['payload_x']-hold[-1]['payload_x'] for r in motion])
            record['payload_distance_m'] = float(distance[-1])
            tau = np.array([[r.get(f'{robot}_joint_torque_{i}_Nm', 0.) for i in range(1, 6)]
                            for r in motion for robot in ('leader', 'follower')]).reshape(len(motion), 2, 5)
            ratios = np.max(tau/effort, axis=(1, 2))
            label = f"{path.name}: delay={result['delay_s']:.2f}s"
            if result.get('follower_speed_limit_m_s', result['speed_m_s']) < result['speed_m_s']:
                label += ', speed limited'
            for robot in ('leader', 'follower'):
                bias = np.mean([[r[robot+'_'+k] for k in ('fx', 'fy', 'fz')] for r in hold], axis=0)
                force = np.array([[r[robot+'_'+k] for k in ('fx', 'fy', 'fz')] for r in motion])
                change = np.linalg.norm(force-bias, axis=1)
                record[robot+'_peak_force_change_N'] = float(np.max(change))
                if robot == 'follower':
                    axes[0].plot(time, change, label=label)
            axes[1].plot(time, distance*1000., label=label)
            axes[2].plot(time, ratios, label=label)
            record['peak_joint_effort_ratio'] = float(np.max(ratios))
        if result['fault_test'] != 'none' and result['passed']:
            late = [r for r in rows if r['wall_time'] > rows[-1]['wall_time']-.2]
            if len(late) > 1:
                dt = late[-1]['wall_time']-late[0]['wall_time']
                record['post_stop_gt_x_speed_m_s'] = {
                    robot: abs(late[-1][robot+'_gt_x']-late[0][robot+'_gt_x'])/dt for robot in ('leader', 'follower')}
        records.append(record)
    axes[0].set_ylabel('Follower wrench change |dF| [N]')
    axes[1].set_ylabel('Payload GT displacement [mm]')
    axes[2].set_ylabel('Max joint reaction / URDF effort')
    axes[2].set_xlabel('Simulation time after motion start [s]')
    axes[2].axhline(1., color='red', linestyle='--', linewidth=1, label='model stop limit')
    for axis in axes:
        axis.grid(True, alpha=.3)
        axis.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(a.output/'comparison.png', dpi=160)
    plt.close(fig)
    (a.output/'summary.json').write_text(json.dumps(records, indent=2))
    with (a.output/'comparison.csv').open('w') as f:
        keys = sorted({k for record in records for k in record})
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(records)
    print(json.dumps(records, indent=2))


if __name__ == '__main__':
    main()
