#!/usr/bin/env python3
"""Offline CP-4 evidence summary; never imported by control nodes."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for path in args.trials:
        report = json.loads((path/'result.json').read_text())
        rows = [json.loads(line) for line in (path/'samples.jsonl').read_text().splitlines()]
        graph = json.loads((path/'graph.json').read_text())
        outputs = [json.loads(line) for line in (path/'command_outputs.jsonl').read_text().splitlines()]
        follower_nodes = ['/follower/follower_hold', '/follower/safety_gate']
        inputs = {n: [v[0] for v in graph[n]['subscriptions']] for n in follower_nodes}
        record = {'case': path.name, 'passed': report['passed'], 'fault': report['fault_test'],
            'dual_zero_latency_wall_s': report.get('dual_zero_latency_wall_s'),
            'leader_encoder_distance_m': report.get('encoder_distance_m', {}).get('leader'),
            'leader_gt_distance_m': report.get('gt_distance_m', {}).get('leader'),
            'payload_gt_distance_m': report.get('gt_distance_m', {}).get('cooperative_payload'),
            'follower_max_requested_vx': max(abs(r.get('follower_requested_vx', 0.)) for r in rows),
            'follower_max_output_vx': max(abs(o['vx']) for o in outputs if o['robot'] == 'follower'),
            'follower_inputs': inputs,
            'goal_publishers': [n for n, value in graph.items() if any(p[0] == '/leader/transport_goal' for p in value['publishers'])],
            'peak_follower_wrench_change_N': report.get('peak_wrench_change_force_N', {}).get('follower'),
            'no_automatic_restart': report.get('no_automatic_restart')}
        record['boundary_passed'] = all(not any(t.startswith('/leader/') or 'goal' in t or 'ground_truth' in t
            or '/world/' in t or t.endswith('/tf') or t.endswith('/tf_static') for t in topics) for topics in inputs.values())
        transitions = {}
        for robot in ('leader', 'follower'):
            history = []
            for row in rows:
                state = row.get(robot+'_state')
                if state and (not history or state != history[-1]['state']):
                    history.append({'sim_time': row['sim_time'], 'state': state})
            transitions[robot] = history
        record['state_transitions'] = transitions
        results.append(record)
        if report['fault_test'] == 'none':
            hold = [r for r in rows if r['phase'] == 'hold']
            moving = [r for r in rows if r['phase'] in ('motion', 'settle')]
            origin, start = hold[-1], moving[0]['sim_time']
            t = [r['sim_time']-start for r in moving]
            fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
            axes[0].plot(t, [(r['leader_encoder_x']-origin['leader_encoder_x'])*1e6 for r in moving], label='Leader encoder')
            axes[0].plot(t, [(r['leader_gt_x']-origin['leader_gt_x'])*1e6 for r in moving], label='Leader GT')
            axes[0].plot(t, [(r['payload_x']-origin['payload_x'])*1e6 for r in moving], label='Payload GT')
            axes[0].plot(t, [(r['follower_gt_x']-origin['follower_gt_x'])*1e6 for r in moving], label='Follower GT')
            axes[0].axhline(report['distance_m']*1e6, color='gray', linestyle='--', label='Leader relative goal')
            axes[0].set_ylabel('Displacement [um]')
            for robot in ('leader', 'follower'):
                axes[1].plot(t, [r[robot+'_output_vx']*1e6 for r in moving], label=robot)
            axes[1].set_ylabel('Base output [um/s]')
            bias = np.mean([[r['follower_'+k] for k in ('fx', 'fy', 'fz')] for r in hold], axis=0)
            force = np.array([[r['follower_'+k] for k in ('fx', 'fy', 'fz')] for r in moving])
            axes[2].plot(t, np.linalg.norm(force-bias, axis=1), label='Follower wrench change (not grip rating)')
            axes[2].set_ylabel('|dF| [N]'); axes[2].set_xlabel('Simulation time after goal send [s]')
            for axis in axes:
                axis.legend(fontsize=8); axis.grid(True, alpha=.3)
            fig.suptitle('CP-4: leader-only goal, stationary follower; no autonomous following')
            fig.tight_layout(); fig.savefig(args.output/(path.name+'.png'), dpi=150); plt.close(fig)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2))
    keys = [k for k in results[0] if k not in ('follower_inputs', 'state_transitions')]
    with (args.output/'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
        writer.writeheader(); writer.writerows(results)
    print(json.dumps(results, indent=2))
    assert all(r['passed'] and r['boundary_passed'] and r['follower_max_requested_vx'] == 0.
               and r['follower_max_output_vx'] == 0. for r in results)


if __name__ == '__main__':
    main()
