#!/usr/bin/env python3
"""Plot saved CP-8R-2 evidence with system matplotlib; no simulator connection."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory', type=Path, required=True)
    p.add_argument('--trials', nargs='+', required=True)
    a = p.parse_args()
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    titles = ['Object angle and leader command', 'Object centre drift (evaluation only)',
              'Follower local xy deflection', 'Follower local yaw deflection',
              'Unfiltered incremental TCP force', 'Unfiltered incremental TCP moment']
    units = ['Angle [deg]', 'Drift [mm]', 'Deflection [mm]', 'Deflection [mrad]', 'Force norm [N]', 'Moment norm [Nm]']
    for trial in a.trials:
        rows = [json.loads(line) for line in (a.audit_directory/trial/'samples.jsonl').open()]
        initial = next(r for r in reversed(rows) if r['phase'] == 'baseline')
        moving = [r for r in rows if r['phase'] in ('rotation', 'hold')]
        t = np.array([r['t_s']-moving[0]['t_s'] for r in moving])
        centre0 = np.array([initial['object_center_GT_'+k] for k in 'xyz'])
        centre = np.array([[r['object_center_GT_'+k] for k in 'xyz'] for r in moving])
        angle = np.unwrap([r['object_yaw_GT_rad'] for r in moving])-initial['object_yaw_GT_rad']
        line, = axes[0, 0].plot(t, np.degrees(angle), label=trial)
        axes[0, 0].plot(t, [math.degrees(r['phi_cmd_rad'] or 0.) for r in moving], '--', color=line.get_color(), alpha=.6)
        axes[0, 1].plot(t, 1e3*np.linalg.norm(centre-centre0, axis=1), label=trial)
        axes[1, 0].plot(t, [1e3*np.hypot(r['follower_local_error_x'], r['follower_local_error_y']) for r in moving], label=trial)
        axes[1, 1].plot(t, [1e3*r['follower_local_error_yaw'] for r in moving], label=trial)
        for start, ax in ((0, axes[2, 0]), (3, axes[2, 1])):
            valid = [r for r in moving if 'follower_raw_tared_tcp_wrench_'+str(start+2) in r]
            ax.plot([r['t_s']-moving[0]['t_s'] for r in valid],
                    [np.linalg.norm([r['follower_raw_tared_tcp_wrench_'+str(i)] for i in range(start, start+3)]) for r in valid], label=trial)
    for ax, cap in ((axes[0, 1], 1.), (axes[2, 0], .5), (axes[2, 1], .05)):
        ax.axhline(cap, color='#b92b27', linestyle=':', linewidth=1.2, label='Experiment limit')
    for ax, title, unit in zip(axes.flat, titles, units):
        ax.set(title=title, xlabel='Time since rotation start [s]', ylabel=unit)
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('CP-8R-2: leader-only command, local follower feedback, fixed grasp', fontsize=14)
    for extension in ('svg', 'png'):
        fig.savefig(a.audit_directory/('local_following.'+extension), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
