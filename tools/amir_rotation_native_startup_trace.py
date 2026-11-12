"""Own-source diagnostic recording; never filters or publishes control data."""
import csv
import json
import math
import time
from collections import Counter


class NativeStartupTrace:
    def __init__(self, output, phase):
        self.output, self.phase = output, phase
        self.stream = (output/'native_startup_events.jsonl').open('x', buffering=1)
        self.latest, self.counts = {}, Counter()

    def active(self):
        return (self.phase[0] in ('native_static_hold', 'tare')
                or self.phase[0].startswith(('native_passive_', 'native_o2_', 'native_follow_')))

    def mark(self, kind, **fields):
        self.stream.write(json.dumps(dict(kind=kind, phase=self.phase[0],
            wall_monotonic_s=time.monotonic(),wall_epoch_s=time.time(),**fields), allow_nan=False)+'\n')
        self.counts[kind] += 1

    def wrench(self, kind, stamp, frame, vector, now):
        if kind not in ('raw', 'filtered') or frame != 'follower/grasp_aligned':
            raise ValueError('native trace requires actual own grasp-aligned wrench')
        if len(vector) != 6 or not all(math.isfinite(v) for v in [stamp, now, *vector]):
            raise ValueError('native trace requires finite source time and six-axis wrench')
        self.latest[kind] = dict(source_stamp=stamp, received_sim_s=now, wrench=list(vector))
        if self.active():
            self.mark(kind, **self.latest[kind])

    def pair_after_tare(self, tare_stamp, now):
        if not math.isfinite(tare_stamp) or not math.isfinite(now) or set(self.latest) != {'raw', 'filtered'}:
            return False
        stamps = [self.latest[k]['source_stamp'] for k in ('raw', 'filtered')]
        return (all(t > tare_stamp and -.02-1e-12 <= now-t <= .05+1e-12 for t in stamps)
                and max(stamps)-min(stamps) <= .02+1e-12)

    def command(self, names, positions, stamp, now, measured_names, measured_positions, measured_stamp):
        if not self.active():
            return
        actual = dict(zip(measured_names, measured_positions))
        if (len(names) != 5 or len(set(names)) != 5 or len(positions) != 5
                or any(n not in actual for n in names)):
            raise ValueError('native trace requires five named own arm joints')
        measured = [actual[n] for n in names]
        if not all(math.isfinite(v) for v in [stamp, now, measured_stamp, *positions, *measured]):
            raise ValueError('native trace requires finite own joint command and measurement')
        self.mark('arm_command', source_stamp=stamp, received_sim_s=now,
                  measured_stamp=measured_stamp, joint_names=list(names), requested=list(positions),
                  measured=measured, error=[a-b for a, b in zip(positions, measured)])

    def close(self):
        self.stream.close()
        keys = ['kind', 'phase', 'source_stamp', 'received_sim_s', 'measured_stamp',
                'wall_monotonic_s', 'wall_epoch_s',
                'wrench', 'joint_names', 'requested', 'measured', 'error', 'tare_completed_s', 'fault']
        with (self.output/'native_startup_events.jsonl').open() as source, (self.output/'native_startup_events.csv').open('w', newline='') as destination:
            writer = csv.DictWriter(destination, fieldnames=keys)
            writer.writeheader()
            for line in source:
                row = json.loads(line)
                writer.writerow({k: json.dumps(v) if isinstance(v, (list, dict)) else v for k, v in row.items()})
        return dict(self.counts)
