"""Evaluation-only, lossless-at-receipt local F/T and controller traces."""
import csv
import json


class LocalControlTrace:
    def __init__(self, folder):
        self.wrench_file = (folder / 'wrench_samples.csv').open('w', buffering=1)
        self.wrench_writer = csv.DictWriter(self.wrench_file, fieldnames=[
            'kind', 'robot', 'phase', 'source_stamp_s', 'receipt_sim_time_s',
            'receipt_wall_time_s', 'frame_id', 'admittance_enabled',
            *['wrench_' + str(i) for i in range(6)]])
        self.wrench_writer.writeheader()
        self.control_file = (folder / 'control_samples.jsonl').open('w', buffering=1)
        self.control_rows = []
        self.folder = folder

    def wrench(self, *, kind, robot, phase, source_stamp_s, receipt_sim_time_s,
               receipt_wall_time_s, frame_id, admittance_enabled, values):
        if len(values) != 6:
            raise ValueError('six wrench components required')
        self.wrench_writer.writerow(dict(kind=kind, robot=robot, phase=phase,
            source_stamp_s=source_stamp_s, receipt_sim_time_s=receipt_sim_time_s,
            receipt_wall_time_s=receipt_wall_time_s, frame_id=frame_id,
            admittance_enabled=admittance_enabled,
            **{'wrench_' + str(i): x for i, x in enumerate(values)}))

    def control(self, value, *, phase, receipt_sim_time_s, receipt_wall_time_s):
        # Keep source and receipt times separate. Preserve SAFE_STOP messages,
        # whose wrench_at_fault may contain a peak absent from the 50 Hz CSV.
        item = dict(phase=phase, receipt_sim_time_s=receipt_sim_time_s,
                    receipt_wall_time_s=receipt_wall_time_s, value=value)
        self.control_file.write(json.dumps(item) + '\n')
        row = {k: v for k, v in item.items() if k != 'value'}
        for key, v in value.items():
            if isinstance(v, (list, tuple)):
                row.update({key + '_' + str(i): x for i, x in enumerate(v)})
            else:
                row[key] = v
        self.control_rows.append(row)

    def close(self):
        if self.control_rows:
            fields = sorted(set().union(*(r.keys() for r in self.control_rows)))
            with (self.folder / 'control_samples.csv').open('w') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                writer.writerows(self.control_rows)
        self.wrench_file.close()
        self.control_file.close()
