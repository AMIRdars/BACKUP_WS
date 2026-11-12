"""Evaluation-only vector decomposition of recorded object-centre drift.

No ROS imports, command publishers, correction parameters, or qualification.
Legacy GT records carry the observer time at receipt, not the GT source time.
The join bound below concerns recorded times only and cannot certify freshness.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


TERMS = ('planned_motion', 'odom_tracking', 'physics_vs_odom', 'attachment_change')


def transform(value):
    result = np.asarray(value, dtype=float)
    if (result.shape != (4, 4) or not np.isfinite(result).all()
            or not np.allclose(result[3], [0., 0., 0., 1.], atol=1e-10, rtol=0.)
            or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-6, rtol=0.)
            or abs(np.linalg.det(result[:3, :3])-1.) > 1e-6):
        raise ValueError('finite rigid homogeneous transform required')
    return result.copy()


def own_pose(debug):
    desired = transform(debug['base_desired_odom'])
    if not np.allclose(desired[:3, 2], [0., 0., 1.], atol=1e-9, rtol=0.):
        raise ValueError('planar desired base rotation required')
    error = np.asarray(debug['base_error_xy_m'], dtype=float)
    yaw_error = float(debug['yaw_error_rad'])
    if error.shape != (2,) or not np.isfinite(error).all() or not math.isfinite(yaw_error):
        raise ValueError('finite recorded planar tracking error required')
    # The leader records desired minus measured, in its odom frame.
    yaw = math.atan2(desired[1, 0], desired[0, 0])-yaw_error
    c, s = math.cos(yaw), math.sin(yaw)
    own = desired.copy()
    own[:2, 3] -= error
    own[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    return own


def decompose(initial_base, initial_object, initial_own, debug, physical_base, physical_object):
    """Exact telescoping at the supplied poses; norms of terms are not additive."""
    base0, obj0 = transform(initial_base), transform(initial_object)
    world_from_odom = base0 @ np.linalg.inv(transform(initial_own))
    attachment0 = np.linalg.inv(base0) @ obj0
    planned = world_from_odom @ transform(debug['base_desired_odom']) @ attachment0
    encoded = world_from_odom @ own_pose(debug) @ attachment0
    rigid = transform(physical_base) @ attachment0
    actual = transform(physical_object)
    vectors = dict(planned_motion=planned[:3, 3]-obj0[:3, 3],
                   odom_tracking=encoded[:3, 3]-planned[:3, 3],
                   physics_vs_odom=rigid[:3, 3]-encoded[:3, 3],
                   attachment_change=actual[:3, 3]-rigid[:3, 3])
    drift = actual[:3, 3]-obj0[:3, 3]
    norm = float(np.linalg.norm(drift))
    result = dict(object_center_drift_m=norm)
    for axis, value in zip('xyz', drift):
        result['object_d'+axis+'_m'] = float(value)
    for term, vector in vectors.items():
        for axis, value in zip('xyz', vector):
            result[term+'_d'+axis+'_m'] = float(value)
        result[term+'_norm_m'] = float(np.linalg.norm(vector))
        result[term+'_signed_projection_m'] = float(vector @ drift/norm) if norm else 0.
    result['vector_closure_residual_m'] = float(np.linalg.norm(sum(vectors.values())-drift))
    result['signed_projection_closure_residual_m'] = abs(sum(
        result[term+'_signed_projection_m'] for term in TERMS)-norm)
    return result


def records(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


class RecordedTimeJoin:
    def __init__(self, iterator):
        self.iterator = iter(iterator)
        self.before = None
        self.after = next(self.iterator, None)
        self.last_requested = -math.inf

    def at(self, timestamp):
        if not math.isfinite(timestamp) or timestamp < self.last_requested:
            raise ValueError('evaluation record times must be finite and nondecreasing')
        self.last_requested = timestamp
        while self.after is not None:
            stamp = float(self.after['sim_time'])
            if not math.isfinite(stamp) or (self.before is not None and stamp < float(self.before['sim_time'])):
                raise ValueError('leader record times must be finite and nondecreasing')
            if stamp > timestamp:
                break
            self.before = self.after
            self.after = next(self.iterator, None)
        return self.before


def analyze(run, output, stride=400):
    """Read completed fixed-grasp artifacts; write only to a fresh output folder."""
    run, output = Path(run).resolve(), Path(output).resolve()
    if stride < 1:
        raise ValueError('positive sampling stride required')
    if output == run or run in output.parents:
        raise ValueError('analysis output must be separate from the measured run')
    verdict_file = run/'comparison_result.json'
    verdict = json.loads(verdict_file.read_text())
    if any(verdict.get(k) is not True for k in ('passed', 'positive_negative_programs_completed',
            'stop_and_hold_confirmed', 'both_fixed_grasps_retained')) or verdict.get('completed_goals') != 2 or verdict.get('grasp_losses') != []:
        raise ValueError('completed positive/negative fixed-grasp stop/hold evidence required')
    truth_file, debug_file = run/'evaluation_only.jsonl', run/'leader_debug.jsonl'
    baseline = None
    for row in records(truth_file):
        if row['phase'] != 'baseline':
            break
        baseline = row
    if baseline is None:
        raise ValueError('recorded pre-motion baseline required')
    first = next(records(debug_file))
    if abs(float(first['phi_rad'])) > 1e-9:
        raise ValueError('initial leader debug must precede rotation')
    initial_time_gap = float(first['sim_time'])-float(baseline['timestamp'])
    if not 0. <= initial_time_gap <= .05+1e-9:
        raise ValueError('initial recorded-time join exceeds 50 ms diagnostic bound')
    base0, obj0 = transform(baseline['leader_base_pose_GT']), transform(baseline['object_pose_GT'])
    own0 = own_pose(first)
    alignment = base0 @ np.linalg.inv(own0)
    center = np.asarray(first['center_odom_xyz'], dtype=float)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError('finite initial latched centre required')
    planned_center = (alignment @ np.r_[center, 1.])[:3]
    fields = ['object_center_drift_m', 'vector_closure_residual_m',
              'signed_projection_closure_residual_m']+[term+'_norm_m' for term in TERMS]
    maxima = dict.fromkeys(fields, 0.)
    phase_maxima, endpoints, peak = {}, {}, None
    count, skipped, eligible = 0, 0, 0
    join = RecordedTimeJoin(records(debug_file))
    output.mkdir(parents=True, exist_ok=False)
    csv_path = output/'attribution.csv'
    max_join_gap = 0.
    with csv_path.open('x', newline='') as stream:
        writer = None
        for index, truth in enumerate(records(truth_file)):
            if truth['phase'] not in ('positive', 'negative', 'hold'):
                continue
            eligible += 1
            if index % stride:
                continue
            t = float(truth['timestamp'])
            debug = join.at(t)
            if debug is None or not 0. <= t-float(debug['sim_time']) <= .05+1e-9:
                skipped += 1
                continue
            gap = t-float(debug['sim_time'])
            value = dict(recorded_time_s=t, phase=truth['phase'], leader_recorded_time_gap_s=gap,
                **decompose(base0, obj0, own0, debug, truth['leader_base_pose_GT'], truth['object_pose_GT']))
            if writer is None:
                writer = csv.DictWriter(stream, fieldnames=list(value))
                writer.writeheader()
            writer.writerow(value)
            count += 1
            max_join_gap = max(max_join_gap, gap)
            for key in fields:
                maxima[key] = max(maxima[key], value[key])
            phase = phase_maxima.setdefault(truth['phase'], dict.fromkeys(fields, 0.))
            for key in fields:
                phase[key] = max(phase[key], value[key])
            endpoints[truth['phase']] = value
            if peak is None or value['object_center_drift_m'] > peak['object_center_drift_m']:
                peak = value
    if not all(phase in endpoints for phase in ('positive', 'negative')):
        raise ValueError('joined diagnostic records required in both directions')
    calibration_file = run/'leader_fixed_grasp_calibration.json'
    calibration = json.loads(calibration_file.read_text()) if calibration_file.exists() else {}
    report = dict(run=str(run), evaluation_only=True, used_by_control=False,
        precision_qualified=False, physical_center_fix_achieved=False,
        sampling_stride=stride, joined_samples=count, eligible_gt_records=eligible,
        skipped_sampled_records=skipped, source_timestamp_freshness_verified=False,
        timestamp_limitation='legacy GT timestamp is observer time at receipt; GT source time was not recorded',
        maximum_recorded_time_gap_s=max_join_gap, initial_recorded_time_gap_s=initial_time_gap,
        initial_object_center_world_xyz_m=obj0[:3, 3].tolist(),
        initial_planned_center_world_xyz_m=planned_center.tolist(),
        initial_center_offset_xyz_m=(planned_center-obj0[:3, 3]).tolist(),
        initial_center_offset_norm_m=float(np.linalg.norm(planned_center-obj0[:3, 3])),
        leader_grasp_calibration_snapshot_sim_s=calibration.get('snapshot_sim_s'),
        leader_grasp_calibration_transferred_from=calibration.get('transferred_from'),
        maxima=maxima, phase_maxima=phase_maxima, sampled_phase_last_records=endpoints,
        sampled_maximum_drift_record=peak,
        interpretation={
            'planned_motion': 'desired base applied to initial physical attachment; includes initial-centre error and any later relatch',
            'odom_tracking': 'recorded own base minus desired base applied to the same initial attachment',
            'physics_vs_odom': 'physical base minus recorded own base applied to the same initial attachment',
            'attachment_change': 'actual object minus its initial attachment propagated by physical base; includes arm motion, compliance and record alignment',
            'sum': 'vector terms and signed projections add; individual maxima and norms do not'},
        completed_result_sha256=hashlib.sha256(verdict_file.read_bytes()).hexdigest())
    (output/'attribution.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stride', type=int, default=400)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run, args.output, args.stride), indent=2))


if __name__ == '__main__':
    main()
