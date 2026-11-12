"""Compare actual independent base measurements without activating geometry.

Recompute from source-time CSV instead of trusting a derived comparison file.
Linear and angular errors retain separate units; no mixed-unit matrix norm is
used as a precision threshold or a transport permit.
"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from amir_rotation_base_metrology import AXES, analyze


def validate_stop(result):
    if (result.get('passed') is not True or result.get('stopped_outputs_confirmed') is not True
            or result.get('completed_cases') != 12
            or result.get('transport_qualified') is not False
            or result.get('precision_qualified') is not False
            or result.get('scope') != 'simulation-only independent ungrasped base metrology'):
        raise ValueError('complete ungrasped twelve-case stop evidence required')
    for robot in ('leader', 'follower'):
        for controller in ('arm_controller', 'mecanum_drive_controller'):
            if result.get('final_controllers', {}).get(robot, {}).get(controller) != 'inactive':
                raise ValueError('both arm/base controllers must be confirmed inactive')


def compare(baseline, candidate, baseline_result, candidate_result):
    for result in (baseline_result, candidate_result):
        validate_stop(result)
    for key in ('speeds', 'accelerations', 'hold_s', 'limits_policy_sha256', 'leader_limits_policy_sha256'):
        if key not in baseline_result or baseline_result[key] != candidate_result.get(key):
            raise ValueError('independent baseline/candidate protocol differs: '+key)
    report = dict(evaluation_only=True, applied_to_control=False, precision_qualified=False,
        transport_qualified=False, physical_center_fix_achieved=False,
        next_gate='fresh loaded static grasp calibration and passive/O2 qualification before bounded loaded revalidation',
        precision_tolerance_established=False, robots={})
    for robot in ('leader', 'follower'):
        measurements = [data['robots'][robot] for data in (baseline, candidate)]
        cases = []
        matrices = []
        for data in measurements:
            matrix = np.asarray(data['physics_from_encoder_matrix'], dtype=float)
            if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or data.get('normalized_design_rank') != 3:
                raise ValueError('finite full-rank independent metrology required')
            keys = [(case['axis'], case['sign']) for case in data['cases']]
            if len(keys) != 6 or set(keys) != {(axis, sign) for axis in AXES for sign in (-1, 1)}:
                raise ValueError('each robot needs unique positive/negative x/y/yaw cases')
            matrices.append(matrix)
        maps = [{(case['axis'], case['sign']): case for case in data['cases']} for data in measurements]
        for axis in AXES:
            for sign in (-1, 1):
                row = dict(axis=axis, sign=sign)
                for label, data in zip(('baseline', 'candidate'), maps):
                    value = data[axis, sign]
                    encoder = np.asarray(value['encoder']['velocity'], dtype=float)
                    physical = np.asarray(value['gt']['velocity'], dtype=float)
                    if (encoder.shape != (3,) or physical.shape != (3,)
                            or not np.isfinite(encoder).all() or not np.isfinite(physical).all()):
                        raise ValueError('finite measured velocity vectors required')
                    index = AXES.index(axis)
                    if encoder[index]*sign <= 0. or physical[index]*sign <= 0.:
                        raise ValueError('positive measured motion in the specified direction required')
                    error = physical-encoder
                    row[label+'_xy_velocity_error_m_s'] = float(np.linalg.norm(error[:2]))
                    row[label+'_yaw_velocity_error_rad_s'] = float(abs(error[2]))
                    row[label+'_excited_axis_scale_error'] = float(physical[index]/encoder[index]-1.)
                for metric in ('xy_velocity_error_m_s', 'yaw_velocity_error_rad_s'):
                    row['candidate_minus_baseline_'+metric] = row['candidate_'+metric]-row['baseline_'+metric]
                cases.append(row)
        report['robots'][robot] = dict(cases=cases,
            baseline_diagonal_scale_error=(np.diag(matrices[0])-1.).tolist(),
            candidate_diagonal_scale_error=(np.diag(matrices[1])-1.).tolist(),
            baseline_positive_negative_matrix_difference=measurements[0]['maximum_positive_negative_matrix_difference'],
            candidate_positive_negative_matrix_difference=measurements[1]['maximum_positive_negative_matrix_difference'])
    return report


def review(baseline_run, candidate_run, output):
    paths = [Path(p).resolve() for p in (baseline_run, candidate_run)]
    output = Path(output).resolve()
    if any(output == path or path in output.parents for path in paths):
        raise ValueError('review must not write inside measured runs')
    reports, results, hashes = [], [], {}
    for run in paths:
        result_file = run/'result.json'
        result = json.loads(result_file.read_text())
        validate_stop(result)
        with (run/'samples.csv').open() as stream:
            recomputed = analyze(list(csv.DictReader(stream)))
        stored = json.loads((run/'metrology.json').read_text())
        if recomputed != stored:
            raise ValueError('stored metrology differs from source CSV recomputation')
        reports.append(recomputed)
        results.append(result)
        hashes[str(result_file)] = hashlib.sha256(result_file.read_bytes()).hexdigest()
        hashes[str(run/'samples.csv')] = hashlib.sha256((run/'samples.csv').read_bytes()).hexdigest()
    report = compare(*reports, *results)
    report['evidence_sha256'] = hashes
    output.mkdir(parents=True, exist_ok=False)
    (output/'review.json').write_text(json.dumps(report, indent=2)+'\n')
    rows = [dict(robot=robot, **row) for robot, data in report['robots'].items() for row in data['cases']]
    with (output/'comparison.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return report
