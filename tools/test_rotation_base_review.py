"""Independent metrology does not silently become transport qualification."""
import copy

import numpy as np
import pytest

from amir_rotation_base_metrology import analyze
from amir_rotation_base_review import compare, validate_stop
from test_rotation_base_metrology import evidence


def result():
    return dict(passed=True, stopped_outputs_confirmed=True, completed_cases=12,
        transport_qualified=False, precision_qualified=False,
        scope='simulation-only independent ungrasped base metrology',
        final_controllers={r: {c: 'inactive' for c in ('arm_controller', 'mecanum_drive_controller')}
                           for r in ('leader', 'follower')},
        speeds=[.002, .002, .00046], accelerations=[.001, .001, .0000048], hold_s=10.,
        limits_policy_sha256='actual-follower-policy', leader_limits_policy_sha256='actual-leader-policy')


@pytest.fixture
def reports():
    return [analyze(evidence(matrix)[0]) for matrix in (np.diag([1., 1., 1.02]), np.diag([1., 1., .999]))]


def test_reports_measured_candidate_effect_in_separate_units_without_permission(reports):
    before = copy.deepcopy(reports)
    report = compare(*reports, result(), result())
    for robot in report['robots'].values():
        for case in robot['cases']:
            if case['axis'] == 'yaw':
                assert case['candidate_minus_baseline_yaw_velocity_error_rad_s'] < 0.
                assert case['baseline_excited_axis_scale_error'] == pytest.approx(.02)
                assert case['candidate_excited_axis_scale_error'] == pytest.approx(-.001)
    for key in ('applied_to_control', 'precision_qualified', 'transport_qualified',
                'physical_center_fix_achieved', 'precision_tolerance_established'):
        assert report[key] is False
    assert report['evaluation_only'] is True
    assert reports == before


@pytest.mark.parametrize('fault', ['incomplete', 'active', 'unsafe', 'wrong_scope', 'fake_qualification'])
def test_refuses_missing_actual_stopping_evidence(fault):
    value = result()
    if fault == 'incomplete':
        value['completed_cases'] = 11
    elif fault == 'active':
        value['final_controllers']['follower']['arm_controller'] = 'active'
    elif fault == 'unsafe':
        value['stopped_outputs_confirmed'] = False
    elif fault == 'wrong_scope':
        value['scope'] = 'loaded transport'
    else:
        value['transport_qualified'] = True
    with pytest.raises(ValueError):
        validate_stop(value)


@pytest.mark.parametrize('key,value', [('speeds', [.01]*3), ('hold_s', 11.),
    ('limits_policy_sha256', 'different'), ('leader_limits_policy_sha256', 'different')])
def test_different_protocol_is_not_a_controlled_geometry_comparison(reports, key, value):
    other = result()
    other[key] = value
    with pytest.raises(ValueError, match='protocol differs'):
        compare(*reports, result(), other)


@pytest.mark.parametrize('fault', ['missing_negative', 'duplicate', 'rank', 'nonfinite', 'wrong_sign'])
def test_refuses_incomplete_or_invalid_directional_measurements(reports, fault):
    candidate = reports[1]['robots']['follower']
    if fault == 'missing_negative':
        candidate['cases'] = [c for c in candidate['cases'] if c['sign'] == 1]
    elif fault == 'duplicate':
        candidate['cases'][-1] = copy.deepcopy(candidate['cases'][0])
    elif fault == 'rank':
        candidate['normalized_design_rank'] = 1
    elif fault == 'nonfinite':
        candidate['physics_from_encoder_matrix'][0][0] = float('nan')
    elif fault == 'wrong_sign':
        candidate['cases'][0]['encoder']['velocity'][0] *= -1
    with pytest.raises(ValueError):
        compare(*reports, result(), result())
