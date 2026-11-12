"""Independent-motion evidence must identify all axes with real source times."""
import copy
import math

import numpy as np
import pytest

from amir_rotation_base_metrology import AXES, analyze, body_log, fit_velocity, calibration_limits


def exp_pose(velocity, t, initial=(3., -2., 3.13)):
    vx, vy, omega = velocity
    angle = omega*t
    if abs(omega) > 1e-12:
        a, b = math.sin(angle)/omega, (1.-math.cos(angle))/omega
        p = np.array([a*vx-b*vy, b*vx+a*vy])
    else:
        p = np.array([vx*t, vy*t])
    c, s = math.cos(initial[2]), math.sin(initial[2])
    p = np.array([[c, -s], [s, c]])@p+initial[:2]
    yaw = math.atan2(math.sin(initial[2]+angle), math.cos(initial[2]+angle))
    return [*p, yaw]


def evidence(matrix=None):
    matrix = np.array(matrix if matrix is not None else [[1.01, .02, .003], [-.01, .99, .002], [.001, -.002, 1.018]])
    rows = []
    for robot in ('leader', 'follower'):
        for i, axis in enumerate(AXES):
            for sign in (-1, 1):
                velocity = np.eye(3)[i]*sign*.002
                for index in range(151):
                    t = index*.02
                    # Different source times, known yaw wrap, and duplicate callbacks.
                    row = dict(robot=robot, axis=axis, sign=sign, phase='plateau',
                        sim_time_s=10.+t+.01, encoder_stamp_s=10.+t, gt_stamp_s=10.+t-.005)
                    for source, v, time in (('encoder', velocity, t), ('gt', matrix@velocity, t-.005)):
                        row.update({source+'_'+k: value for k, value in zip(AXES, exp_pose(v, time))})
                    rows += [row, dict(row)]
    return rows, matrix


def test_recovers_full_matrix_from_own_source_time_not_arrival_duplicates():
    rows, matrix = evidence()
    result = analyze(rows)
    assert result['evaluation_only'] and not result['applied_to_control']
    assert not result['transport_qualified'] and not result['precision_qualified']
    for robot in result['robots'].values():
        np.testing.assert_allclose(robot['physics_from_encoder_matrix'], matrix, atol=1e-10)
        assert robot['normalized_design_rank'] == 3
        assert robot['cases'][0]['encoder']['source_samples'] == 151


def test_se2_log_handles_curved_translation():
    velocity = [.01, -.02, .03]
    initial = [0., 0., 0.]
    np.testing.assert_allclose(body_log(initial, exp_pose(velocity, 2., initial)), np.array(velocity)*2., atol=1e-12)


def test_calibration_uses_smaller_actual_caps_without_raising_either_policy():
    follower = dict(base_velocity_limits=[.002]*3, base_acceleration_limits=[.001, .001, .002])
    leader = dict(rotation_max_base_speed=.001, rotation_max_angular_speed=.00046,
                  rotation_max_base_acceleration=.0005, rotation_max_angular_acceleration=.0000048)
    before = copy.deepcopy((follower, leader))
    speed, acceleration = calibration_limits(follower, leader)
    np.testing.assert_allclose(speed, [.001, .001, .00046])
    np.testing.assert_allclose(acceleration, [.0005, .0005, .0000048])
    assert (follower, leader) == before


@pytest.mark.parametrize('bad', [[0., .002, .002], [.002], [float('nan')]*3])
def test_missing_or_invalid_actual_limits_cannot_produce_motion_plan(bad):
    follower = dict(base_velocity_limits=bad, base_acceleration_limits=[.001]*3)
    leader = dict(rotation_max_base_speed=.002, rotation_max_angular_speed=.00046,
                  rotation_max_base_acceleration=.001, rotation_max_angular_acceleration=.0000048)
    with pytest.raises(ValueError):
        calibration_limits(follower, leader)


@pytest.mark.parametrize('failure', ['missing_negative', 'stale', 'skew', 'gap', 'nan', 'duplicate_change', 'wrong_sign', 'arc_only'])
def test_refuses_invalid_evidence(failure):
    rows, _ = evidence()
    if failure == 'missing_negative':
        rows = [r for r in rows if r['sign'] == 1]
    elif failure == 'stale':
        rows[0]['sim_time_s'] += .1
    elif failure == 'skew':
        rows[0]['gt_stamp_s'] -= .03
        rows[0]['sim_time_s'] -= .005
    elif failure == 'gap':
        rows = [r for r in rows if not 11. < r['encoder_stamp_s'] < 11.2]
    elif failure == 'nan':
        rows[0]['gt_x'] = float('nan')
    elif failure == 'duplicate_change':
        rows[0]['gt_x'] += .0001
    elif failure == 'wrong_sign':
        for row in rows:
            if row['axis'] == 'x':
                row['sign'] *= -1
    elif failure == 'arc_only':
        for row in rows:
            # All axes are aliases for the same nonzero body excitation.
            t = row['encoder_stamp_s']-10.
            v = np.ones(3)*row['sign']*.002
            for source in ('encoder', 'gt'):
                row.update({source+'_'+k: value for k, value in zip(AXES, exp_pose(v, t))})
    with pytest.raises(ValueError):
        analyze(rows)


def test_steady_fit_does_not_invent_unobserved_interval():
    rows, _ = evidence()
    case = [r for r in rows if r['robot'] == 'leader' and r['axis'] == 'x' and r['sign'] == 1]
    for row in case:
        row['sim_time_s'] += 100.
    fit = fit_velocity(case, 'encoder')
    assert fit['duration_s'] == pytest.approx(3.)
    assert fit['source_samples'] == 151


def test_reports_direction_dependence_without_issuing_precision_qualification():
    rows, matrix = evidence()
    for row in rows:
        if row['sign'] == -1:
            index = AXES.index(row['axis'])
            v = matrix@(np.eye(3)[index]*-.002)*.95
            row.update({ 'gt_'+k: value for k, value in zip(AXES, exp_pose(v, row['gt_stamp_s']-10.))})
    result = analyze(rows)
    assert result['robots']['leader']['maximum_positive_negative_matrix_difference'] > .04
    assert not result['precision_qualified']
