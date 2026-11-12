"""Vector attribution, frame invariance and legacy-time limitations."""
import copy
import json
import math

import numpy as np
import pytest

from amir_rotation_center_attribution import (
    TERMS, RecordedTimeJoin, analyze, decompose, own_pose, transform,
)


def pose(x=0., y=0., z=0., yaw=0.):
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    result[:3, 3] = [x, y, z]
    return result


def debug(desired=None, error=(0., 0.), yaw_error=0., stamp=.02, phi=0.):
    return dict(base_desired_odom=(np.eye(4) if desired is None else desired).tolist(),
        base_error_xy_m=list(error), yaw_error_rad=yaw_error, sim_time=stamp,
        phi_rad=phi, center_odom_xyz=[1., 0., .6])


def vector(result, term):
    return np.array([result[term+'_d'+axis+'_m'] for axis in 'xyz'])


def test_exact_center_rotation_has_no_drift_with_arbitrary_odom_origin():
    base0, obj0, own0 = pose(3., -2., yaw=.7), pose(4., -2., .6), pose(-6., 5., yaw=3.13)
    center_in_base = np.linalg.inv(base0) @ obj0
    for angle in [.4, -.8]:
        world_base = pose(4., -2.) @ pose(yaw=angle) @ pose(-1., 0., yaw=.7)
        desired = own0 @ np.linalg.inv(base0) @ world_base
        actual = world_base @ center_in_base
        result = decompose(base0, obj0, own0, debug(desired), world_base, actual)
        assert result['object_center_drift_m'] < 1e-14
        for term in TERMS:
            assert result[term+'_norm_m'] < 1e-14


def test_isolates_wrong_initial_center_without_blame_on_tracking_or_grasp():
    actual_center = np.array([1., 0., .6])
    wrong_center = np.array([1.002, -.001, .6])
    rotation = pose(yaw=.5)
    initial_obj = pose(*actual_center)
    desired = pose(*wrong_center) @ rotation @ pose(*(-wrong_center))
    actual_obj = desired @ initial_obj
    result = decompose(np.eye(4), initial_obj, np.eye(4), debug(desired), desired, actual_obj)
    expected = (np.eye(3)-rotation[:3, :3]) @ (wrong_center-actual_center)
    np.testing.assert_allclose(vector(result, 'planned_motion'), expected, atol=1e-15)
    assert result['object_center_drift_m'] > 0.
    for term in TERMS[1:]:
        assert result[term+'_norm_m'] < 1e-14


def test_vectors_and_signed_projections_close_with_canceling_terms():
    # Norms would add to .6, but actual drift is only .2.
    desired, physical, actual = pose(.1), pose(.3), pose(.2, z=.6)
    result = decompose(np.eye(4), pose(z=.6), np.eye(4),
        debug(desired, error=(.1, 0.)), physical, actual)
    np.testing.assert_allclose([result[t+'_signed_projection_m'] for t in TERMS], [.1, -.1, .3, -.1])
    assert result['object_center_drift_m'] == pytest.approx(.2)
    assert sum(result[t+'_norm_m'] for t in TERMS) == pytest.approx(.6)
    assert result['vector_closure_residual_m'] < 1e-14
    assert result['signed_projection_closure_residual_m'] < 1e-14


def test_yaw_wrap_sign_and_input_immutability():
    value = debug(pose(2., 3., yaw=-math.pi+.01), error=(.3, -.2), yaw_error=.02)
    before = copy.deepcopy(value)
    own = own_pose(value)
    np.testing.assert_allclose(own[:2, 3], [1.7, 3.2])
    np.testing.assert_allclose(own[:3, :3], pose(yaw=math.pi-.01)[:3, :3])
    assert value == before


@pytest.mark.parametrize('bad', [np.zeros((4, 4)), np.eye(3), np.eye(4)*2.,
                               np.diag([-1., 1., 1., 1.]), np.full((4, 4), math.nan)])
def test_refuses_nonrigid_or_nonfinite_transform(bad):
    with pytest.raises(ValueError):
        transform(bad)


def test_refuses_nonplanar_desired_and_nonfinite_tracking_error():
    tilted = np.eye(4)
    tilted[:3, :3] = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]
    with pytest.raises(ValueError, match='planar'):
        own_pose(debug(tilted))
    with pytest.raises(ValueError, match='finite'):
        own_pose(debug(error=(math.nan, 0.)))


def test_join_never_uses_future_leader_record_and_rejects_time_reversal():
    join = RecordedTimeJoin([debug(stamp=1.), debug(stamp=1.02)])
    assert join.at(.99) is None
    assert join.at(1.019)['sim_time'] == 1.
    assert join.at(1.02)['sim_time'] == 1.02
    with pytest.raises(ValueError, match='nondecreasing'):
        join.at(1.01)
    with pytest.raises(ValueError, match='leader record'):
        RecordedTimeJoin([debug(stamp=1.), debug(stamp=.9)]).at(2.)


def recorded_run(tmp_path):
    run = tmp_path/'measured'
    run.mkdir()
    verdict = dict(passed=True, positive_negative_programs_completed=True,
        stop_and_hold_confirmed=True, both_fixed_grasps_retained=True,
        completed_goals=2, grasp_losses=[])
    (run/'comparison_result.json').write_text(json.dumps(verdict))
    obj0 = pose(1., 0., .6)
    truth = [dict(timestamp=.0, phase='baseline', leader_base_pose_GT=np.eye(4).tolist(),
                  object_pose_GT=obj0.tolist())]
    motion = [debug(stamp=.02)]
    for stamp, phase, x in [(.04, 'positive', .01), (.08, 'negative', -.02)]:
        physical = pose(x)
        truth.append(dict(timestamp=stamp, phase=phase, leader_base_pose_GT=physical.tolist(),
                          object_pose_GT=(physical@obj0).tolist()))
        motion.append(debug(stamp=stamp-.002))
    for file, values in [('evaluation_only.jsonl', truth), ('leader_debug.jsonl', motion)]:
        (run/file).write_text(''.join(json.dumps(x)+'\n' for x in values))
    return run


def test_completed_artifacts_write_separate_timeseries_with_explicit_time_limitation(tmp_path):
    run = recorded_run(tmp_path)
    original = {p.name: p.read_bytes() for p in run.iterdir()}
    result = analyze(run, tmp_path/'diagnosis', stride=1)
    assert result['joined_samples'] == 2
    assert result['maxima']['physics_vs_odom_norm_m'] == pytest.approx(.02)
    assert not result['source_timestamp_freshness_verified']
    assert not result['physical_center_fix_achieved'] and not result['precision_qualified']
    assert not result['used_by_control']
    assert (tmp_path/'diagnosis/attribution.csv').is_file()
    assert {p.name: p.read_bytes() for p in run.iterdir()} == original
    with pytest.raises(FileExistsError):
        analyze(run, tmp_path/'diagnosis', stride=1)


@pytest.mark.parametrize('fault', ['failed', 'loss', 'unfinished', 'missing_negative', 'initial_gap'])
def test_incomplete_or_unsafe_proof_cannot_be_reported_as_complete(tmp_path, fault):
    run = recorded_run(tmp_path)
    path = run/'comparison_result.json'
    verdict = json.loads(path.read_text())
    if fault == 'failed':
        verdict['passed'] = False
    elif fault == 'loss':
        verdict['grasp_losses'] = [{}]
    elif fault == 'unfinished':
        verdict['completed_goals'] = 1
    elif fault == 'missing_negative':
        p = run/'evaluation_only.jsonl'
        p.write_text('\n'.join(p.read_text().splitlines()[:-1])+'\n')
    elif fault == 'initial_gap':
        p = run/'leader_debug.jsonl'
        values = [json.loads(x) for x in p.read_text().splitlines()]
        values[0]['sim_time'] = .1
        p.write_text(''.join(json.dumps(x)+'\n' for x in values))
    path.write_text(json.dumps(verdict))
    with pytest.raises(ValueError):
        analyze(run, tmp_path/'invalid', stride=1)
    assert not (tmp_path/'invalid/attribution.json').exists()


def test_cannot_write_analysis_inside_measured_run(tmp_path):
    run = recorded_run(tmp_path)
    with pytest.raises(ValueError, match='separate'):
        analyze(run, run/'analysis', stride=1)
