"""Never substitute receipt time or zero bridge stamps for source time."""
import copy

import pytest

from amir_rotation_source_time_capture import ROBOTS, match_odom, parse_gt, stamp_ns


def message():
    return dict(header=dict(stamp=dict(sec='7904', nsec=482000000)), pose=[
        dict(name=name, position=dict(x=i, y=0., z=0.), orientation=dict(x=0., y=0., z=0., w=1.))
        for i, name in enumerate(ROBOTS)])


def test_preserves_original_vector_source_stamp_and_all_three_poses():
    value = message()
    before = copy.deepcopy(value)
    result = parse_gt(value)
    assert result['gt_source_time_ns'] == 7904482000000
    assert set(result['poses']) == set(ROBOTS)
    assert value == before


def test_proto_json_omitted_zero_fields_keep_declared_scalar_defaults():
    value = message()
    value['pose'][0]['position'] = {}
    value['pose'][0]['orientation'] = dict(w=1.)
    assert parse_gt(value)['poses']['leader'][0][3] == 0.


@pytest.mark.parametrize('sec,nsec', [(True, 0), (-1, 0), ('1.5', 0), (1, -1), (1, 10**9)])
def test_source_stamp_must_not_be_repaired_with_receipt_clock(sec, nsec):
    with pytest.raises(ValueError):
        stamp_ns(sec, nsec)


@pytest.mark.parametrize('fault', ['zero_stamp', 'missing_stamp', 'missing_pose', 'duplicate_pose', 'invalid_quaternion'])
def test_unknown_gt_time_or_model_pose_is_rejected(fault):
    value = message()
    if fault == 'zero_stamp':
        value['header']['stamp'] = dict(sec=0, nsec=0)
    elif fault == 'missing_stamp':
        del value['header']
    elif fault == 'missing_pose':
        value['pose'].pop()
    elif fault == 'duplicate_pose':
        value['pose'].append(value['pose'][0])
    else:
        value['pose'][0]['orientation']['w'] = .5
    with pytest.raises((ValueError, KeyError)):
        parse_gt(value)


def test_source_join_uses_latest_previous_sample_with_existing_twenty_ms_bound():
    history = [dict(stamp_ns=x) for x in [1_000_000_000, 1_010_000_000, 1_030_000_000]]
    assert match_odom(history, 1_025_000_000)['stamp_ns'] == 1_010_000_000
    assert match_odom(history, 1_050_000_000)['stamp_ns'] == 1_030_000_000
    assert match_odom(history, 1_050_000_001) is None
    assert match_odom(history, 999_000_000) is None
