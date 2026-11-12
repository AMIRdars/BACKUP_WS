"""Callback interleaving must not masquerade as source skew or hide staleness."""
import math

import pytest

from amir_rotation_base_alignment import select_source_pair


def test_reproduces_latest_callback_skew_with_valid_actual_history():
    own = [(77.89, 'encoder_previous'), (77.894, 'encoder_latest')]
    gt = [(77.852, 'gt_previous'), (77.872, 'gt_latest')]
    assert own[-1][0]-gt[-1][0] > .02
    pair = select_source_pair(own, gt, 77.896)
    assert pair['encoder'] == own[0]
    assert pair['gt'] == gt[1]
    assert abs(pair['encoder'][0]-pair['gt'][0]) <= .02


def test_prefers_newest_common_time_and_then_closest_original_sample():
    old, near, future = object(), object(), object()
    pair = select_source_pair([(1., old), (1.012, near), (1.03, future)],
                              [(1., object()), (1.014, object())], 1.03)
    assert pair['encoder'][1] is near
    assert pair['encoder'][0] == 1.012
    assert pair['gt'][0] == 1.014


@pytest.mark.parametrize('own,gt,now', [
    ([(1., 'a')], [(1.03, 'b')], 1.03),
    ([(1., 'a')], [(1., 'b')], 1.050001),
    ([(1.020001, 'a')], [(1.020001, 'b')], 1.),
    ([], [(1., 'b')], 1.),
    ([(1., 'a')], [], 1.),
    ([(math.nan, 'a')], [(1., 'b')], 1.),
    ([(1., 'a')], [(math.inf, 'b')], 1.),
])
def test_preserves_existing_skew_age_future_and_missing_source_rejection(own, gt, now):
    with pytest.raises(ValueError):
        select_source_pair(own, gt, now)


def test_does_not_interpolate_or_modify_records_or_fabricate_source_times():
    own, gt = [(3., {'id': 1}), (3.008, {'id': 2})], [(3.002, {'id': 3})]
    result = select_source_pair(own, gt, 3.009)
    assert result['encoder'][1] is own[0][1]
    assert result['gt'][1] is gt[0][1]
    assert result['encoder'][0] == 3. and result['gt'][0] == 3.002
    assert len(own) == 2 and len(gt) == 1


def test_boundary_keeps_original_fifty_ms_age_and_twenty_ms_skew():
    pair = select_source_pair([(1., 'own')], [(1.02, 'gt')], 1.05)
    assert pair['encoder'][0] == 1.
