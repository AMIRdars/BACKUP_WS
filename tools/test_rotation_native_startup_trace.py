import json

import pytest

from amir_rotation_native_startup_trace import NativeStartupTrace


def test_post_tare_requires_both_original_source_stamps_to_advance(tmp_path):
    trace = NativeStartupTrace(tmp_path, ['tare'])
    assert not trace.pair_after_tare(1., 1.01)
    trace.wrench('raw', 1.01, 'follower/grasp_aligned', [0.]*6, 1.01)
    trace.wrench('filtered', 1., 'follower/grasp_aligned', [0.]*6, 1.01)
    assert not trace.pair_after_tare(1., 1.01)
    trace.wrench('filtered', 1.01, 'follower/grasp_aligned', [0.]*6, 1.01)
    assert trace.pair_after_tare(1., 1.01)
    assert not trace.pair_after_tare(1., 1.07)
    assert not trace.pair_after_tare(1., .98)
    trace.wrench('raw', 1.04, 'follower/grasp_aligned', [0.]*6, 1.04)
    assert not trace.pair_after_tare(1., 1.04)
    trace.close()


def test_preserves_raw_spike_and_actual_command_error_without_filtering(tmp_path):
    trace = NativeStartupTrace(tmp_path, ['native_passive_static'])
    trace.wrench('raw', 2., 'follower/grasp_aligned', [2.2, 0., 0., .051, 0., 0.], 2.004)
    trace.command(list('abcde'), [1., 2., 3., 4., 5.], 2., 2.004, list('edcba'), [4., 3., 2., 1., 0.], 2.)
    assert trace.close() == {'raw': 1, 'arm_command': 1}
    rows = [json.loads(line) for line in (tmp_path/'native_startup_events.jsonl').read_text().splitlines()]
    assert rows[0]['wrench'] == [2.2, 0., 0., .051, 0., 0.]
    assert 0.<rows[0]['wall_monotonic_s']<=rows[1]['wall_monotonic_s']
    assert 0.<rows[0]['wall_epoch_s']<=rows[1]['wall_epoch_s']
    assert rows[1]['error'] == [1.]*5
    assert '2.2' in (tmp_path/'native_startup_events.csv').read_text()


@pytest.mark.parametrize('frame,vector', [('world', [0.]*6), ('follower/grasp_aligned', [0.]*5),
    ('follower/grasp_aligned', [float('nan')]*6)])
def test_invalid_trace_data_is_refused(tmp_path, frame, vector):
    trace = NativeStartupTrace(tmp_path, ['tare'])
    with pytest.raises(ValueError):
        trace.wrench('raw', 1., frame, vector, 1.)
    trace.close()
