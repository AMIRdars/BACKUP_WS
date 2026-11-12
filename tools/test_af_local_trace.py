"""Ensure the evaluator does not alias F/T peaks into its slower CSV timer."""
import csv
import json
import pytest
from amir_af_local_trace import LocalControlTrace


def test_every_received_sample_and_distinct_clock_are_retained(tmp_path):
    trace = LocalControlTrace(tmp_path)
    for i, force in enumerate([.1, .51, .1]):
        trace.wrench(kind='filtered', robot='follower', phase='leader_translation',
            source_stamp_s=10 + i * .01, receipt_sim_time_s=10.004 + i * .01,
            receipt_wall_time_s=200 + i * .01, frame_id='follower/grasp_aligned',
            admittance_enabled=True, values=[force, 0, 0, 0, 0, 0])
    trace.close()
    rows = list(csv.DictReader((tmp_path / 'wrench_samples.csv').open()))
    assert len(rows) == 3
    assert max(float(r['wrench_0']) for r in rows) == .51
    assert float(rows[1]['source_stamp_s']) == 10.01
    assert float(rows[1]['receipt_sim_time_s']) == pytest.approx(10.014)
    assert float(rows[1]['receipt_wall_time_s']) == 200.01


def test_fault_peak_and_latched_step_are_not_dropped(tmp_path):
    trace = LocalControlTrace(tmp_path)
    trace.control({'source_stamp_s': 10., 'state': 'TRANSPORT',
                   'e_track_xy_m': [1e-6, 2e-6]},
                  phase='leader_translation', receipt_sim_time_s=10.004,
                  receipt_wall_time_s=200.)
    value = {'source_stamp_s': 10., 'state': 'SAFE_STOP',
             'sample_is_last_control_step': True,
             'wrench_at_fault': [.52, 0, 0, 0, 0, 0]}
    trace.control(value, phase='leader_translation', receipt_sim_time_s=10.014,
                  receipt_wall_time_s=200.01)
    trace.close()
    rows = list(csv.DictReader((tmp_path / 'control_samples.csv').open()))
    assert rows[1]['sample_is_last_control_step'] == 'True'
    assert rows[1]['wrench_at_fault_0'] == '0.52'
    assert rows[0]['e_track_xy_m_1'] == '2e-06'
    saved = [json.loads(s) for s in (tmp_path / 'control_samples.jsonl').read_text().splitlines()]
    assert saved[1]['value'] == value


def test_empty_trace_has_headers_and_closes(tmp_path):
    trace = LocalControlTrace(tmp_path)
    with pytest.raises(ValueError, match='six wrench'):
        trace.wrench(kind='raw', robot='leader', phase='startup', source_stamp_s=1.,
            receipt_sim_time_s=1., receipt_wall_time_s=2., frame_id='sensor',
            admittance_enabled=False, values=[])
    trace.close()
    assert list(csv.DictReader((tmp_path / 'wrench_samples.csv').open())) == []
