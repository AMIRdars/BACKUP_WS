"""Unsafe CLI environments must exit before process creation or ROS init."""
import sys
from unittest.mock import Mock

import pytest

import amir_rotation_base_fixture as fixture
import amir_rotation_base_probe as probe


@pytest.mark.parametrize('kind', ['probe', 'fixture'])
def test_unisolated_cli_cannot_create_output_or_ros_process(tmp_path, monkeypatch, kind):
    output = tmp_path/'forbidden'
    monkeypatch.delenv('IGN_PARTITION', raising=False)
    forbidden = Mock(side_effect=AssertionError('ROS/process creation forbidden'))
    monkeypatch.setattr(probe.rclpy, 'init', forbidden)
    monkeypatch.setattr(probe.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(fixture, 'prepare', forbidden)
    argv = ['test', '--output', str(output), '--reference', str(tmp_path/'unused')]
    if kind == 'probe':
        argv += ['--limits-policy', 'unused', '--leader-limits-policy', 'unused', '--hold-s', '10']
    monkeypatch.setattr(sys, 'argv', argv)
    with pytest.raises(SystemExit) as error:
        (probe if kind == 'probe' else fixture).main()
    assert error.value.code == 2
    assert not output.exists() and not forbidden.called


@pytest.mark.parametrize('hold', ['nan', '0'])
def test_invalid_observation_duration_cannot_start_physics(tmp_path, monkeypatch, hold):
    for key, value in dict(ROS_DISTRO='humble', ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='209',
                           IGN_PARTITION='amir_cp8r3_base_metrology_guard_test').items():
        monkeypatch.setenv(key, value)
    output = tmp_path/'forbidden'
    forbidden = Mock(side_effect=AssertionError('ROS/process creation forbidden'))
    monkeypatch.setattr(probe.rclpy, 'init', forbidden)
    monkeypatch.setattr(probe.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(sys, 'argv', ['test', '--output', str(output), '--reference', 'unused',
        '--limits-policy', 'unused', '--leader-limits-policy', 'unused', '--hold-s', hold])
    with pytest.raises(SystemExit):
        probe.main()
    assert not output.exists() and not forbidden.called
