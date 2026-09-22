"""Tests for wrist force/torque metrics and safety thresholds."""

import pytest

from cooperative_transport_control.wrench_metrics import RunningWrenchMetrics
from cooperative_transport_control.wrench_metrics import WrenchVector
from cooperative_transport_control.wrench_metrics import stop_reason


def test_wrench_norms_and_running_statistics():
    first = WrenchVector(3.0, 4.0, 0.0, 0.0, 0.0, 2.0)
    second = WrenchVector(0.0, 0.0, 12.0, 0.0, 3.0, 4.0)
    metrics = RunningWrenchMetrics()
    metrics.update(first)
    metrics.update(second)
    assert first.force_norm == pytest.approx(5.0)
    assert metrics.force_rms == pytest.approx((0.5 * (25.0 + 144.0)) ** 0.5)
    assert metrics.torque_rms == pytest.approx((0.5 * (4.0 + 25.0)) ** 0.5)
    assert metrics.peak_force == pytest.approx(12.0)
    assert metrics.peak_torque == pytest.approx(5.0)


@pytest.mark.parametrize(
    'wrench, expected', [
        (WrenchVector(51.0, 0.0, 0.0, 0.0, 0.0, 0.0), 'Fx='),
        (WrenchVector(0.0, -51.0, 0.0, 0.0, 0.0, 0.0), 'Fy='),
        (WrenchVector(0.0, 0.0, 101.0, 0.0, 0.0, 0.0), 'Fz='),
        (WrenchVector(0.0, 0.0, 0.0, 0.0, 0.0, 16.0), 'Mz='),
    ])
def test_stop_reason_reports_exceeded_component(wrench, expected):
    assert stop_reason(wrench, 50.0, 100.0, 15.0).startswith(expected)


def test_stop_reason_accepts_values_at_limits():
    wrench = WrenchVector(50.0, -50.0, 100.0, 15.0, -15.0, 15.0)
    assert stop_reason(wrench, 50.0, 100.0, 15.0) is None
