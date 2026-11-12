"""Verify reference continuity, velocity bounds and physical distance integral."""
import numpy as np
import pytest
from amir_cp3_trial import reference


@pytest.mark.parametrize('distance,speed,acceleration', [(.001, .00025, .001), (.00001, .00025, .001)])
def test_reference_integral_and_bounds(distance, speed, acceleration):
    duration = reference(0, distance, speed, acceleration)[2]
    times = np.linspace(0, duration, 10001)
    values = np.array([reference(t, distance, speed, acceleration)[:2] for t in times])
    assert np.trapz(values[:, 1], times) == pytest.approx(distance, rel=1e-6)
    assert np.all(np.diff(values[:, 0]) >= -1e-12)
    assert np.max(values[:, 1]) <= speed
    assert np.max(np.abs(np.diff(values[:, 1])/np.diff(times))) <= acceleration+1e-10
    assert reference(-1, distance, speed, acceleration)[:2] == (0., 0.)
    assert reference(duration+1, distance, speed, acceleration)[:2] == (distance, 0.)
