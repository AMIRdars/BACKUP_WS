import math

import pytest

from cooperative_rotation.trajectory import (
    AccelerationLimitedTrajectory,
    ConstantSpeedTrajectory,
    settling_transport_angle,
    TrackingErrorGovernor,
)


def test_acceleration_limited_trajectory_has_smooth_ramps():
    trajectory = AccelerationLimitedTrajectory(10.0, 2.0, 1.0)
    assert trajectory.duration == pytest.approx(7.0)
    assert trajectory.sample(1.0) == pytest.approx(0.5)
    assert trajectory.sample(3.0) == pytest.approx(4.0)
    assert trajectory.sample(6.5) == pytest.approx(9.875)
    assert trajectory.sample(7.0) == pytest.approx(10.0)
    assert trajectory.complete(7.0)


def test_acceleration_limited_trajectory_preserves_negative_direction():
    trajectory = AccelerationLimitedTrajectory(-1.0, 2.0, 1.0)
    assert trajectory.sample(trajectory.duration) == pytest.approx(-1.0)


@pytest.mark.parametrize('acceleration', [0.0, -1.0, math.inf])
def test_acceleration_limited_trajectory_rejects_invalid_acceleration(
        acceleration):
    with pytest.raises(ValueError):
        AccelerationLimitedTrajectory(1.0, 1.0, acceleration)


def test_positive_trajectory_is_continuous_and_bounded():
    trajectory = ConstantSpeedTrajectory(math.pi / 2.0, math.pi / 18.0)
    assert trajectory.duration == pytest.approx(9.0)
    assert trajectory.sample(-1.0) == 0.0
    assert trajectory.sample(4.5) == pytest.approx(math.pi / 4.0)
    assert trajectory.sample(20.0) == pytest.approx(math.pi / 2.0)
    assert trajectory.complete(9.0)


def test_negative_target_preserves_direction():
    trajectory = ConstantSpeedTrajectory(-1.0, 0.2)
    assert trajectory.sample(2.0) == pytest.approx(-0.4)
    assert trajectory.sample(10.0) == pytest.approx(-1.0)


@pytest.mark.parametrize('speed', [0.0, -1.0, math.inf])
def test_invalid_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        ConstantSpeedTrajectory(1.0, speed)


def test_tracking_governor_pauses_and_resumes_with_hysteresis():
    governor = TrackingErrorGovernor(
        math.radians(6.0), math.radians(3.0))
    assert governor.allow_progress(math.radians(5.9))
    assert not governor.allow_progress(math.radians(6.0))
    assert not governor.allow_progress(math.radians(4.0))
    assert governor.allow_progress(math.radians(3.0))


@pytest.mark.parametrize('pause,resume', [
    (0.0, 0.0),
    (1.0, -0.1),
    (1.0, 1.1),
    (math.inf, 0.5),
])
def test_tracking_governor_rejects_invalid_thresholds(pause, resume):
    with pytest.raises(ValueError):
        TrackingErrorGovernor(pause, resume)


def test_settling_transport_angle_compensates_and_clamps():
    corrected = settling_transport_angle(
        math.radians(90.0), math.radians(89.0), 2.0, math.radians(4.0))
    assert math.degrees(corrected) == pytest.approx(92.0)

    clamped = settling_transport_angle(
        math.radians(90.0), math.radians(80.0), 2.0, math.radians(4.0))
    assert math.degrees(clamped) == pytest.approx(94.0)


@pytest.mark.parametrize('gain,limit', [(-1.0, 1.0), (1.0, -1.0)])
def test_settling_transport_angle_rejects_invalid_values(gain, limit):
    with pytest.raises(ValueError):
        settling_transport_angle(1.0, 0.9, gain, limit)
