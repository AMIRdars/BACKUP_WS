"""Scalar rotation trajectories."""

import math


class TrackingErrorGovernor:
    """Pause reference progress while the plant catches up.

    The separate resume threshold provides hysteresis, preventing rapid
    pause/resume switching near the tracking-error boundary.
    """

    def __init__(self, pause_error: float, resume_error: float) -> None:
        if not math.isfinite(pause_error) or pause_error <= 0.0:
            raise ValueError('pause_error must be finite and positive.')
        if (not math.isfinite(resume_error) or resume_error < 0.0
                or resume_error > pause_error):
            raise ValueError(
                'resume_error must be finite and within [0, pause_error].')
        self.pause_error = pause_error
        self.resume_error = resume_error
        self.paused = False

    def allow_progress(self, tracking_error: float) -> bool:
        """Return whether the reference may advance for this control tick."""
        error = abs(tracking_error)
        if self.paused:
            if error <= self.resume_error:
                self.paused = False
        elif error >= self.pause_error:
            self.paused = True
        return not self.paused


def settling_transport_angle(
        target_angle: float, actual_angle: float, gain: float,
        maximum_correction: float) -> float:
    """Add bounded endpoint overdrive to remove compliant grasp error."""
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError('gain must be finite and non-negative.')
    if not math.isfinite(maximum_correction) or maximum_correction < 0.0:
        raise ValueError(
            'maximum_correction must be finite and non-negative.')
    error = math.atan2(
        math.sin(target_angle - actual_angle),
        math.cos(target_angle - actual_angle),
    )
    correction = max(
        -maximum_correction, min(maximum_correction, gain * error))
    return target_angle + correction


class ConstantSpeedTrajectory:
    """A signed, bounded constant-angular-speed trajectory."""

    def __init__(self, target_angle: float, angular_speed: float) -> None:
        if not math.isfinite(target_angle):
            raise ValueError('Target angle must be finite.')
        if not math.isfinite(angular_speed) or angular_speed <= 0.0:
            raise ValueError('Angular speed must be positive and finite.')
        self.target_angle = target_angle
        self.angular_speed = angular_speed

    @property
    def duration(self) -> float:
        """Return planned motion duration in seconds."""
        return abs(self.target_angle) / self.angular_speed

    def sample(self, elapsed: float) -> float:
        """Return the bounded desired angle at elapsed seconds."""
        progress = min(
            abs(self.target_angle), self.angular_speed * max(0.0, elapsed))
        return math.copysign(progress, self.target_angle)

    def complete(self, elapsed: float) -> bool:
        """Return whether the target angle has been reached."""
        return elapsed >= self.duration


class AccelerationLimitedTrajectory:
    """A bounded trapezoidal trajectory with smooth start and stop."""

    def __init__(
            self, target_angle: float, angular_speed: float,
            angular_acceleration: float) -> None:
        if not math.isfinite(target_angle):
            raise ValueError('Target angle must be finite.')
        if not math.isfinite(angular_speed) or angular_speed <= 0.0:
            raise ValueError('Angular speed must be positive and finite.')
        if (not math.isfinite(angular_acceleration)
                or angular_acceleration <= 0.0):
            raise ValueError(
                'Angular acceleration must be positive and finite.')
        self.target_angle = target_angle
        self.angular_speed = angular_speed
        self.angular_acceleration = angular_acceleration
        distance = abs(target_angle)
        full_speed_ramp_distance = angular_speed ** 2 / angular_acceleration
        if distance <= full_speed_ramp_distance:
            self.peak_speed = math.sqrt(distance * angular_acceleration)
            self.ramp_time = self.peak_speed / angular_acceleration
            self.cruise_time = 0.0
        else:
            self.peak_speed = angular_speed
            self.ramp_time = angular_speed / angular_acceleration
            self.cruise_time = (
                distance - full_speed_ramp_distance) / angular_speed

    @property
    def duration(self) -> float:
        return 2.0 * self.ramp_time + self.cruise_time

    def sample(self, elapsed: float) -> float:
        time = max(0.0, min(float(elapsed), self.duration))
        distance = abs(self.target_angle)
        if time < self.ramp_time:
            progress = 0.5 * self.angular_acceleration * time ** 2
        elif time < self.ramp_time + self.cruise_time:
            ramp_distance = (
                0.5 * self.angular_acceleration * self.ramp_time ** 2)
            progress = ramp_distance + self.peak_speed * (
                time - self.ramp_time)
        else:
            remaining = self.duration - time
            progress = distance - 0.5 * self.angular_acceleration * remaining ** 2
        return math.copysign(progress, self.target_angle)

    def complete(self, elapsed: float) -> bool:
        return elapsed >= self.duration
