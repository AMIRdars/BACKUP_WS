"""Pure calculations for cooperative-transport wrist wrench monitoring."""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class WrenchVector:
    """Six-axis wrench components in one wrist sensor frame."""

    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float

    @property
    def force_norm(self) -> float:
        """Return resultant force magnitude in newtons."""
        return math.sqrt(self.fx ** 2 + self.fy ** 2 + self.fz ** 2)

    @property
    def torque_norm(self) -> float:
        """Return resultant moment magnitude in newton-metres."""
        return math.sqrt(self.tx ** 2 + self.ty ** 2 + self.tz ** 2)


def stop_reason(
        wrench: WrenchVector, planar_force_limit: float,
        vertical_force_limit: float,
        torque_limit: float) -> Optional[str]:
    """Return the first component threshold exceeded, or ``None``."""
    for axis, value in (('Fx', wrench.fx), ('Fy', wrench.fy)):
        if abs(value) > planar_force_limit:
            return f'{axis}={value:.2f} N exceeds {planar_force_limit:.2f} N'
    if abs(wrench.fz) > vertical_force_limit:
        return (
            f'Fz={wrench.fz:.2f} N exceeds '
            f'{vertical_force_limit:.2f} N')
    for axis, value in (('Mx', wrench.tx), ('My', wrench.ty),
                        ('Mz', wrench.tz)):
        if abs(value) > torque_limit:
            return f'{axis}={value:.2f} Nm exceeds {torque_limit:.2f} Nm'
    return None


class RunningWrenchMetrics:
    """Accumulate resultant-force and resultant-moment RMS and peaks."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Clear all accumulated samples."""
        self.count = 0
        self.force_squared_sum = 0.0
        self.torque_squared_sum = 0.0
        self.peak_force = 0.0
        self.peak_torque = 0.0

    def update(self, wrench: WrenchVector) -> None:
        """Add one six-axis sample."""
        force = wrench.force_norm
        torque = wrench.torque_norm
        self.count += 1
        self.force_squared_sum += force ** 2
        self.torque_squared_sum += torque ** 2
        self.peak_force = max(self.peak_force, force)
        self.peak_torque = max(self.peak_torque, torque)

    @property
    def force_rms(self) -> float:
        """Return resultant-force RMS, or zero before the first sample."""
        return math.sqrt(self.force_squared_sum / self.count) \
            if self.count else 0.0

    @property
    def torque_rms(self) -> float:
        """Return resultant-moment RMS, or zero before the first sample."""
        return math.sqrt(self.torque_squared_sum / self.count) \
            if self.count else 0.0
