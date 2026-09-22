"""Bounded one-axis admittance used to relieve cooperative internal force."""

from typing import NamedTuple


class AdmittanceState(NamedTuple):
    """Observable state of the one-axis virtual mass-spring-damper."""

    filtered_force: float
    displacement: float
    velocity: float


class OneAxisAdmittance:
    """Integrate ``M*a + D*v + K*x = F`` with safety bounds."""

    def __init__(
        self,
        mass: float,
        damping: float,
        stiffness: float,
        force_deadband: float,
        filter_time_constant: float,
        max_displacement: float,
        max_velocity: float,
    ) -> None:
        if mass <= 0.0:
            raise ValueError('mass must be positive')
        if damping < 0.0 or stiffness < 0.0:
            raise ValueError('damping and stiffness must be non-negative')
        if force_deadband < 0.0 or filter_time_constant < 0.0:
            raise ValueError('filter and deadband must be non-negative')
        if max_displacement < 0.0 or max_velocity < 0.0:
            raise ValueError('admittance limits must be non-negative')
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.force_deadband = force_deadband
        self.filter_time_constant = filter_time_constant
        self.max_displacement = max_displacement
        self.max_velocity = max_velocity
        self.reset()

    def reset(self) -> None:
        """Clear filter and virtual motion state."""
        self.filtered_force = 0.0
        self.displacement = 0.0
        self.velocity = 0.0

    def update(self, force: float, dt: float) -> AdmittanceState:
        """Advance one bounded integration step and return its new state."""
        dt = max(1.0e-6, dt)
        if self.filter_time_constant > 0.0:
            alpha = dt / (self.filter_time_constant + dt)
            self.filtered_force += alpha * (force - self.filtered_force)
        else:
            self.filtered_force = force

        magnitude = abs(self.filtered_force)
        if magnitude <= self.force_deadband:
            effective_force = 0.0
        else:
            direction = 1.0 if self.filtered_force > 0.0 else -1.0
            effective_force = (
                self.filtered_force
                - self.force_deadband * direction
            )
        acceleration = (
            effective_force
            - self.damping * self.velocity
            - self.stiffness * self.displacement
        ) / self.mass
        velocity = self.velocity + acceleration * dt
        velocity = max(-self.max_velocity, min(self.max_velocity, velocity))
        displacement = self.displacement + velocity * dt
        displacement = max(
            -self.max_displacement,
            min(self.max_displacement, displacement),
        )
        at_positive_limit = (
            displacement >= self.max_displacement and velocity > 0.0)
        at_negative_limit = (
            displacement <= -self.max_displacement and velocity < 0.0)
        if at_positive_limit or at_negative_limit:
            velocity = 0.0
        self.displacement = displacement
        self.velocity = velocity
        return AdmittanceState(
            self.filtered_force, self.displacement, self.velocity)
