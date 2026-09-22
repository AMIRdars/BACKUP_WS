"""Tests for the bounded single-axis admittance controller."""

import pytest

from cooperative_transport_control.admittance import OneAxisAdmittance


def controller(**overrides):
    """Return a deterministic controller with optional parameter changes."""
    values = {
        'mass': 3.0,
        'damping': 10.0,
        'stiffness': 20.0,
        'force_deadband': 1.0,
        'filter_time_constant': 0.0,
        'max_displacement': 0.02,
        'max_velocity': 0.01,
    }
    values.update(overrides)
    return OneAxisAdmittance(**values)


def test_deadband_produces_no_motion() -> None:
    state = controller().update(0.5, 0.02)
    assert state.displacement == pytest.approx(0.0)
    assert state.velocity == pytest.approx(0.0)


def test_force_generates_bounded_motion_in_same_direction() -> None:
    admittance = controller()
    states = [admittance.update(10.0, 0.02) for _ in range(200)]
    assert states[0].velocity > 0.0
    assert states[-1].displacement <= 0.02
    assert max(abs(state.velocity) for state in states) <= 0.01


def test_filter_rejects_an_instantaneous_step() -> None:
    state = controller(filter_time_constant=0.18).update(10.0, 0.02)
    assert state.filtered_force == pytest.approx(1.0)
    assert state.velocity == pytest.approx(0.0)


def test_reset_clears_dynamic_state() -> None:
    admittance = controller()
    admittance.update(-10.0, 0.02)
    admittance.reset()
    assert admittance.filtered_force == 0.0
    assert admittance.displacement == 0.0
    assert admittance.velocity == 0.0


@pytest.mark.parametrize('mass', [0.0, -1.0])
def test_non_positive_mass_is_rejected(mass: float) -> None:
    with pytest.raises(ValueError):
        controller(mass=mass)
