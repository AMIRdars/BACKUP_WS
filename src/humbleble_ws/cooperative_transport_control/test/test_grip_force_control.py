import math

import pytest

from cooperative_transport_control.grip_force_control import (
    holding_recovery_position,
    next_gripper_position,
)


def _next(position, forces, contact=True):
    return next_gripper_position(
        position, forces, contact, target_force=50.0,
        force_tolerance=5.0, position_gain=0.001,
        maximum_step=0.01, search_step=0.02,
        minimum_position=-1.0, maximum_position=0.15)


def test_searches_forward_before_bilateral_contact():
    assert math.isclose(_next(-0.5, (0.0, 0.0), False), -0.48)


def test_backs_off_from_large_unilateral_force():
    assert math.isclose(_next(-0.3, (100.0, 0.0), False), -0.31)


def test_closes_when_force_is_below_target():
    assert math.isclose(_next(-0.3, (30.0, 40.0)), -0.29)


def test_opens_when_force_is_above_target():
    assert math.isclose(_next(-0.3, (70.0, 80.0)), -0.31)


def test_holds_position_inside_force_deadband():
    assert math.isclose(_next(-0.3, (47.0, 53.0)), -0.3)


def test_rejects_missing_finger_force():
    with pytest.raises(ValueError):
        next_gripper_position(
            0.0, (1.0,), True, 50.0, 5.0, 0.001, 0.01, 0.02,
            -1.0, 0.15)


def _recover(position, command, forces):
    return holding_recovery_position(
        position, command, forces, target_force=17.0,
        minimum_force=8.0, release_force=18.0,
        position_gain=0.001, maximum_step=0.01,
        minimum_position=-1.0, maximum_position=0.15)


def test_holding_keeps_command_inside_the_release_deadband():
    assert _recover(-0.50, -0.51, (9.0, 17.0)) == -0.51


def test_holding_opens_when_a_finger_exceeds_the_release_threshold():
    assert _recover(-0.50, -0.51, (21.0, 23.0)) == pytest.approx(-0.506)


def test_holding_only_closes_to_recover_weak_contact():
    assert _recover(-0.50, -0.50, (7.0, 19.0)) == -0.49


def test_holding_retention_mode_does_not_open_on_force_peak():
    position = holding_recovery_position(
        -0.50, -0.50, (23.0, 24.0), target_force=14.0,
        minimum_force=8.0, release_force=18.0,
        position_gain=0.001, maximum_step=0.01,
        minimum_position=-1.0, maximum_position=0.15,
        allow_release=False)
    assert position == -0.50


def test_holding_retention_mode_still_recovers_weak_contact():
    position = holding_recovery_position(
        -0.50, -0.50, (4.0, 21.0), target_force=14.0,
        minimum_force=8.0, release_force=18.0,
        position_gain=0.001, maximum_step=0.01,
        minimum_position=-1.0, maximum_position=0.15,
        allow_release=False)
    assert position == -0.49
