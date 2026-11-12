"""A balanced average cannot hide an individual finger outside its target."""

from collections import deque
from types import SimpleNamespace

import pytest

from cooperative_rotation.rotation_controller import CooperativeRotationController


def reason(forces):
    parameters = {
        'pre_rotation_force_stability_duration': 2.0,
        'pre_rotation_minimum_normal_force': 8.0,
        'pre_rotation_target_normal_force': 20.0,
        'pre_rotation_target_tolerance': 3.0,
        'pre_rotation_force_variation_limit': 2.0,
        'pre_rotation_force_timeout': 0.25,
    }
    controller = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=parameters[key]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
        _finger_force_history={
            index: deque((stamp, force) for stamp in (7.9, 8.1, 9.0, 10.0))
            for index, force in enumerate(forces)},
    )
    return CooperativeRotationController._pre_rotation_force_stability_reason(controller)


def test_all_fingers_settled_near_twenty_newtons_can_start():
    assert reason((18.5, 21.5, 19.0, 22.0)) is None


@pytest.mark.parametrize('forces', [
    (16.0, 24.0, 16.0, 24.0),
    (20.0, 20.0, float('nan'), 20.0),
])
def test_mean_twenty_newtons_does_not_hide_an_invalid_finger(forces):
    assert reason(forces) == 'waiting for each finger force near target'
