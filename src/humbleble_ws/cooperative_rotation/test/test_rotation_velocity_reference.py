"""Published velocities must describe the same rigid motion as the pose goal."""

from types import SimpleNamespace

import numpy as np
import pytest

from cooperative_rotation.rotation_controller import CooperativeRotationController


@pytest.mark.parametrize('speed', [0.02, -0.02, 0.0])
def test_offset_axis_rotation_publishes_matching_linear_and_angular_velocity(speed):
    initial = np.eye(4)
    initial[0, 3] = 1.0
    calls = []
    controller = SimpleNamespace(
        _object_initial=initial,
        _axis_point=np.zeros(3),
        _axis=np.array([0.0, 0.0, 1.0]),
        _command_transform=lambda *args: calls.append(args))
    CooperativeRotationController._command_angle(
        controller, np.pi / 2, angular_velocity=speed)
    target, _, linear, angular = calls[0]
    assert target[:2, 3] == pytest.approx([0.0, 1.0], abs=1.0e-12)
    assert linear == pytest.approx([-speed, 0.0, 0.0], abs=1.0e-12)
    assert angular == speed


def test_settling_pose_correction_does_not_publish_a_stale_rotation_velocity():
    calls = []
    controller = SimpleNamespace(
        _object_initial=np.eye(4), _axis_point=np.zeros(3),
        _axis=np.array([0.0, 0.0, 1.0]),
        _command_transform=lambda *args: calls.append(args))
    CooperativeRotationController._command_angle(controller, 1.0, 1.1)
    assert calls[0][2] == pytest.approx([0.0, 0.0, 0.0])
    assert calls[0][3] == 0.0
