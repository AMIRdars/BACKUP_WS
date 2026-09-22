import math

import numpy as np
import pytest

from cooperative_rotation.transform_utils import (
    axis_angle_to_rotation,
    invert_transform,
    make_transform,
    quaternion_to_rpy,
    rotate_transform_about_axis,
    rotation_error,
    signed_axis_rotation,
    transform_components,
)


def test_rodrigues_rotates_x_to_y_about_z():
    rotation = axis_angle_to_rotation([0.0, 0.0, 2.0], math.pi / 2.0)
    assert rotation @ np.array([1.0, 0.0, 0.0]) == pytest.approx(
        [0.0, 1.0, 0.0], abs=1.0e-12)


def test_zero_axis_is_rejected():
    with pytest.raises(ValueError):
        axis_angle_to_rotation([0.0, 0.0, 0.0], 1.0)


def test_transform_round_trip_and_inverse():
    transform = make_transform(
        [1.0, -2.0, 0.4], [0.0, 0.0, math.sin(0.3), math.cos(0.3)])
    assert transform @ invert_transform(transform) == pytest.approx(np.eye(4))
    translation, quaternion = transform_components(transform)
    rebuilt = make_transform(translation, quaternion)
    assert rebuilt == pytest.approx(transform)


def test_offset_axis_rotates_position_and_orientation():
    initial = make_transform([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    target = rotate_transform_about_axis(
        initial, [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], math.pi / 2.0)
    assert target[:3, 3] == pytest.approx([1.0, 1.0, 0.0])
    assert target[:3, 0] == pytest.approx([0.0, 1.0, 0.0], abs=1.0e-12)


def test_relative_grasp_transform_remains_constant():
    object_initial = make_transform(
        [0.2, -0.1, 0.4], [0.0, 0.0, 0.0, 1.0])
    gripper_initial = make_transform(
        [-0.4, -0.1, 0.4], [0.0, 0.0, 0.0, 1.0])
    object_to_gripper = invert_transform(object_initial) @ gripper_initial
    object_target = rotate_transform_about_axis(
        object_initial, object_initial[:3, 3], [0.0, 0.0, 1.0], 0.7)
    gripper_target = object_target @ object_to_gripper
    assert invert_transform(object_target) @ gripper_target == pytest.approx(
        object_to_gripper)


def test_signed_angle_and_orientation_error():
    initial = np.eye(3)
    current = axis_angle_to_rotation([0.0, 0.0, 1.0], -0.4)
    assert signed_axis_rotation(initial, current, [0.0, 0.0, 1.0]) \
        == pytest.approx(-0.4)
    assert rotation_error(current, initial) == pytest.approx(0.4)


def test_quaternion_to_rpy_uses_world_x_y_z_angles():
    quaternion = [
        0.0, 0.0, math.sin(math.radians(30.0) / 2.0),
        math.cos(math.radians(30.0) / 2.0),
    ]
    roll, pitch, yaw = quaternion_to_rpy(quaternion)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert math.degrees(yaw) == pytest.approx(30.0)
