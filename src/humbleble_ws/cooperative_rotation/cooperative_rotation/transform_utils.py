"""Dependency-light homogeneous-transform helpers for rotation planning."""

import math
from typing import Iterable, Tuple

import numpy as np


_EPSILON = 1.0e-12


def normalized(vector: Iterable[float]) -> np.ndarray:
    """Return a unit-length three-vector, rejecting a zero-length input."""
    result = np.asarray(vector, dtype=float)
    if result.shape != (3,):
        raise ValueError('Expected a three-element vector.')
    norm = float(np.linalg.norm(result))
    if norm <= _EPSILON:
        raise ValueError('Rotation axis must have non-zero length.')
    return result / norm


def axis_angle_to_rotation(axis: Iterable[float], angle: float) -> np.ndarray:
    """Create a 3x3 active rotation matrix using Rodrigues' formula."""
    x, y, z = normalized(axis)
    skew = np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])
    identity = np.eye(3)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        identity * cosine
        + (1.0 - cosine) * np.outer((x, y, z), (x, y, z))
        + skew * sine
    )


def quaternion_to_rotation(quaternion: Iterable[float]) -> np.ndarray:
    """Convert an x,y,z,w quaternion to a 3x3 rotation matrix."""
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,):
        raise ValueError('Expected an x,y,z,w quaternion.')
    norm = float(np.linalg.norm(quaternion))
    if norm <= _EPSILON:
        raise ValueError('Quaternion must have non-zero length.')
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),
         2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def quaternion_to_rpy(quaternion: Iterable[float]) -> Tuple[float, float, float]:
    """Convert an x,y,z,w quaternion to world-frame roll, pitch, and yaw."""
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,):
        raise ValueError('Expected an x,y,z,w quaternion.')
    norm = float(np.linalg.norm(quaternion))
    if norm <= _EPSILON:
        raise ValueError('Quaternion must have non-zero length.')
    x, y, z, w = quaternion / norm
    roll = math.atan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = 2.0 * (w * y - z * x)
    pitch = (math.copysign(math.pi / 2.0, pitch_sine)
             if abs(pitch_sine) >= 1.0 else math.asin(pitch_sine))
    yaw = math.atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized x,y,z,w quaternion."""
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('Expected a 3x3 rotation matrix.')
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
        w = 0.25 * scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1]
                - rotation[2, 2]) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0]
                - rotation[2, 2]) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0]
                - rotation[1, 1]) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale
    quaternion = np.array([x, y, z, w])
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def make_transform(
        translation: Iterable[float], quaternion: Iterable[float]
) -> np.ndarray:
    """Create a homogeneous transform from translation and quaternion."""
    translation = np.asarray(translation, dtype=float)
    if translation.shape != (3,):
        raise ValueError('Expected a three-element translation.')
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_rotation(quaternion)
    transform[:3, 3] = translation
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid homogeneous transform."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError('Expected a 4x4 homogeneous transform.')
    inverse = np.eye(4)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def rotate_transform_about_axis(
        initial: np.ndarray,
        axis_point: Iterable[float],
        axis_direction: Iterable[float],
        angle: float,
) -> np.ndarray:
    """Rotate a world pose about a world-frame line by ``angle`` radians."""
    initial = np.asarray(initial, dtype=float)
    point = np.asarray(axis_point, dtype=float)
    if initial.shape != (4, 4) or point.shape != (3,):
        raise ValueError('Expected a 4x4 transform and a three-element point.')
    rotation = axis_angle_to_rotation(axis_direction, angle)
    target = np.eye(4)
    target[:3, :3] = rotation @ initial[:3, :3]
    target[:3, 3] = point + rotation @ (initial[:3, 3] - point)
    return target


def transform_components(
        transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return translation and x,y,z,w quaternion from a transform."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError('Expected a 4x4 homogeneous transform.')
    return transform[:3, 3].copy(), rotation_to_quaternion(transform[:3, :3])


def signed_axis_rotation(
        initial_rotation: np.ndarray,
        current_rotation: np.ndarray,
        axis: Iterable[float],
) -> float:
    """Estimate signed rotation about ``axis`` from two orientations."""
    relative = np.asarray(current_rotation) @ np.asarray(initial_rotation).T
    axis = normalized(axis)
    sine = 0.5 * float(np.dot(axis, np.array([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ])))
    cosine = max(-1.0, min(1.0, 0.5 * (float(np.trace(relative)) - 1.0)))
    return math.atan2(sine, cosine)


def rotation_error(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular distance between two orientations."""
    relative = np.asarray(first) @ np.asarray(second).T
    cosine = max(-1.0, min(1.0, 0.5 * (float(np.trace(relative)) - 1.0)))
    return math.acos(cosine)
