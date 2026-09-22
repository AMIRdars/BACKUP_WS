"""Pure planar math used by the cooperative-motion Action server."""

import math
from typing import Iterable, List, NamedTuple, Tuple


class Pose2(NamedTuple):
    """Planar object pose in the configured world frame."""

    x: float
    y: float
    yaw: float


class Twist2(NamedTuple):
    """Planar velocity in a robot base frame."""

    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _matmul(left: List[List[float]], right: List[List[float]]) -> List[List[float]]:
    return [
        [sum(left[row][index] * right[index][column] for index in range(4))
         for column in range(4)]
        for row in range(4)
    ]


def _translation(x: float, y: float, z: float) -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_x(angle: float) -> List[List[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cosine, -sine, 0.0],
        [0.0, sine, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_y(angle: float) -> List[List[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        [cosine, 0.0, sine, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sine, 0.0, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_z(angle: float) -> List[List[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        [cosine, -sine, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _arm_gripper_transform(joints: Iterable[float]) -> List[List[float]]:
    """Return the transform of AMIR's ``gripper_base_1``."""
    q = tuple(joints)
    if len(q) != 5:
        raise ValueError('AMIR arm FK requires five joint positions')

    transform = _rotation_z(-0.5 * math.pi)
    for translation, rotation in (
            ((0.0, -0.001523, -0.097), _rotation_z(q[0])),
            ((0.0, 0.0, 0.097), _rotation_x(q[1])),
            ((0.0, 0.31, 0.0), _rotation_x(q[2])),
            ((0.0, 0.31, 0.0), _rotation_x(q[3])),
            ((0.0, 0.0405, 0.0), _rotation_y(q[4])),
    ):
        transform = _matmul(transform, _translation(*translation))
        transform = _matmul(transform, rotation)
    return transform


def arm_grasp_position(joints: Iterable[float]) -> Tuple[float, float, float]:
    """Return ``gripper_base_1`` relative to ``base_footprint``.

    The cooperative payload attaches to ``gripper_base_1``. Keeping this
    point fixed, rather than the visual TCP marker, preserves the actual
    simulated and physical grasp.
    """
    transform = _arm_gripper_transform(joints)
    return transform[0][3], transform[1][3], transform[2][3]


def arm_tcp_position(joints: Iterable[float]) -> Tuple[float, float, float]:
    """Return AMIR TCP position relative to ``base_footprint``.

    The AMIR URDF mounts the arm on ``base_link``, which is rotated -90 deg
    from ``base_footprint``.  The five transforms below mirror the joint
    origins and axes in ``amir_for_rover.xacro``.  The TCP marker is useful for
    visualization; grasp compensation uses ``arm_grasp_position`` below.
    """
    transform = _arm_gripper_transform(joints)
    # tcp_fixed: gripper_base_1 -> tcp_link
    transform = _matmul(transform, _translation(0.0, 0.0795, 0.0))
    return transform[0][3], transform[1][3], transform[2][3]


def tcp_world_position(base: Pose2, joints: Iterable[float]) -> Tuple[float, float, float]:
    """Return the TCP position in world coordinates."""
    local_x, local_y, local_z = arm_tcp_position(joints)
    cosine, sine = math.cos(base.yaw), math.sin(base.yaw)
    return (
        base.x + cosine * local_x - sine * local_y,
        base.y + sine * local_x + cosine * local_y,
        local_z,
    )


def grasp_world_position(base: Pose2, joints: Iterable[float]) -> Tuple[float, float, float]:
    """Return the attached gripper base position in world coordinates."""
    local_x, local_y, local_z = arm_grasp_position(joints)
    cosine, sine = math.cos(base.yaw), math.sin(base.yaw)
    return (
        base.x + cosine * local_x - sine * local_y,
        base.y + sine * local_x + cosine * local_y,
        local_z,
    )


def world_to_body_velocity(vx: float, vy: float, body_yaw: float) -> Tuple[float, float]:
    """Rotate a world-frame velocity into AMIR's base frame."""
    cosine, sine = math.cos(body_yaw), math.sin(body_yaw)
    return cosine * vx + sine * vy, -sine * vx + cosine * vy


def clamp_vector(x: float, y: float, limit: float) -> Tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude <= limit or magnitude == 0.0:
        return x, y
    scale = limit / magnitude
    return scale * x, scale * y


def rate_limit(previous: Twist2, target: Twist2, dt: float,
               max_linear_acceleration: float,
               max_angular_acceleration: float) -> Twist2:
    """Limit a planar command's acceleration."""
    delta_x, delta_y = clamp_vector(
        target.x - previous.x, target.y - previous.y,
        max_linear_acceleration * dt)
    delta_yaw = max(
        -max_angular_acceleration * dt,
        min(max_angular_acceleration * dt, target.yaw - previous.yaw),
    )
    return Twist2(
        previous.x + delta_x,
        previous.y + delta_y,
        previous.yaw + delta_yaw,
    )


def resolve_target_yaw(initial_yaw: float, angle_mode: int,
                       target_yaw: float, rotation_angle: float,
                       absolute_mode: int) -> float:
    """Resolve a goal's absolute or relative orientation once at start."""
    if angle_mode == absolute_mode:
        return normalize_angle(target_yaw)
    return normalize_angle(initial_yaw + rotation_angle)


def rotate_point_about_pivot(initial: Pose2, pivot_x: float, pivot_y: float,
                             angle: float) -> Tuple[float, float]:
    """Return the ideal object-center position after a planar pivot turn."""
    c = math.cos(angle)
    s = math.sin(angle)
    dx = initial.x - pivot_x
    dy = initial.y - pivot_y
    return pivot_x + c * dx - s * dy, pivot_y + s * dx + c * dy


def pivot_velocity(current: Pose2, pivot_x: float, pivot_y: float,
                   desired_x: float, desired_y: float, target_yaw: float,
                   position_kp: float, yaw_kp: float,
                   max_angular_velocity: float) -> Tuple[float, float, float]:
    """Object-frame command specified in the implementation guide, for feedback."""
    yaw_error = normalize_angle(target_yaw - current.yaw)
    wz = max(-max_angular_velocity, min(max_angular_velocity, yaw_kp * yaw_error))
    vx = -wz * (current.y - pivot_y) + position_kp * (desired_x - current.x)
    vy = wz * (current.x - pivot_x) + position_kp * (desired_y - current.y)
    return vx, vy, wz
