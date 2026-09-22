"""Planar rigid-payload kinematics shared by the coordinator and tests."""

import math
from typing import NamedTuple, Optional, Tuple


class Pose2(NamedTuple):
    """Planar pose."""

    x: float
    y: float
    yaw: float


class Twist2(NamedTuple):
    """Planar twist in a robot base frame."""

    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw without depending on tf_transformations / legacy NumPy APIs."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def estimate_payload_pose(robot1: Pose2, robot2: Pose2) -> Pose2:
    """
    Estimate payload center and yaw from the two base centers.

    Robot 1 is placed at the negative end and robot 2 at the positive end of
    the payload, so the vector robot1 -> robot2 defines payload +X.
    """
    return Pose2(
        0.5 * (robot1.x + robot2.x),
        0.5 * (robot1.y + robot2.y),
        math.atan2(robot2.y - robot1.y, robot2.x - robot1.x),
    )


def clamp(value: float, limit: float) -> float:
    """Symmetrically clamp a scalar."""
    return max(-limit, min(limit, value))


def clamp_vector(x: float, y: float, limit: float) -> Tuple[float, float]:
    """Limit a two-dimensional vector while preserving its direction."""
    magnitude = math.hypot(x, y)
    if magnitude <= limit or magnitude == 0.0:
        return x, y
    scale = limit / magnitude
    return x * scale, y * scale


def world_to_body(vx: float, vy: float, body_yaw: float) -> Tuple[float, float]:
    """Rotate a world-frame velocity into a robot base frame."""
    c = math.cos(body_yaw)
    s = math.sin(body_yaw)
    return c * vx + s * vy, -s * vx + c * vy


def rigid_body_commands(
    robot1: Pose2,
    robot2: Pose2,
    goal: Pose2,
    position_kp: float,
    yaw_kp: float,
    heading_kp: float,
    max_linear_speed: float,
    max_angular_speed: float,
    feedback_pose: Optional[Pose2] = None,
    alignment_error: Optional[Tuple[float, float]] = None,
    alignment_kp: float = 0.0,
    alignment_max_fraction: float = 1.0,
    center_velocity_override: Optional[Tuple[float, float]] = None,
    separation_target: Optional[float] = None,
    separation_kp: float = 0.0,
    separation_max_correction_speed: float = 0.0,
) -> Tuple[Twist2, Twist2, Pose2]:
    """
    Compute synchronized base commands from one payload goal.

    The object-level velocity is shared. Each base additionally receives the
    tangential velocity omega cross r required by a rigid-body turn. Commands
    are then transformed into the local frame of each mecanum base.
    """
    formation = estimate_payload_pose(robot1, robot2)
    feedback = feedback_pose or formation
    center_vx = position_kp * (goal.x - feedback.x)
    center_vy = position_kp * (goal.y - feedback.y)
    if alignment_error is not None:
        correction_x = alignment_kp * alignment_error[0]
        correction_y = alignment_kp * alignment_error[1]
        correction_limit = max(0.0, alignment_max_fraction) * math.hypot(
            center_vx, center_vy)
        correction_x, correction_y = clamp_vector(
            correction_x, correction_y, correction_limit)
        center_vx += correction_x
        center_vy += correction_y
    if center_velocity_override is not None:
        center_vx, center_vy = center_velocity_override
    center_vx, center_vy = clamp_vector(center_vx, center_vy, max_linear_speed)
    formation_omega = clamp(
        yaw_kp * normalize_angle(goal.yaw - formation.yaw),
        max_angular_speed,
    )
    feedback_omega = clamp(
        yaw_kp * normalize_angle(goal.yaw - feedback.yaw),
        max_angular_speed,
    )

    separation_correction = 0.0
    separation_direction = (0.0, 0.0)
    separation = math.hypot(
        robot2.x - robot1.x, robot2.y - robot1.y)
    if (separation_target is not None and separation > 1.0e-9
            and separation_kp > 0.0
            and separation_max_correction_speed > 0.0):
        # A positive error means the bases are too far apart.  Apply equal
        # and opposite world-frame velocities so the payload centre is not
        # translated by the separation correction itself.
        separation_correction = clamp(
            0.5 * separation_kp * (separation - separation_target),
            separation_max_correction_speed)
        separation_direction = (
            (robot2.x - robot1.x) / separation,
            (robot2.y - robot1.y) / separation,
        )

    commands = []
    for index, robot in enumerate((robot1, robot2)):
        rx = robot.x - formation.x
        ry = robot.y - formation.y
        world_vx = center_vx - formation_omega * ry
        world_vy = center_vy + formation_omega * rx
        direction = 1.0 if index == 0 else -1.0
        world_vx += direction * separation_correction * separation_direction[0]
        world_vy += direction * separation_correction * separation_direction[1]
        world_vx, world_vy = clamp_vector(world_vx, world_vy, max_linear_speed)
        local_vx, local_vy = world_to_body(world_vx, world_vy, robot.yaw)

        desired_heading = feedback.yaw + (math.pi if index == 1 else 0.0)
        heading_error = normalize_angle(desired_heading - robot.yaw)
        local_omega = clamp(
            feedback_omega + heading_kp * heading_error,
            max_angular_speed,
        )
        commands.append(Twist2(local_vx, local_vy, local_omega))

    return commands[0], commands[1], formation


def rate_limit(previous: Twist2, target: Twist2, dt: float,
               max_linear_acceleration: float,
               max_angular_acceleration: float) -> Twist2:
    """Apply planar linear-vector and angular acceleration limits."""
    dvx = target.x - previous.x
    dvy = target.y - previous.y
    dvx, dvy = clamp_vector(dvx, dvy, max_linear_acceleration * dt)
    domega = clamp(
        target.yaw - previous.yaw,
        max_angular_acceleration * dt,
    )
    return Twist2(
        previous.x + dvx,
        previous.y + dvy,
        previous.yaw + domega,
    )
