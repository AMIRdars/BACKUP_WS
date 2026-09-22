"""Planar clearance calculations for cooperative-transport evaluation."""

import math
from typing import Iterable, NamedTuple, Sequence, Tuple

from .kinematics import Pose2


class CircularObstacle(NamedTuple):
    """Circular obstacle in the world frame."""

    name: str
    x: float
    y: float
    radius: float


def circle_clearance(
        pose: Pose2, body_radius: float,
        obstacle: CircularObstacle) -> float:
    """Return signed clearance between two planar circles."""
    center_distance = math.hypot(
        pose.x - obstacle.x, pose.y - obstacle.y)
    return center_distance - body_radius - obstacle.radius


def oriented_box_clearance(
        pose: Pose2, length: float, width: float,
        obstacle: CircularObstacle) -> float:
    """Return signed clearance from a circle to an oriented rectangle."""
    dx = obstacle.x - pose.x
    dy = obstacle.y - pose.y
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    outside_x = max(abs(local_x) - 0.5 * length, 0.0)
    outside_y = max(abs(local_y) - 0.5 * width, 0.0)
    return math.hypot(outside_x, outside_y) - obstacle.radius


def minimum_system_clearance(
        payload: Pose2, robots: Sequence[Pose2],
        obstacles: Iterable[CircularObstacle], payload_length: float,
        payload_width: float, robot_radius: float) -> Tuple[float, str]:
    """Return minimum approximate clearance and the responsible body pair."""
    minimum = float('inf')
    pair = 'unavailable'
    for obstacle in obstacles:
        payload_clearance = oriented_box_clearance(
            payload, payload_length, payload_width, obstacle)
        if payload_clearance < minimum:
            minimum = payload_clearance
            pair = f'payload/{obstacle.name}'
        for index, robot in enumerate(robots, start=1):
            robot_clearance = circle_clearance(
                robot, robot_radius, obstacle)
            if robot_clearance < minimum:
                minimum = robot_clearance
                pair = f'robot{index}/{obstacle.name}'
    return minimum, pair
