"""Waypoint definitions for cooperative-transport evaluation scenarios."""

import math
from typing import List, NamedTuple


class Waypoint(NamedTuple):
    """Planar waypoint."""

    x: float
    y: float
    yaw: float


def scenario_waypoints(name: str) -> List[Waypoint]:
    """Return relative waypoints for a named evaluation scenario."""
    if name == 'straight_0_5m':
        return [Waypoint(0.50, 0.0, 0.0)]
    if name in ('turn_30', 'turn_90'):
        final_degrees = 30 if name == 'turn_30' else 90
        radius = 0.60 if name == 'turn_30' else 0.80
        # Five-degree increments keep the friction grasp loaded smoothly for
        # both turns; 15-degree target jumps can turn a path-sampling artifact
        # into a slip fault before the bases have accelerated.
        step = 5
        result = []
        for degrees in range(step, final_degrees + 1, step):
            angle = math.radians(degrees)
            result.append(Waypoint(
                radius * math.sin(angle),
                radius * (1.0 - math.cos(angle)),
                angle,
            ))
        return result
    if name == 'slalom_short':
        return [
            # Keep adjacent heading changes small.  A four-point version made
            # the target jump directly from -8 to +8 degrees, which is not a
            # physically continuous trajectory and correctly tripped the
            # friction-grasp slip monitor during the reversal.
            Waypoint(0.06, 0.04, math.radians(4.0)),
            Waypoint(0.12, 0.08, math.radians(8.0)),
            Waypoint(0.16, 0.04, math.radians(4.0)),
            Waypoint(0.20, 0.00, math.radians(0.0)),
            Waypoint(0.24, -0.04, math.radians(-4.0)),
            Waypoint(0.28, -0.08, math.radians(-8.0)),
            Waypoint(0.32, -0.04, math.radians(-4.0)),
            Waypoint(0.36, 0.00, math.radians(0.0)),
            Waypoint(0.40, 0.04, math.radians(4.0)),
            Waypoint(0.44, 0.08, math.radians(8.0)),
            Waypoint(0.52, 0.04, math.radians(4.0)),
            Waypoint(0.60, 0.0, 0.0),
        ]
    if name == 'slalom_obstacles':
        # Use a visible S curve rather than a nearly straight centre-line
        # traversal.  A 0.5 m straight lead-in is followed by negative and
        # positive sin^2 lobes and a 0.5 m straight exit.  Both lobe joins have
        # zero lateral slope, avoiding a discontinuous command at the centre
        # reversal while the complete long formation passes the cylinders.
        amplitude = 0.55
        lead_in = 0.50
        lobe_length = 2.50
        curve_end = lead_in + 2.0 * lobe_length
        result = []
        for index in range(1, 49):
            x = 0.125 * index
            if x <= lead_in or x >= curve_end:
                y = 0.0
                slope = 0.0
            elif x <= lead_in + lobe_length:
                phase = math.pi * (x - lead_in) / lobe_length
                y = -amplitude * math.sin(phase) ** 2
                slope = -(amplitude * math.pi / lobe_length) * math.sin(
                    2.0 * phase)
            else:
                phase = math.pi * (
                    x - lead_in - lobe_length) / lobe_length
                y = amplitude * math.sin(phase) ** 2
                slope = (amplitude * math.pi / lobe_length) * math.sin(
                    2.0 * phase)
            # Mecanum lateral motion supplies the weave.  The long payload is
            # kept nearly parallel to the corridor because large tangent-yaw
            # tracking twists the opposed friction grasps vertically.  The
            # small continuous yaw target still indicates curve direction.
            result.append(Waypoint(x, y, 0.08 * math.atan(slope)))
        return result
    if name == 'slalom_open':
        # Large open-field S curve: 0.5 m entry, two 3.5 m sin^2 lobes and a
        # 0.5 m exit.  A full metre of lateral travel makes the weave clearly
        # visible while the long lobe length keeps curvature continuous.
        amplitude = 1.0
        lead_in = 0.50
        lobe_length = 3.50
        curve_end = lead_in + 2.0 * lobe_length
        result = []
        for index in range(1, 65):
            x = 0.125 * index
            if x <= lead_in or x >= curve_end:
                y = 0.0
                slope = 0.0
            elif x <= lead_in + lobe_length:
                phase = math.pi * (x - lead_in) / lobe_length
                y = -amplitude * math.sin(phase) ** 2
                slope = -(amplitude * math.pi / lobe_length) * math.sin(
                    2.0 * phase)
            else:
                phase = math.pi * (
                    x - lead_in - lobe_length) / lobe_length
                y = amplitude * math.sin(phase) ** 2
                slope = (amplitude * math.pi / lobe_length) * math.sin(
                    2.0 * phase)
            result.append(Waypoint(x, y, 0.08 * math.atan(slope)))
        return result
    raise ValueError(
        f'Unknown scenario {name!r}; choose straight_0_5m, turn_30, '
        'turn_90, slalom_short, slalom_obstacles, or slalom_open.')


def transform_waypoints(
        waypoints: List[Waypoint], origin: Waypoint) -> List[Waypoint]:
    """Transform relative waypoints into the world frame."""
    cosine = math.cos(origin.yaw)
    sine = math.sin(origin.yaw)
    result = []
    for waypoint in waypoints:
        result.append(Waypoint(
            origin.x + cosine * waypoint.x - sine * waypoint.y,
            origin.y + sine * waypoint.x + cosine * waypoint.y,
            origin.yaw + waypoint.yaw,
        ))
    return result
