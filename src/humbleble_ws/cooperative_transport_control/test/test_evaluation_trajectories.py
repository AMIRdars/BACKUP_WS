"""Tests for cooperative-transport evaluation trajectories."""

import math

import pytest

from cooperative_transport_control.evaluation_trajectories import Waypoint
from cooperative_transport_control.evaluation_trajectories import scenario_waypoints
from cooperative_transport_control.evaluation_trajectories import transform_waypoints


def test_turn_30_ends_at_thirty_degrees():
    waypoints = scenario_waypoints('turn_30')
    assert len(waypoints) == 6
    assert waypoints[-1].yaw == pytest.approx(math.radians(30.0))


def test_turn_90_uses_five_degree_increments():
    waypoints = scenario_waypoints('turn_90')
    assert len(waypoints) == 18
    yaws = [0.0] + [waypoint.yaw for waypoint in waypoints]
    assert max(abs(second - first) for first, second in zip(yaws, yaws[1:])) \
        == pytest.approx(math.radians(5.0))


def test_slalom_has_no_discontinuous_heading_reversal():
    waypoints = scenario_waypoints('slalom_short')
    yaws = [0.0] + [waypoint.yaw for waypoint in waypoints]
    assert max(abs(second - first) for first, second in zip(yaws, yaws[1:])) \
        <= math.radians(4.0) + 1.0e-12


def test_obstacle_slalom_clears_complete_field_with_smooth_headings():
    waypoints = scenario_waypoints('slalom_obstacles')
    assert len(waypoints) == 48
    assert waypoints[-1] == pytest.approx(Waypoint(6.0, 0.0, 0.0))
    # Four samples (0.5 m) provide straight entry and exit sections.
    assert all(waypoint.y == pytest.approx(0.0) for waypoint in waypoints[:4])
    assert all(waypoint.y == pytest.approx(0.0) for waypoint in waypoints[-4:])
    assert min(waypoint.y for waypoint in waypoints) == pytest.approx(-0.55)
    assert max(waypoint.y for waypoint in waypoints) == pytest.approx(0.55)
    assert waypoints[23].y == pytest.approx(0.0)
    yaws = [0.0] + [waypoint.yaw for waypoint in waypoints]
    assert max(abs(second - first) for first, second in zip(yaws, yaws[1:])) \
        < math.radians(8.0)


def test_open_slalom_has_full_metre_s_curve_and_straight_ends():
    waypoints = scenario_waypoints('slalom_open')
    assert len(waypoints) == 64
    assert waypoints[-1] == pytest.approx(Waypoint(8.0, 0.0, 0.0))
    assert all(waypoint.y == pytest.approx(0.0) for waypoint in waypoints[:4])
    assert all(waypoint.y == pytest.approx(0.0) for waypoint in waypoints[-4:])
    assert waypoints[17].y == pytest.approx(-1.0)
    assert waypoints[31].y == pytest.approx(0.0)
    assert waypoints[45].y == pytest.approx(1.0)
    yaws = [0.0] + [waypoint.yaw for waypoint in waypoints]
    assert max(abs(second - first) for first, second in zip(yaws, yaws[1:])) \
        < math.radians(2.0)


def test_transform_rotates_offsets_into_world():
    transformed = transform_waypoints(
        [Waypoint(1.0, 0.0, 0.2)],
        Waypoint(2.0, 3.0, math.pi / 2.0),
    )
    assert transformed[0].x == pytest.approx(2.0)
    assert transformed[0].y == pytest.approx(4.0)
    assert transformed[0].yaw == pytest.approx(math.pi / 2.0 + 0.2)


def test_unknown_scenario_is_rejected():
    with pytest.raises(ValueError):
        scenario_waypoints('not_a_scenario')
