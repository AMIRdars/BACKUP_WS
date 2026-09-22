"""Tests for planar obstacle-clearance calculations."""

import math

import pytest

from cooperative_transport_control.kinematics import Pose2
from cooperative_transport_control.obstacle_clearance import CircularObstacle
from cooperative_transport_control.obstacle_clearance import circle_clearance
from cooperative_transport_control.obstacle_clearance import minimum_system_clearance
from cooperative_transport_control.obstacle_clearance import oriented_box_clearance


def test_circle_clearance_is_signed():
    obstacle = CircularObstacle('post', 1.0, 0.0, 0.2)
    assert circle_clearance(Pose2(0.0, 0.0, 0.0), 0.3, obstacle) \
        == pytest.approx(0.5)
    assert circle_clearance(Pose2(0.7, 0.0, 0.0), 0.3, obstacle) \
        == pytest.approx(-0.2)


def test_oriented_payload_clearance_respects_yaw():
    obstacle = CircularObstacle('post', 0.0, 1.0, 0.2)
    horizontal = oriented_box_clearance(
        Pose2(0.0, 0.0, 0.0), 1.2, 0.1, obstacle)
    vertical = oriented_box_clearance(
        Pose2(0.0, 0.0, math.pi / 2.0), 1.2, 0.1, obstacle)
    assert horizontal == pytest.approx(0.75)
    assert vertical == pytest.approx(0.2)


def test_minimum_clearance_reports_body_pair():
    obstacles = [CircularObstacle('post', 2.0, 0.0, 0.2)]
    clearance, pair = minimum_system_clearance(
        Pose2(0.0, 0.0, 0.0),
        [Pose2(1.5, 0.0, 0.0), Pose2(-1.5, 0.0, math.pi)],
        obstacles, 1.2, 0.1, 0.2)
    assert clearance == pytest.approx(0.1)
    assert pair == 'robot1/post'
