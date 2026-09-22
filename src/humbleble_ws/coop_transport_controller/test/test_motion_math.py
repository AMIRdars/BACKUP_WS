import math

import pytest

from coop_transport_controller.motion_math import (
    Pose2, arm_grasp_position, arm_tcp_position, grasp_world_position,
    normalize_angle, pivot_velocity, resolve_target_yaw, rotate_point_about_pivot)


def test_angle_normalization_crosses_pi_by_shortest_path():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)


def test_relative_target_yaw_is_resolved_once_from_start():
    assert resolve_target_yaw(3.0, 1, 0.0, 0.5, 0) == pytest.approx(-2.78318530718)


def test_pivot_endpoint_is_on_circle():
    x, y = rotate_point_about_pivot(Pose2(1.0, 0.0, 0.0), 0.0, 0.0, math.pi / 2.0)
    assert (x, y) == pytest.approx((0.0, 1.0))


def test_positive_pivot_rotation_has_positive_y_tangent():
    vx, vy, wz = pivot_velocity(
        Pose2(1.0, 0.0, 0.0), 0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 1.0, 0.2)
    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(0.2)
    assert wz == pytest.approx(0.2)


def test_amir_target_posture_is_inside_fk_and_changes_horizontal_tcp_position():
    initial = arm_tcp_position((0.0, 0.0, 0.0, 0.0, 0.0))
    target = arm_tcp_position((0.0, 1.2, -1.5, 0.3, 1.5))
    assert initial == pytest.approx((0.738477, 0.0, 0.0), abs=1.0e-6)
    assert target[0] == pytest.approx(0.526962, abs=1.0e-6)
    assert target[1] == pytest.approx(0.0, abs=1.0e-6)
    assert target[2] == pytest.approx(0.197321, abs=1.0e-6)


def test_tcp_world_position_respects_opposite_amir_headings():
    joints = (0.0, 0.0, 0.0, 0.0, 0.0)
    first = grasp_world_position(Pose2(-1.34, 0.0, 0.0), joints)
    second = grasp_world_position(Pose2(1.34, 0.0, math.pi), joints)
    assert first[0] < -0.5
    assert second[0] > 0.5
    assert first[1] == pytest.approx(second[1], abs=1.0e-9)


def test_grasp_fk_excludes_tcp_marker_offset():
    grasp = arm_grasp_position((0.0, 0.0, 0.0, 0.0, 0.0))
    tcp = arm_tcp_position((0.0, 0.0, 0.0, 0.0, 0.0))
    assert grasp[0] == pytest.approx(0.658977, abs=1.0e-6)
    assert tcp[0] == pytest.approx(grasp[0] + 0.0795, abs=1.0e-6)
