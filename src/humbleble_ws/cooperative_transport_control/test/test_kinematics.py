import math

import pytest

from cooperative_transport_control.kinematics import (
    Pose2,
    Twist2,
    estimate_payload_pose,
    normalize_angle,
    rate_limit,
    rigid_body_commands,
    world_to_body,
)


def test_payload_pose_comes_from_base_midpoint_and_axis():
    payload = estimate_payload_pose(Pose2(-1.34, 0.0, 0.0), Pose2(1.34, 0.0, math.pi))
    assert payload.x == pytest.approx(0.0)
    assert payload.y == pytest.approx(0.0)
    assert payload.yaw == pytest.approx(0.0)


def test_world_velocity_is_rotated_for_opposed_robot():
    assert world_to_body(0.05, 0.0, 0.0) == pytest.approx((0.05, 0.0))
    assert world_to_body(0.05, 0.0, math.pi) == pytest.approx((-0.05, 0.0))


def test_straight_goal_generates_same_world_direction():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.34, 0.0, 0.0), Pose2(1.34, 0.0, math.pi),
        Pose2(1.0, 0.0, 0.0), 0.5, 0.8, 0.3, 0.08, 0.10)
    assert command1.x == pytest.approx(0.08)
    assert command2.x == pytest.approx(-0.08)
    assert command1.y == pytest.approx(0.0, abs=1.0e-9)
    assert command2.y == pytest.approx(0.0, abs=1.0e-9)


def test_turn_adds_opposite_tangential_velocities():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.34, 0.0, 0.0), Pose2(1.34, 0.0, math.pi),
        Pose2(0.0, 0.0, 0.2), 0.5, 0.8, 0.3, 0.08, 0.10)
    assert command1.y < 0.0
    # Robot 2 local Y is opposite world Y because its heading is pi.
    assert command2.y < 0.0
    assert command1.yaw == pytest.approx(0.10)
    assert command2.yaw == pytest.approx(0.10)


def test_distance_control_applies_equal_opposite_closing_commands():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.40, 0.0, 0.0), Pose2(1.40, 0.0, math.pi),
        Pose2(0.0, 0.0, 0.0), 0.5, 0.8, 0.3, 0.20, 0.10,
        separation_target=2.68, separation_kp=1.0,
        separation_max_correction_speed=0.10)
    assert command1.x > 0.0
    assert command2.x > 0.0


def test_distance_control_opens_an_overcompressed_formation():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.30, 0.0, 0.0), Pose2(1.30, 0.0, math.pi),
        Pose2(0.0, 0.0, 0.0), 0.5, 0.8, 0.3, 0.20, 0.10,
        separation_target=2.68, separation_kp=1.0,
        separation_max_correction_speed=0.10)
    assert command1.x < 0.0
    assert command2.x < 0.0


def test_measured_payload_feedback_decouples_body_yaw_from_formation_yaw():
    command1, command2, formation = rigid_body_commands(
        Pose2(-1.34, 0.0, 0.2), Pose2(1.34, 0.0, math.pi + 0.2),
        Pose2(0.0, 0.0, 0.2), 0.5, 0.8, 0.3, 0.08, 0.10,
        feedback_pose=Pose2(0.0, 0.0, 0.2))
    # The base centers must still orbit until their connecting axis reaches
    # the goal, but already-aligned base headings need no further rotation.
    assert formation.yaw == pytest.approx(0.0)
    assert command1.y < 0.0
    assert command2.y < 0.0
    assert command1.yaw == pytest.approx(0.0, abs=1.0e-9)
    assert command2.yaw == pytest.approx(0.0, abs=1.0e-9)


def test_alignment_error_slows_a_formation_that_leads_the_payload():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.34, 0.0, 0.0), Pose2(1.34, 0.0, math.pi),
        Pose2(1.0, 0.0, 0.0), 0.02, 0.8, 0.3, 0.08, 0.10,
        alignment_error=(-0.01, 0.0), alignment_kp=1.0)
    assert command1.x == pytest.approx(0.01)
    assert command2.x == pytest.approx(-0.01)


def test_alignment_correction_cannot_cancel_goal_progress():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.34, 0.0, 0.0), Pose2(1.34, 0.0, math.pi),
        Pose2(1.0, 0.0, 0.0), 0.02, 0.8, 0.3, 0.08, 0.10,
        alignment_error=(-0.10, 0.0), alignment_kp=1.0,
        alignment_max_fraction=0.75)
    assert command1.x == pytest.approx(0.005)
    assert command2.x == pytest.approx(-0.005)


def test_recovery_velocity_can_temporarily_reverse_formation():
    command1, command2, _ = rigid_body_commands(
        Pose2(-1.34, 0.0, 0.0), Pose2(1.34, 0.0, math.pi),
        Pose2(1.0, 0.0, 0.0), 0.5, 0.8, 0.3, 0.08, 0.10,
        center_velocity_override=(-0.015, 0.0))
    assert command1.x == pytest.approx(-0.015)
    assert command2.x == pytest.approx(0.015)


def test_rate_limit_bounds_vector_and_angular_change():
    limited = rate_limit(
        Twist2(0.0, 0.0, 0.0), Twist2(1.0, 1.0, 1.0),
        0.1, 0.2, 0.3)
    assert math.hypot(limited.x, limited.y) == pytest.approx(0.02)
    assert limited.yaw == pytest.approx(0.03)


def test_normalize_angle_wraps():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
