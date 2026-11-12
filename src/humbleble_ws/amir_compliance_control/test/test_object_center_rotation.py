"""Section 21 geometry: independent signs, derivatives and frame covariance."""
import math

import numpy as np
import pytest

from amir_compliance_control.object_center_rotation_trajectory import (
    ObjectCenterRotation, initialize_object_pose, inverse_transform, quintic,
    rigid_transform, rotate_about_center, world_velocity_to_body, wrap_pi)
from amir_compliance_control.wrench import origin
import xml.etree.ElementTree as ET


def pose(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    return origin(ET.Element('origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy))))


@pytest.mark.parametrize('angle', [0., -5., 5., -15., 15., -30., 30., -90., 90.])
def test_quintic_endpoints_and_exact_peak_rate(angle):
    delta = math.radians(angle)
    for t in (-1., 0.):
        assert quintic(t, 40., delta) == (0., 0., 0.)
    for t in (40., 41.):
        assert quintic(t, 40., delta) == (delta, 0., 0.)
    p, v, a = quintic(20., 40., delta)
    assert p == pytest.approx(delta/2)
    assert v == pytest.approx(1.875*delta/40.)
    assert a == 0.


@pytest.mark.parametrize('time,duration,delta', [
    (0., 0., 1.), (0., -1., 1.), (0., np.inf, 1.), (np.nan, 1., 1.),
    (0., 1., np.nan), (0., 1., np.inf)])
def test_invalid_interpolation_rejected(time, duration, delta):
    with pytest.raises(ValueError):
        quintic(time, duration, delta)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_independent_quarter_circle_sign_and_nonzero_center_height(sign):
    center = np.array([2., -3., .7])
    np.testing.assert_allclose(rotate_about_center(center+[.6, 0., .2], center, sign*math.pi/2),
                               center+[0., sign*.6, .2], atol=1e-14)


def test_full_calibration_recovers_pose_in_arbitrary_leader_frame():
    rng = np.random.default_rng(810)
    for _ in range(30):
        wb = pose(rng.normal(size=3), [0., 0., rng.uniform(-math.pi, math.pi)])
        bg = pose(rng.normal(size=3), rng.uniform(-2., 2., 3))
        og = pose(rng.normal(size=3), rng.uniform(-2., 2., 3))
        wo = initialize_object_pose(wb, bg, og)
        np.testing.assert_allclose(wo @ og, wb @ bg, atol=3e-15)
        change = pose(rng.normal(size=3), rng.uniform(-2., 2., 3))
        np.testing.assert_allclose(initialize_object_pose(change@wb, bg, og), change@wo, atol=3e-15)


@pytest.mark.parametrize('degrees', [-90., -30., -15., -5., 5., 15., 30., 90.])
def test_radius_center_rigid_closure_orientation_and_derivatives(degrees):
    obj = pose([.3, -.2, .7], [0., 0., .4])
    base = pose([-1.1, .3, .02], [0., 0., 3.1])
    # Grasp pitch at gimbal lock: preserve the full matrix, never Euler yaw.
    bg = pose([.47, -.001, .66], [.2, math.pi/2, -.5])
    grasp = base @ bg
    original = [x.copy() for x in (obj, base, grasp)]
    trajectory = ObjectCenterRotation(obj, base, grasp, duration=40., delta_yaw_rad=math.radians(degrees))
    # Mutating caller-owned buffers must not change the initial latch.
    obj[:3, 3] += 20.
    ref0 = trajectory.sample(0.)
    for actual, expected in zip((ref0.object_pose, ref0.base_pose, ref0.grasp_pose), original):
        np.testing.assert_allclose(actual, expected, atol=2e-15)
    for t in np.linspace(.2, 39.8, 31):
        ref = trajectory.sample(t)
        np.testing.assert_allclose(ref.object_pose[:3, 3], original[0][:3, 3], atol=1e-15)
        np.testing.assert_allclose(ref.base_pose @ bg, ref.grasp_pose, atol=2e-15)
        np.testing.assert_allclose(inverse_transform(ref.object_pose)@ref.grasp_pose,
                                   inverse_transform(original[0])@original[2], atol=3e-15)
        for name in ('base', 'grasp'):
            point = getattr(ref, name+'_pose')[:3, 3]
            assert np.linalg.norm(point-original[0][:3, 3]) == pytest.approx(
                np.linalg.norm(original[1 if name == 'base' else 2][:3, 3]-original[0][:3, 3]))
            h = 2e-4
            before, after = trajectory.sample(t-h), trajectory.sample(t+h)
            numeric_v = (getattr(after, name+'_pose')[:3, 3]-getattr(before, name+'_pose')[:3, 3])/(2*h)
            numeric_a = (getattr(after, name+'_velocity_world')-getattr(before, name+'_velocity_world'))/(2*h)
            np.testing.assert_allclose(numeric_v, getattr(ref, name+'_velocity_world'), atol=4e-10)
            np.testing.assert_allclose(numeric_a, getattr(ref, name+'_acceleration_world'), atol=4e-11)
        assert ref.base_pose[2, 3] == pytest.approx(original[1][2, 3], abs=1e-15)
        assert ref.grasp_pose[2, 3] == pytest.approx(original[2][2, 3], abs=1e-15)
    end = trajectory.sample(40.)
    assert end.base_yaw_unwrapped == pytest.approx(3.1+math.radians(degrees))
    assert not end.base_velocity_world.any() and not end.grasp_velocity_world.any()


def test_body_conversion_uses_actual_orientation_and_wrap_crosses_pi():
    np.testing.assert_allclose(world_velocity_to_body([1., 0., 0.], pose(rpy=[0., 0., math.pi/2])[:3, :3]),
                               [0., -1., 0.], atol=1e-15)
    assert wrap_pi(math.radians(181)) == pytest.approx(math.radians(-179))
    assert wrap_pi(math.radians(-181)) == pytest.approx(math.radians(179))
    assert wrap_pi(3*math.pi) == pytest.approx(-math.pi)
    with pytest.raises(ValueError):
        wrap_pi(np.nan)


@pytest.mark.parametrize('bad', [np.zeros((4, 4)), np.diag([-1., 1., 1., 1.]),
                                 np.diag([2., 1., 1., 1.]), np.full((4, 4), np.nan), np.eye(3)])
def test_nonrigid_or_incomplete_calibration_rejected(bad):
    with pytest.raises(ValueError):
        rigid_transform(bad)
    with pytest.raises(ValueError):
        initialize_object_pose(np.eye(4), np.eye(4), bad)


def test_horizontal_task_rejects_tilted_object_or_base():
    for object0, base0 in ((pose(rpy=[.01, 0., 0.]), np.eye(4)),
                            (np.eye(4), pose(rpy=[0., .01, 0.]))):
        with pytest.raises(ValueError, match='upright'):
            ObjectCenterRotation(object0, base0, np.eye(4), duration=40., delta_yaw_rad=.1)
