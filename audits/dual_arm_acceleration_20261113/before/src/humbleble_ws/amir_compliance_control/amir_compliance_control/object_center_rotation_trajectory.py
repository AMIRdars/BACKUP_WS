"""CP-8R-0 geometry only, in one continuous leader-local planning frame.

Transforms use parent_T_child, metres and radians. G must be the SAME physical
frame in FK and calibration (TCP and wrist anchor are not interchangeable).
There is no ROS, Gazebo, follower state, command publisher or default tuning.
"""
from dataclasses import dataclass
import math

import numpy as np

from .wrench import finite, rotation


def rigid_transform(value):
    """Copy and validate an SE(3) transform; reject reflections and scale."""
    t = finite(value, (4, 4)).copy()
    r = t[:3, :3]
    if (not np.allclose(t[3], [0., 0., 0., 1.], atol=1e-10, rtol=0.)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-10, rtol=0.)
            or not math.isclose(np.linalg.det(r), 1., abs_tol=1e-10)):
        raise ValueError('proper rigid transform required')
    return t


def inverse_transform(value):
    t = rigid_transform(value)
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return result


def initialize_object_pose(leader_odom_T_base, base_T_grasp, object_T_grasp):
    """Section 21.2: leader odom/FK + a known FULL grasp calibration only.

    An SDF attachment-link pose alone is not object_T_grasp unless that link
    and the actual FK grasp frame coincide in both position AND orientation.
    """
    return (rigid_transform(leader_odom_T_base) @ rigid_transform(base_T_grasp)
            @ inverse_transform(object_T_grasp))


def wrap_pi(angle):
    if not math.isfinite(angle):
        raise ValueError('finite angle required')
    return (angle + math.pi) % (2. * math.pi) - math.pi


def quintic(t, duration, delta_yaw_rad):
    """Relative angle, angular velocity and acceleration; hold outside [0,T]."""
    if (not all(math.isfinite(v) for v in (t, duration, delta_yaw_rad))
            or duration <= 0.):
        raise ValueError('finite time/angle and positive duration required')
    if t <= 0.:
        return 0., 0., 0.
    if t >= duration:
        return float(delta_yaw_rad), 0., 0.
    u = t / duration
    return (delta_yaw_rad * u**3 * (10. - 15.*u + 6.*u**2),
            delta_yaw_rad / duration * 30.*u**2 * (1.-u)**2,
            delta_yaw_rad / duration**2 * 60.*u * (1.-u) * (1.-2.*u))


def rotate_about_center(point, center, phi):
    if not math.isfinite(phi):
        raise ValueError('finite angle required')
    p, c = finite(point, (3,)), finite(center, (3,))
    return c + rotation([0., 0., 1.], phi) @ (p-c)


def world_velocity_to_body(velocity, world_R_body):
    """Use the actual body's rotation for a later SE(2) tracking adapter."""
    t = np.eye(4)
    t[:3, :3] = finite(world_R_body, (3, 3))
    return rigid_transform(t)[:3, :3].T @ finite(velocity, (3,))


@dataclass(frozen=True)
class RotationReference:
    phi: float
    omega: float
    alpha: float
    object_pose: np.ndarray
    base_pose: np.ndarray
    base_yaw_unwrapped: float
    grasp_pose: np.ndarray
    base_velocity_world: np.ndarray
    base_acceleration_world: np.ndarray
    grasp_velocity_world: np.ndarray
    grasp_acceleration_world: np.ndarray


class ObjectCenterRotation:
    """Latched ideal rigid references, not a controller or a grasp guarantee.

    The base argument is the audited odometry control frame (base_footprint
    here). Reconstruct the physical base_link with its URDF fixed transform.
    Full grasp rotation is retained even at an Euler-angle singularity.
    """
    def __init__(self, object_pose0, base_pose0, grasp_pose0, *,
                 duration, delta_yaw_rad):
        quintic(0., duration, delta_yaw_rad)
        self._object0 = rigid_transform(object_pose0)
        self._base0 = rigid_transform(base_pose0)
        self._grasp0 = rigid_transform(grasp_pose0)
        for pose in (self._object0, self._base0):
            if not np.allclose(pose[:3, 2], [0., 0., 1.], atol=1e-9, rtol=0.):
                raise ValueError('object and base must have upright z axes')
        for pose in (self._object0, self._base0, self._grasp0):
            pose.setflags(write=False)
        self._duration, self._delta = float(duration), float(delta_yaw_rad)

    def sample(self, t):
        phi, omega, alpha = quintic(t, self._duration, self._delta)
        r = rotation([0., 0., 1.], phi)
        center = self._object0[:3, 3]

        def pose_at(pose0):
            pose = np.eye(4)
            pose[:3, :3] = r @ pose0[:3, :3]
            pose[:3, 3] = center + r @ (pose0[:3, 3]-center)
            return pose

        def derivatives(pose):
            radius = pose[:3, 3]-center
            tangent = np.cross([0., 0., 1.], radius)
            return (omega*tangent,
                    alpha*tangent + omega**2*np.cross([0., 0., 1.], tangent))

        obj, base, grasp = map(pose_at, (self._object0, self._base0, self._grasp0))
        bv, ba = derivatives(base)
        gv, ga = derivatives(grasp)
        yaw0 = math.atan2(self._base0[1, 0], self._base0[0, 0])
        return RotationReference(phi, omega, alpha, obj, base, yaw0+phi,
                                 grasp, bv, ba, gv, ga)
