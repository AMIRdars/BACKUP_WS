"""Chapter 24: measured-state XY velocity DLS. No base or peer input."""
import numpy as np
from .wrench import finite


def xy_velocity_dls(model, q, velocity, *, damping, singular_stop, margin, speed):
    q, velocity = finite(q, (5,)), finite(velocity, (2,))
    if any(not np.isfinite(v) or v <= 0 for v in (damping, singular_stop, margin, speed)):
        raise ValueError('positive DLS policies required')
    joint_margin = np.minimum(q-model.lower, model.upper-q)
    if np.min(joint_margin) <= margin:
        raise ValueError('ARM_ONLY joint software margin')
    j = finite(model.jacobian(q)[:2], (2, 5))
    sigma = float(np.linalg.svd(j, compute_uv=False)[-1])
    if sigma <= singular_stop:
        raise ValueError('ARM_ONLY XY task singularity')
    # Increase regularization continuously as the measured task approaches its
    # stop boundary; no orientation task or whole-body allocation is inserted.
    lam = damping*max(1., 4.*singular_stop/sigma)
    qdot = j.T@np.linalg.solve(j@j.T+lam**2*np.eye(2), velocity)
    limits = np.minimum(model.velocity, speed)
    qdot *= min(1., float(np.min(limits/np.maximum(np.abs(qdot), 1e-15))))
    return qdot, sigma, float(np.min(joint_margin)), lam


def nullspace_pose_velocity(model, q, primary, secondary_velocity, *, damping, angular_weight, speed):
    """Preserve XY direction; minimize z/rotation drift in its 3D nullspace.

    A four-component secondary request is generally not exactly achievable.
    Return its residual instead of asserting six-axis arm controllability.
    angular_weight is a task scaling in metres/radian, not a robot dimension.
    """
    q, primary = finite(q, (5,)), finite(primary, (5,))
    secondary_velocity = finite(secondary_velocity, (4,))
    if any(not np.isfinite(v) or v <= 0 for v in (damping, angular_weight, speed)):
        raise ValueError('positive nullspace policies required')
    j = finite(model.jacobian(q), (6, 5))
    _, _, vt = np.linalg.svd(j[:2], full_matrices=True)
    basis = vt[2:].T
    weight = np.array([1., angular_weight, angular_weight, angular_weight])
    a = (j[2:]@basis)*weight[:, None]
    error = (secondary_velocity-j[2:]@primary)*weight
    # Three unknown nullspace velocities, not a six-DOF pose inverse.
    coefficients = np.linalg.solve(a.T@a+damping**2*np.eye(3), a.T@error)
    correction = basis@coefficients
    result = primary+correction
    limits = np.minimum(model.velocity, speed)
    scale = min(1., float(np.min(limits/np.maximum(np.abs(result), 1e-15))))
    result *= scale
    return result, {'nullspace_xy_leak_m_s': (j[:2]@correction).tolist(),
        'secondary_velocity_residual': (j[2:]@result-secondary_velocity).tolist(),
        'joint_velocity_scale': scale}
