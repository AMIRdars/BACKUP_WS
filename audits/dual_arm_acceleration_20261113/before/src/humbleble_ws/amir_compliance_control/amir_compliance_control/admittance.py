"""Bounded virtual x/y mechanics and xyz DLS, independent of ROS/evaluation."""
import numpy as np
from .wrench import finite


class PlanarAdmittance:
    def __init__(self, mass, damping, stiffness, displacement, velocity, acceleration):
        self.mass, self.damping, self.stiffness, self.limit, self.vmax, self.amax = (
            finite(v, (2,)) for v in (mass, damping, stiffness, displacement, velocity, acceleration))
        if (np.any(self.stiffness < 0) or any(np.any(v <= 0)
                for v in (self.mass, self.damping, self.limit, self.vmax, self.amax))):
            raise ValueError('positive virtual mechanics and motion bounds required')
        self.x, self.v = np.zeros(2), np.zeros(2)
        self.saturated = np.zeros(2, dtype=bool)

    def step(self, force, dt):
        force = finite(force, (2,))
        if not np.isfinite(dt) or not 0 < dt <= .05:
            raise ValueError('admittance timestep outside watchdog interval')
        # Implicit Euler with small substeps; bounded velocity and acceleration.
        count = max(1, int(np.ceil(dt/.005)))
        h = dt/count
        self.saturated[:] = False
        for _ in range(count):
            desired = (self.mass*self.v+h*(force-self.stiffness*self.x))/(
                self.mass+h*self.damping+h*h*self.stiffness)
            v = np.clip(desired, self.v-self.amax*h, self.v+self.amax*h)
            v = np.clip(v, -self.vmax, self.vmax)
            proposed = self.x+h*v
            x = np.clip(proposed, -self.limit, self.limit)
            hit = proposed != x
            # No hidden integrator beyond the workspace boundary.
            v[hit] = 0.
            self.saturated |= hit | (np.abs(desired) > self.vmax)
            self.x, self.v = x, v
        return self.x.copy()


def solve_xyz(model, seed, target, *, margin=.01, damping=.001, tolerance=1e-7):
    """Position-only local DLS. Orientation is intentionally unconstrained."""
    q, target = finite(seed, (5,)).copy(), finite(target, (3,))
    if (not np.isfinite(margin) or margin <= 0 or not np.isfinite(damping) or damping <= 0
            or not np.isfinite(tolerance) or tolerance <= 0):
        raise ValueError('positive IK policies required')
    lower, upper = model.lower+margin, model.upper-margin
    if np.any(lower >= upper) or np.any(q <= lower) or np.any(q >= upper):
        raise ValueError('IK seed reached software joint margin')
    for _ in range(40):
        error = target-model.frames(q)[0]
        j = model.jacobian(q)[:3]
        if np.linalg.svd(j, compute_uv=False)[-1] < .001:
            raise ValueError('xyz task lost local rank')
        if np.linalg.norm(error) <= tolerance:
            return q
        step = j.T@np.linalg.solve(j@j.T+damping*damping*np.eye(3), error)
        peak = np.max(np.abs(step))
        if peak > .01:
            step *= .01/peak
        q = np.clip(q+step, lower+1e-9, upper-1e-9)
    raise ValueError('DLS cannot reach bounded xyz target')


def rotation_error(target, current):
    """SO(3) log in base axes; avoids Euler singularities at the audited home."""
    r = finite(target, (3, 3))@finite(current, (3, 3)).T
    theta = float(np.arccos(np.clip((np.trace(r)-1)/2, -1., 1.)))
    skew = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    if theta < 1e-6:
        return .5*skew
    if theta > np.pi-.01:
        raise ValueError('rotation error outside local orientation chart')
    return theta/(2*np.sin(theta))*skew


def solve_pose(model, seed, target, rotation, *, margin=.01, damping=.001):
    """Local pose DLS: verifies residuals, never assumes a six-DOF arm."""
    q, target = finite(seed, (5,)).copy(), finite(target, (3,))
    lower, upper = model.lower+margin, model.upper-margin
    if np.any(q <= lower) or np.any(q >= upper):
        raise ValueError('pose IK seed reached software margin')
    for _ in range(80):
        p, sensor = model.frames(q)
        ep, er = target-p, rotation_error(rotation, sensor[:3, :3])
        if np.linalg.norm(ep) < 1e-7 and np.linalg.norm(er) < 1e-6:
            return q
        # .1 m is a task weighting scale, not an asserted robot dimension.
        j = model.jacobian(q)*np.array([1., 1., 1., .1, .1, .1])[:, None]
        error = np.r_[ep, .1*er]
        step = j.T@np.linalg.solve(j@j.T+damping*damping*np.eye(6), error)
        step *= min(1., .01/max(np.max(np.abs(step)), 1e-12))
        q = np.clip(q+step, lower+1e-9, upper-1e-9)
    raise ValueError('pose target is not reachable with audited arm freedoms')
