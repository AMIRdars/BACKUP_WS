"""CP-8 local geometric model and bounded weighted velocity allocation.

Columns are five arm joints, body vx, body vy, body wz. No world pose is
needed. Metric scales, regularization and acceleration limits are software
policies, never physical inertia or load ratings.
"""
import numpy as np
from .wrench import finite


def base_jacobian(position):
    p = finite(position, (3,))
    base = np.zeros((6, 3))
    base[0, 0] = base[1, 1] = base[5, 2] = 1.
    base[:3, 2] = np.cross([0., 0., 1.], p)
    return base


def whole_jacobian(model, q, *, at_sensor=False):
    tcp, sensor = model.frames(q)
    p = sensor[:3, 3] if at_sensor else tcp
    arm = model.jacobian(q).copy()
    if at_sensor:
        arm[:3] += np.cross(arm[3:].T, p-tcp).T
    return np.c_[arm, base_jacobian(p)]


def box_least_squares(a, b, lower, upper, diagnostics=None):
    """Strictly convex bounded LS by a primal active set, with KKT check.

    The caller appends positive regularization rows. Clipping an unconstrained
    inverse alone does not solve this problem when columns are coupled.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.ndim != 2 or a.shape[1] == 0:
        raise ValueError('invalid bounded least squares matrix')
    n = a.shape[1]
    lower, upper = finite(lower, (n,)), finite(upper, (n,))
    if (a.ndim != 2 or b.shape != (a.shape[0],) or not np.isfinite(a).all()
            or not np.isfinite(b).all() or np.any(lower > upper)):
        raise ValueError('invalid bounded least squares inputs')
    h, f = a.T@a, a.T@b
    if np.linalg.eigvalsh(h)[0] <= 0:
        raise ValueError('positive regularization required')
    x = np.clip(np.linalg.solve(h, f), lower, upper)
    fixed = upper-lower < 1e-14
    active = fixed | (x <= lower+1e-14) | (x >= upper-1e-14)
    for iteration in range(256):
        free = ~active
        if free.any():
            trial = x.copy()
            trial[free] = np.linalg.solve(h[np.ix_(free, free)],
                f[free]-h[np.ix_(free, active)]@x[active])
            direction = trial-x
            alpha = 1.
            for i in np.flatnonzero(free):
                if direction[i] > 0: alpha = min(alpha, (upper[i]-x[i])/direction[i])
                if direction[i] < 0: alpha = min(alpha, (lower[i]-x[i])/direction[i])
            x = np.clip(x+max(0., alpha)*direction, lower, upper)
            if alpha < 1.-1e-12:
                active |= (x <= lower+1e-14) | (x >= upper-1e-14)
                continue
        gradient = h@x-f
        violation = np.zeros(n)
        at_lower = active & ~fixed & (x <= lower+1e-14)
        at_upper = active & ~fixed & (x >= upper-1e-14)
        violation[at_lower] = np.maximum(0., -gradient[at_lower])
        violation[at_upper] = np.maximum(0., gradient[at_upper])
        if np.max(violation) <= 1e-11:
            if free.any() and np.max(np.abs(gradient[free])) > 1e-9:
                raise ValueError('bounded allocation KKT residual')
            if diagnostics is not None: diagnostics['iterations'] = iteration+1
            return x
        active[np.argmax(violation)] = False
    raise ValueError('bounded allocation did not converge')


class WholeBodyAllocator:
    def __init__(self, model, *, margin=.01, arm_speed=.03, arm_acceleration=.15,
                 base_speed=(.0005, .0005, .001), base_acceleration=(.001, .001, .002), base_filter_hz=0.,
                 base_cost_velocity=None, task_metric=(1., 1., 1., .1, .1, .1), base_regularization=.00003,
                 lock_base_yaw=False):
        if type(lock_base_yaw) is not bool:raise ValueError('explicit base yaw lock boolean required')
        self.lock_base_yaw=lock_base_yaw
        self.model, self.margin = model, float(margin)
        self.scale = np.r_[np.minimum(model.velocity, arm_speed), finite(base_speed, (3,))]
        self.acceleration = np.r_[np.full(5, arm_acceleration), finite(base_acceleration, (3,))]
        if (not np.isfinite(self.margin) or self.margin <= 0 or not np.isfinite(self.scale).all()
                or not np.isfinite(self.acceleration).all()
                or np.any(self.scale <= 0) or np.any(self.acceleration <= 0)):
            raise ValueError('positive finite allocation policies required')
        self.previous = np.zeros(8)
        self.task_metric = finite(task_metric, (6,)).copy()
        if np.any(self.task_metric <= 0.):
            raise ValueError('positive task metric required')
        self.base_regularization = float(base_regularization)
        if not np.isfinite(self.base_regularization) or self.base_regularization <= 0.:
            raise ValueError('positive base preference regularization required')
        self.base_filter_hz = float(base_filter_hz)
        if not np.isfinite(self.base_filter_hz) or self.base_filter_hz < 0:
            raise ValueError('nonnegative base filter cutoff required')
        self.last = {}
        # Cost normalization is independent of the admissible speed envelope.
        # Widening a bound must not make fast arm tasks effectively free on the
        # base. None preserves the former dimensionless costs exactly.
        self.base_cost_velocity = self.scale[5:].copy() if base_cost_velocity is None else finite(base_cost_velocity, (3,))
        if np.any(self.base_cost_velocity <= 0):
            raise ValueError('positive base cost velocity scales required')
        self.decision_scale = self.scale.copy()
        self.decision_scale[5:] = self.base_cost_velocity

    def step(self, measured, requested, task_velocity, base_preference, slow_budget, dt):
        q, qr = finite(measured, (5,)), finite(requested, (5,))
        desired, preferred = finite(task_velocity, (6,)), finite(base_preference, (3,))
        if not np.isfinite(dt) or not 0 < dt <= .05 or not np.isfinite(slow_budget) or slow_budget < 0:
            raise ValueError('allocation timestep/budget outside local envelope')
        lo, hi = self.model.lower+self.margin, self.model.upper-self.margin
        if np.any(q <= lo) or np.any(q >= hi) or np.any(qr < lo) or np.any(qr > hi):
            raise ValueError('allocation reached software joint margin')
        j = whole_jacobian(self.model, q)
        metric = self.task_metric
        arm_singular = np.linalg.svd(metric[:, None]*j[:, :5], compute_uv=False)
        available=j[:,:7] if self.lock_base_yaw else j
        whole_singular = np.linalg.svd(metric[:, None]*available, compute_uv=False)
        if whole_singular[-1] < 1e-5:
            raise ValueError('whole body lost local task rank')
        fraction = np.minimum(q-lo, hi-q)/(hi-lo)
        # Continuous costs rise near a joint boundary and near arm singularity.
        weights = (1.+(.08/np.maximum(fraction, 1e-6))**2)*(1.+(.02/max(arm_singular[-1], 1e-9))**2)
        lower = np.maximum(-self.scale, self.previous-self.acceleration*dt)
        upper = np.minimum(self.scale, self.previous+self.acceleration*dt)
        lower[:5] = np.maximum(lower[:5], (lo-qr)/dt)
        upper[:5] = np.minimum(upper[:5], (hi-qr)/dt)
        # Only the low-pass measured deflection grants base authority. A new
        # force sample cannot directly drive the base. Permit bounded braking
        # when this authority shrinks, preserving acceleration constraints.
        credit = self.scale[5:]*min(1., slow_budget)
        credit = np.maximum(credit, np.maximum(0., np.abs(self.previous[5:])-self.acceleration[5:]*dt))
        lower[5:] = np.maximum(lower[5:], -credit)
        upper[5:] = np.minimum(upper[5:], credit)
        if self.lock_base_yaw:lower[7]=upper[7]=0.
        if np.any(lower > upper+1e-14):
            raise ValueError('joint viability and acceleration bounds conflict')
        lower = np.minimum(lower, upper)
        # Numerical units stay independent of admissible speed. A 1 m/s bound
        # must not inflate the Hessian by 10,000x relative to a 0.01 m/s task
        # scale. Bounds still use the actual velocity envelope above.
        task = metric[:, None]*j*self.decision_scale[None, :]/.001
        target = metric*desired/.001
        regularization = np.r_[.000003*weights, np.full(3, self.base_regularization)]
        prior = np.r_[np.zeros(5), preferred/self.decision_scale[5:]]
        a = np.vstack((task, np.diag(np.sqrt(regularization))))
        b = np.r_[target, np.sqrt(regularization)*prior]
        diagnostics = {}
        z = box_least_squares(a, b, lower/self.decision_scale, upper/self.decision_scale, diagnostics)
        candidate = z[5:]*self.decision_scale[5:]
        if self.base_filter_hz > 0:
            # Granting a slow amplitude envelope alone does not prevent the
            # fast task from switching base direction inside that envelope.
            # Filter the actual base velocity, preserve hard braking bounds,
            # then SOLVE AGAIN with that base fixed. Never just post-filter a
            # coupled inverse while leaving its arm solution inconsistent.
            alpha = 1.-np.exp(-2*np.pi*self.base_filter_hz*dt)
            slow = self.previous[5:]+alpha*(candidate-self.previous[5:])
            slow = np.clip(slow, lower[5:], upper[5:])
            lower[5:] = upper[5:] = slow
            first_iterations = diagnostics['iterations']
            z = box_least_squares(a, b, lower/self.decision_scale, upper/self.decision_scale, diagnostics)
            diagnostics['candidate_iterations'] = first_iterations
        self.previous = z*self.decision_scale
        self.last = {**diagnostics, 'arm_weights': weights.tolist(), 'arm_min_singular': float(arm_singular[-1]),
            'whole_min_singular': float(whole_singular[-1]), 'joint_margin_rad': float(np.min(np.minimum(q-lo, hi-q))),
            'residual_twist': (j@self.previous-desired).tolist(),
            'velocity': self.previous.tolist(), 'base_credit': credit.tolist(),
            'candidate_base_velocity': candidate.tolist(), 'base_filter_hz': self.base_filter_hz,
            'base_regularization': self.base_regularization,
            'active_bounds': ((abs(self.previous-lower)<1e-10)|(abs(self.previous-upper)<1e-10)).tolist()}
        if self.lock_base_yaw:self.last['base_yaw_locked']=True
        return self.previous.copy()
