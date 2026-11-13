"""CP-8R-2 local arm soft task and slow xy/yaw recentering, no ROS input.

Five arm joints cannot satisfy six independent pose constraints at the audited
home. The arm task is a bounded weighted fit with its residual exposed. There
are no base decision columns; whole-body allocation belongs to CP-8R-3.
"""
import numpy as np

from .whole_body import box_least_squares
from .wrench import finite


class ArmSoftTask:
    def __init__(self, model, *, metric, speed, acceleration, margin):
        self.model = model
        self.metric = finite(metric, (6,))
        self.speed = np.minimum(model.velocity, float(speed))
        self.acceleration, self.margin = float(acceleration), float(margin)
        if (np.any(self.metric <= 0.) or not np.isfinite(self.speed).all() or np.any(self.speed <= 0.)
                or not np.isfinite(self.acceleration) or self.acceleration <= 0.
                or not np.isfinite(self.margin) or self.margin <= 0.
                or np.any(model.lower+self.margin >= model.upper-self.margin)):
            raise ValueError('positive finite arm task policies required')
        self.previous = np.zeros(5)
        self.last = {}

    def step(self, measured, requested, desired_twist, dt):
        q, qr, v = finite(measured, (5,)), finite(requested, (5,)), finite(desired_twist, (6,))
        if not np.isfinite(dt) or not 0. < dt <= .05:
            raise ValueError('arm task timestep outside envelope')
        low, high = self.model.lower+self.margin, self.model.upper-self.margin
        if np.any(q <= low) or np.any(q >= high) or np.any(qr < low) or np.any(qr > high):
            raise ValueError('arm task reached software joint margin')
        j = self.model.jacobian(q)
        a = self.metric[:, None]*j
        singular = np.linalg.svd(a, compute_uv=False)
        if singular[-1] < .001:
            raise ValueError('arm soft task lost audited rank five')
        lower = np.maximum(-self.speed, self.previous-self.acceleration*dt)
        upper = np.minimum(self.speed, self.previous+self.acceleration*dt)
        lower = np.maximum(lower, (low-qr)/dt)
        upper = np.minimum(upper, (high-qr)/dt)
        if np.any(lower > upper):
            raise ValueError('joint viability conflicts with arm acceleration')
        # Positive regularization, rather than clipping a coupled inverse.
        velocity = box_least_squares(np.vstack((a, .001*np.eye(5))),
                                     np.r_[self.metric*v, np.zeros(5)], lower, upper)
        self.previous = velocity
        self.last = {'residual_twist': (j@velocity-v).tolist(),
                     'arm_min_metric_singular': float(singular[-1]),
                     'joint_margin_rad': float(np.min(np.r_[q-low, high-q]))}
        return velocity.copy()


class YawRecentering:
    """Measured local deflection + optional slow own-force velocity memory.

    Compensation uses the ACTUAL limited yaw command. Force memory is local,
    filtered, bounded, resettable, and has no leader trajectory or centre.
    """
    def __init__(self, *, cutoff, gains, velocity, acceleration,
                 force_gains=(0., 0., 0.), force_velocity_gains=(0., 0., 0.), force_cutoff=.1):
        self.gains, self.vmax, self.amax, self.force_gains, self.force_velocity_gains = (
            finite(v, (3,)) for v in (gains, velocity, acceleration, force_gains, force_velocity_gains))
        self.cutoff, self.force_cutoff = float(cutoff), float(force_cutoff)
        if (np.any(self.gains < 0) or np.any(self.vmax <= 0) or np.any(self.amax <= 0)
                or np.any(self.force_gains < 0) or np.any(self.force_velocity_gains < 0)
                or not np.isfinite(self.cutoff) or self.cutoff <= 0
                or not np.isfinite(self.force_cutoff) or self.force_cutoff <= 0):
            raise ValueError('invalid slow recentering policy')
        self.low, self.force_low, self.memory, self.command = (np.zeros(3) for _ in range(4))

    def step(self, measured_error, grasp_position, wrench_xy_yaw, dt):
        error, p, force = (finite(v, (3,)) for v in (measured_error, grasp_position, wrench_xy_yaw))
        if not np.isfinite(dt) or not 0. < dt <= .05:
            raise ValueError('yaw recentering timestep outside envelope')
        self.low += -np.expm1(-2*np.pi*self.cutoff*dt)*(error-self.low)
        self.force_low += -np.expm1(-2*np.pi*self.force_cutoff*dt)*(force-self.force_low)
        increment = self.force_gains*self.force_low*dt
        trial_memory = np.clip(self.memory+increment, -self.vmax, self.vmax)
        # A proportional term on the SAME slow force supplies damping in an
        # arm-unavailable direction. Unlike leaking velocity memory, it does
        # not require a permanent force to sustain constant transport velocity.
        force_velocity = self.force_velocity_gains*self.force_low
        trial = self.gains*self.low+trial_memory+force_velocity
        trial_body = np.array([trial[0]+p[1]*trial[2], trial[1]-p[0]*trial[2], trial[2]])
        outward = (abs(trial_body) > self.vmax) & (trial_body*increment > 0)
        self.memory = np.where(outward, self.memory, trial_memory)
        task = self.gains*self.low+self.memory+force_velocity
        desired_wz = np.clip(task[2], -self.vmax[2], self.vmax[2])
        wz = self.command[2]+np.clip(desired_wz-self.command[2], -self.amax[2]*dt, self.amax[2]*dt)
        desired_xy = task[:2]-wz*np.array([-p[1], p[0]])
        desired_xy = np.clip(desired_xy, -self.vmax[:2], self.vmax[:2])
        self.command[:2] += np.clip(desired_xy-self.command[:2], -self.amax[:2]*dt, self.amax[:2]*dt)
        self.command[2] = wz
        return self.command.copy()
