"""Local slow base recentering; no leader, goal, world pose or TF input."""
import numpy as np
from .wrench import finite


class BaseRecentering:
    def __init__(self, cutoff, gains, velocity, acceleration, *, integral_gains=(0., 0., 0.),
                 force_acceleration_gains=(0., 0., 0.), force_filter_hz=.5,
                 stationary_hold=False):
        self.cutoff = float(cutoff)
        self.gains, self.vmax, self.amax = (finite(v, (3,)) for v in (gains, velocity, acceleration))
        self.integral_gains = finite(integral_gains, (3,))
        self.force_gains = finite(force_acceleration_gains, (3,))
        self.force_filter_hz = float(force_filter_hz)
        if (not np.isfinite(self.cutoff) or self.cutoff <= 0 or np.any(self.gains < 0)
                or np.any(self.vmax <= 0) or np.any(self.amax <= 0) or np.any(self.integral_gains < 0)
                or np.any(self.force_gains < 0) or not np.isfinite(self.force_filter_hz) or self.force_filter_hz <= 0
                or np.any((self.force_gains > 0) & (self.integral_gains > 0))):
            raise ValueError('finite positive recentering policies required')
        self.low, self.command = np.zeros(3), np.zeros(3)
        self.integral = np.zeros(3)
        self.force_low, self.force_memory = np.zeros(3), np.zeros(3)
        self.force_memory_enabled = bool(np.any(self.force_gains > 0))
        self.stationary_hold = bool(stationary_hold)
        if self.stationary_hold and not self.force_memory_enabled:
            raise ValueError('local stationary hold requires the own-force mode')
        self.seen_motion = np.zeros(3,dtype=bool)
        self.quiet_time = np.zeros(3)
        self.hold = np.zeros(3,dtype=bool)

    def step(self, error, grasp, dt, *, measured_force=None, measured_base_velocity=None,
             allow_stationary_hold=True):
        error, grasp = finite(error, (3,)), finite(grasp, (3,))
        if not np.isfinite(dt) or not 0 < dt <= .05:
            raise ValueError('base recentering time discontinuity')
        self.low += (1-np.exp(-2*np.pi*self.cutoff*dt))*(error-self.low)
        if self.force_memory_enabled:
            force = finite(measured_force, (3,))
            if self.stationary_hold:
                base_velocity = finite(measured_base_velocity, (3,))
                active_axes = self.force_gains > 0
                self.seen_motion |= active_axes & (abs(base_velocity) > .001)
                # Existing experiment settling bands, used only after this
                # robot has actually moved. No goal/leader completion signal.
                quiet = active_axes & self.seen_motion & (abs(base_velocity) <= .00001) & (abs(self.command) <= .00001) & (abs(error) <= .0001) & allow_stationary_hold
                self.quiet_time = np.where(quiet,self.quiet_time+dt,0.)
                self.hold |= self.quiet_time >= .3
                # Hysteresis: quiet-entry tolerance is not the release band.
                # Reject a significant new motion or persistent arm deflection.
                self.hold &= (abs(error) <= .0001) & (abs(base_velocity) <= .001)
                self.hold &= allow_stationary_hold
                if np.any(abs(error[:2]) > .0001) or abs(error[2]) > .001:
                    self.hold[:] = False
            self.force_low += (1-np.exp(-2*np.pi*self.force_filter_hz*dt))*(force-self.force_low)
            increment = self.force_gains*self.force_low*dt
            trial = self.gains*self.low+self.force_memory+increment
            wz = trial[2]
            body_trial = np.array([trial[0]+grasp[1]*wz,trial[1]-grasp[0]*wz,wz])
            outward = (np.abs(body_trial) > self.vmax) & (body_trial*increment > 0)
            self.force_memory = np.clip(self.force_memory+np.where(outward,0.,increment),-self.vmax,self.vmax)
            self.force_memory[self.hold] = 0.
            self.force_low[self.hold] = 0.
        # Optional sustained-deflection memory, in task velocity units. With
        # zero gains the legacy P controller has exactly its original behavior.
        # The finite cap prevents windup; no leader reference is used here.
        increment = self.integral_gains*self.low*dt
        trial = self.gains*self.low+self.integral+increment
        wz = trial[2]
        body_trial = np.array([trial[0]+grasp[1]*wz, trial[1]-grasp[0]*wz, wz])
        outward = (np.abs(body_trial) > self.vmax) & (body_trial*increment > 0)
        self.integral = np.clip(self.integral+np.where(outward, 0., increment), -self.vmax, self.vmax)
        task_velocity = self.gains*self.low+self.integral
        if self.force_memory_enabled:
            task_velocity += self.force_memory
            task_velocity[self.hold] = 0.
        wz = task_velocity[2]
        desired = np.array([task_velocity[0]+grasp[1]*wz,
                            task_velocity[1]-grasp[0]*wz, wz])
        desired = np.clip(desired, -self.vmax, self.vmax)
        self.command += np.clip(desired-self.command, -self.amax*dt, self.amax*dt)
        return self.command.copy()

    def authority(self, displacement):
        envelope = finite(displacement, (3,))
        if np.any(envelope <= 0):
            raise ValueError('positive displacement scales required')
        if self.stationary_hold and np.any(self.hold):
            # Zero preference alone still permits the whole-body allocator to
            # move the base. Constrain it while the local quiet hold is valid.
            return 0.
        credit = float(np.linalg.norm(self.low/envelope))
        if np.any(self.integral_gains > 0) or self.force_memory_enabled:
            # Retain the full slow PI request, including its proportional
            # damping term. Capping only at the integral removes that damping
            # once velocity exceeds the instantaneous deflection authority.
            credit = max(credit, float(np.max(np.abs(self.command)/self.vmax)))
        return credit
