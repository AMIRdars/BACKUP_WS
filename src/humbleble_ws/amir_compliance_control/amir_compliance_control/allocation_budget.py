"""Slow authority for a whole-body compliance task, separate from FK error.

The virtual state is never relabeled measured yaw and never makes a separate
base command. The sole eight-variable allocator retains all hard bounds.
"""
import numpy as np
from .wrench import finite


class TaskComplianceBudget:
    def __init__(self,limits,cutoff_hz):
        self.limits=finite(limits,(3,)).copy();self.cutoff=float(cutoff_hz)
        if np.any(self.limits<=0.) or not np.isfinite(self.cutoff) or self.cutoff<=0.:
            raise ValueError('existing positive displacement scales and slow cutoff required')
        self.low=np.zeros(3)

    def step(self,virtual_target_xy_yaw,dt):
        target=finite(virtual_target_xy_yaw,(3,))/self.limits
        if not np.isfinite(dt) or not 0.<dt<=.05:raise ValueError('allocation budget interval outside envelope')
        self.low += -np.expm1(-2*np.pi*self.cutoff*dt)*(target-self.low)
        return float(np.linalg.norm(self.low))

    def reset(self):
        self.low[:]=0.
