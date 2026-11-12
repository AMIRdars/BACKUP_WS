"""Chapter 23 A/B/C ablation, in ONE explicitly named local base frame."""
import math
import numpy as np
from .wrench import finite
from .rotation_observer import wrap_pi

MODES=('XY_ONLY','XY_YAW','XY_YAW_COMP','WHOLE_BODY')


def base_reference(position,home,psi,psi_home,*,mode,yaw_valid,gains,velocity):
    if mode not in MODES[:3]:raise ValueError('exclusive A/B/C mode required')
    p,initial=finite(position,(3,)),finite(home,(3,))
    gain,limit=finite(gains,(3,)),finite(velocity,(3,))
    if np.any(gain<0.) or np.any(limit<=0.) or not np.isfinite([psi,psi_home]).all():
        raise ValueError('finite nonnegative gains and positive explicit limits required')
    if mode!='XY_ONLY' and not yaw_valid:raise ValueError('local yaw not observable')
    error=np.r_[(p-initial)[:2],wrap_pi(psi-psi_home)]
    omega=0. if mode=='XY_ONLY' else float(np.clip(gain[2]*error[2],-limit[2],limit[2]))
    compensation=omega*np.array([p[1],-p[0]]) if mode=='XY_YAW_COMP' else np.zeros(2)
    command=np.r_[np.clip(gain[:2]*error[:2]+compensation,-limit[:2],limit[:2]),omega]
    return command,compensation,error


class RotationRecentering:
    """One LPF and limiter; compensation uses the limited yaw actually sent.

    WHOLE_BODY has only an UNCOMPENSATED prior. Its J_base already contains
    omega cross p, so no explicit compensation is applied to its task/prior.
    No force velocity memory or world-velocity feedforward is used in A/B/C.
    """
    def __init__(self,*,mode,gains,velocity,acceleration,cutoff):
        if mode not in MODES:raise ValueError('unknown exclusive rotation mode')
        self.mode=mode
        self.gains,self.vmax,self.amax=(finite(v,(3,)) for v in (gains,velocity,acceleration))
        self.cutoff=float(cutoff)
        if np.any(self.gains<0.) or np.any(self.vmax<=0.) or np.any(self.amax<=0.) or not math.isfinite(self.cutoff) or self.cutoff<=0.:
            raise ValueError('explicit finite recentering policies required')
        self.low,self.command,self.memory=(np.zeros(3) for _ in range(3))
        self.compensation=np.zeros(2)

    def step(self,error,position,dt,*,yaw_valid):
        error,p=finite(error,(3,)),finite(position,(3,))
        if not math.isfinite(dt) or not 0.<dt<=.05:raise ValueError('recentring timestep outside envelope')
        if self.mode!='XY_ONLY' and not yaw_valid:raise ValueError('local yaw not observable')
        self.low += -np.expm1(-2*np.pi*self.cutoff*dt)*(error-self.low)
        desired_w=0. if self.mode=='XY_ONLY' else np.clip(self.gains[2]*self.low[2],-self.vmax[2],self.vmax[2])
        return self.limit_command(self.gains[:2]*self.low[:2],desired_w,p,dt)

    def limit_command(self,translation,requested_w,position,dt):
        """Shared production/probe limiter; no second actuator writer."""
        translation,p=finite(translation,(2,)),finite(position,(3,))
        if not np.isfinite([requested_w,dt]).all() or not 0.<dt<=.05:
            raise ValueError('finite local twist and bounded timestep required')
        desired_w=0. if self.mode=='XY_ONLY' else np.clip(requested_w,-self.vmax[2],self.vmax[2])
        omega=self.command[2]+np.clip(desired_w-self.command[2],-self.amax[2]*dt,self.amax[2]*dt)
        self.compensation=omega*np.array([p[1],-p[0]]) if self.mode=='XY_YAW_COMP' else np.zeros(2)
        desired=np.clip(translation+self.compensation,-self.vmax[:2],self.vmax[:2])
        self.command[:2] += np.clip(desired-self.command[:2],-self.amax[:2]*dt,self.amax[:2]*dt)
        self.command[2]=omega
        return self.command.copy()


def whole_body_task(virtual_velocity,target,error,pose_gain):
    """World instantaneous task in local axes; base coupling is in J_whole.

    In particular this does NOT add J_base @ base_preference to the target.
    The preference belongs solely to the allocator's regularization term.
    """
    if not math.isfinite(pose_gain) or pose_gain<=0.:raise ValueError('positive pose gain required')
    return finite(virtual_velocity,(6,))+pose_gain*(finite(target,(6,))-finite(error,(6,)))
