"""Velocity validation for an audited simulation position-servo reference.

For q_dot=(q_ref-q)/tau, bounding the current reference error by tau*V
and the subsequent reference slope by V bounds modeled servo velocity by V.
Actual joint velocity, effort, position, freshness and grasp remain independently
checked by the safety gate. This is not a hardware load certificate.
"""
import numpy as np
from .wrench import finite


def servo_reference_ratio(actual, first, second, first_time, second_time, now, tau, limits):
    actual,first,second,tau,limits=[finite(v,(5,)) for v in (actual,first,second,tau,limits)]
    if np.any(tau<=0) or np.any(limits<=0) or not np.all(np.isfinite([first_time,second_time,now])) or second_time<=first_time:
        raise ValueError('audited positive servo response and ordered times required')
    if now>=second_time:raise ValueError('servo reference already expired')
    fraction=float(np.clip((now-first_time)/(second_time-first_time),0.,1.))
    reference=first+fraction*(second-first)
    ratio=float(np.max(np.abs(reference-actual)/tau/limits))
    segment=float(np.max(np.abs(second-first)/(second_time-first_time)/limits))
    if ratio>1.+1e-9 or segment>1.+1e-9:
        raise ValueError('servo reference violates unchanged URDF velocity')
    return max(ratio,segment)
