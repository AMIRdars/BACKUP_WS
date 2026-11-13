"""Bounded quintic arm translation, independent of base/peer/ground truth."""
import math
import numpy as np


def duration(distance, speed, acceleration):
    if not all(math.isfinite(v) and v > 0 for v in (distance,speed,acceleration)):
        raise ValueError('positive finite arm trajectory bounds required')
    return max(1.875*distance/speed, math.sqrt((10/math.sqrt(3))*distance/acceleration))


def reference(t, distance, speed, acceleration):
    if not math.isfinite(t) or not math.isfinite(distance) or distance == 0:
        raise ValueError('finite time and nonzero distance required')
    total = duration(abs(distance),speed,acceleration)
    u = float(np.clip(t/total,0.,1.))
    s = 10*u**3-15*u**4+6*u**5
    v = (30*u**2-60*u**3+30*u**4)/total
    return distance*s, distance*v, total
