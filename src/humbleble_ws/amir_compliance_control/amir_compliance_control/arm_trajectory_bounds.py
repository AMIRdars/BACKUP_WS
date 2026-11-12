"""Project two streaming position knots without relaxing joint-speed bounds."""
import numpy as np


def streaming_knots(proposed_end, velocity, horizon, limits,
                    start_lower, start_upper, end_lower, end_upper):
    """Preserve both knot slew/measurement boxes and the segment speed limit.

    A rapidly changing DLS request can demand an impossible *offset* even when
    the two position boxes overlap. Project that offset first, then the start
    position. Empty position boxes remain a fault; no measured-state or joint
    safety boundary is widened.
    """
    if np.any(start_lower > start_upper) or np.any(end_lower > end_upper):
        raise ValueError('ARM_ONLY streaming knot position bounds infeasible')
    lo=np.maximum(end_lower-start_upper,-limits*horizon)
    hi=np.minimum(end_upper-start_lower,limits*horizon)
    if np.any(lo > hi):
        raise ValueError('ARM_ONLY streaming knot rate infeasible')
    requested=horizon*velocity
    offset=np.clip(requested,lo,hi)
    start=np.clip(proposed_end-offset,np.maximum(start_lower,end_lower-offset),
                  np.minimum(start_upper,end_upper-offset))
    return start,start+offset,offset/horizon
