"""Pair existing encoder/GT samples by their original source timestamps.

Independent evaluator only. No interpolation, receipt-time stamping, raised
age/skew limits, controller changes, or use by a transport follower.
"""
from bisect import bisect_left
import math


def select_source_pair(encoder_history, gt_history, now_s):
    """Return freshest matchable GT and its nearest encoder source sample.

Latest callbacks can arrive at different instants even when their histories
contain a valid pair. Keep source age <=50ms, future <=20ms and skew <=20ms.
    GT defines the evaluation instant; prefer its latest matchable source time,
    then the smallest absolute skew. Never interpolate either measurement.
"""
    now_s = float(now_s)
    if not math.isfinite(now_s):
        raise ValueError('finite current simulation time required')
    histories = []
    for history in (encoder_history, gt_history):
        values = []
        for stamp, message in history:
            stamp = float(stamp)
            if not math.isfinite(stamp):
                raise ValueError('nonfinite feedback source stamp')
            if -.02-1e-9 <= now_s-stamp <= .05+1e-9:
                values.append((stamp, message))
        histories.append(sorted(values, key=lambda value: value[0]))
    encoder, truth = histories
    stamps = [entry[0] for entry in encoder]
    best, score = None, None
    for gt in truth:
        index = bisect_left(stamps, gt[0])
        for i in (index-1, index):
            if not 0 <= i < len(encoder):
                continue
            own = encoder[i]
            skew = abs(own[0]-gt[0])
            if skew > .02+1e-9:
                continue
            value = (gt[0], -skew, own[0])
            if score is None or value > score:
                score, best = value, {'encoder': own, 'gt': gt}
    if best is None:
        raise ValueError('no encoder/GT source pair within existing age/future/skew bounds')
    return best
