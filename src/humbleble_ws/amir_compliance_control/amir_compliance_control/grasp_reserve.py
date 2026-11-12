"""Own native finger reserve, using the existing CP-9/11 experiment bounds."""
import numpy as np
from .wrench import finite


def native_reserve_risk(sample, sim_time, receipt_age):
    try:
        if (receipt_age > .15 or sample['state'] != 'HOLDING' or not sample['healthy']
                or not -.02 <= sim_time-float(sample['sim_time']) <= .25
                or not all(sample['own_contacts'][side] for side in ('left', 'right'))):
            raise ValueError('native grasp reserve is not fresh and holding')
        forces = finite([sample['own_forces_N'][side] for side in ('left', 'right')], (2,))
        if np.min(forces) < 8. or np.max(forces) > 26.:
            raise ValueError('native grasp force outside existing 8/26 N envelope')
        return max(0., 1.-min((np.min(forces)-8.)/12., (26.-np.max(forces))/6.))
    except (KeyError, TypeError) as exc:
        raise ValueError('native grasp reserve measurement invalid') from exc
