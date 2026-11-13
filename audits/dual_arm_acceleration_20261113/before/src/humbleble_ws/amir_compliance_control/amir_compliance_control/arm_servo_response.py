"""Validate a simulation position-servo response, never hardware load ratings."""
import hashlib
import numpy as np
from .wrench import finite


def model_digest(xml, directory):
    return hashlib.sha256(xml.replace(str(directory), '__AF_RUN_DIRECTORY__').encode()).hexdigest()


def validate_response(cert, xml, directory):
    names = [f'Joint_{i}' for i in range(1,6)]
    if (cert.get('scope')!='simulation_position_servo_response_only'
            or cert.get('passed') is not True or cert.get('ungrasped') is not True
            or cert.get('joint_names')!=names
            or cert.get('normalized_urdf_sha256')!=model_digest(xml,directory)):
        raise ValueError('matching measured ungrasped servo response required')
    gain,rate=float(cert['position_proportional_gain']),float(cert['controller_update_rate_hz'])
    if not np.isfinite(gain*rate) or gain<=0 or rate<=0:
        raise ValueError('positive measured servo parameters required')
    tau=finite(cert['joint_compensation_s'],(5,))
    if np.any(tau<0) or np.any(tau>.1):raise ValueError('servo compensation outside experiment envelope')
    covered=0
    for i,name in enumerate(names):
        measured=cert['joint_response'][name]
        if tau[i]>0:
            if (measured['samples']<20 or not np.isfinite(measured['velocity_gain_s_inv'])
                    or abs(measured['velocity_gain_s_inv']/(gain*rate)-1)>.15
                    or abs(tau[i]-1/(gain*rate))>1e-9):
                raise ValueError('servo response not verified for '+name)
            covered+=1
    if covered<3:raise ValueError('insufficient measured servo response coverage')
    return tau
