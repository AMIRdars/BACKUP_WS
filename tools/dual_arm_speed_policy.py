"""Explicit simulation speed envelope, constrained by both unchanged URDFs.

This is a motion-planning policy, not a qualification of friction/load limits.
The existing contact, wrench, position and base-stop guards remain separate.
"""
import math
from pathlib import Path

import numpy as np
from amir_compliance_control.wrench import LocalModel
from amir_native_arm_support import SCENE


def validate_speed_policy(root, joint_speed, joint_acceleration, tcp_acceleration):
    values = (joint_speed, joint_acceleration, tcp_acceleration)
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('positive finite dual-arm speed/acceleration policy required')
    # Acceleration is an explicit simulation experiment limit; URDFs do not
    # provide an acceleration rating. Do not describe it as a hardware rating.
    if joint_acceleration > .5 or tcp_acceleration > .05:
        raise ValueError('dual-arm acceleration exceeds bounded simulation exploration envelope')
    limits = {}
    for robot in ('leader', 'follower'):
        model = LocalModel((Path(root) / SCENE / (robot + '.urdf')).read_text())
        if joint_speed > float(np.min(model.velocity)):
            raise ValueError('dual-arm joint speed exceeds unchanged URDF velocity limit: ' + robot)
        limits[robot] = model.velocity.tolist()
    return {'joint_speed_rad_s': joint_speed, 'joint_acceleration_rad_s2': joint_acceleration,
            'tcp_acceleration_m_s2': tcp_acceleration, 'URDF_velocity_rad_s': limits,
            'follower_acceleration_limit_m_s2':[max(.02,2*tcp_acceleration),.02],
            'scope': 'explicit Gazebo exploration; contact and tracking must be measured separately'}


def select_speed(root, distance, requested_speed, joint_speed, joint_acceleration, tcp_acceleration):
    from dual_arm_workspace import trajectory_rate_proof
    policy = validate_speed_policy(root, joint_speed, joint_acceleration, tcp_acceleration)
    if not math.isfinite(requested_speed) or not 0 < requested_speed <= .2:
        raise ValueError('dual-arm requested speed must be positive and <=0.2 m/s; actual profile is URDF bounded')
    selected = requested_speed
    for _ in range(5):
        proof = trajectory_rate_proof(root, distance, selected, tcp_acceleration)
        ratio = max(proof['peak_joint_speed_rad_s'] / (.95 * joint_speed),
                    math.sqrt(proof['peak_joint_acceleration_rad_s2'] / (.95 * joint_acceleration)))
        if ratio <= 1. + 1e-9:
            policy.update(requested_speed_m_s=requested_speed, selected_speed_m_s=selected,
                          actual_profile_peak_speed_m_s=1.875 * distance / proof['duration_s'],
                          trajectory_start_delay_s=0.,
                          servo_streaming_model_enabled=selected>.008)
            return policy, proof
        selected /= ratio * 1.001
    raise ValueError('could not bound both-arm joint velocity/acceleration before simulation')
