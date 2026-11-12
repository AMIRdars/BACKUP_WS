"""Necessary reach checks from the real URDF; never an IK/collision certificate.

Used by the experiment planner only. No peer goal or object GT enters the
follower. Free z and free orientation give a deliberately optimistic bound.
"""
import hashlib
import xml.etree.ElementTree as ET

import numpy as np

from .wrench import LocalModel, finite


def backward_reach_bound(xml, distance):
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError('positive finite backward distance required')
    model = LocalModel(xml)
    root = ET.fromstring(xml)
    q = finite([float(root.find(
        f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param").text)
        for name in model.names], (5,))
    home = model.frames(q)[0]
    # Both existing scenes face the follower toward the leader (yaw pi).
    # Leader world -x therefore requires follower local +x at fixed grasp.
    target_xy = home[:2] + [distance, 0.]
    transform, previous_axes, spheres = np.eye(4), [], []
    for index, (joint, fixed) in enumerate(zip(model.chain, model._origins)):
        transform = transform @ fixed
        if joint.get('type') == 'fixed':
            continue
        center = transform[:3, 3].copy()
        # The center is fixed in the base only if it lies on every preceding
        # revolute axis. This checks, rather than assumes, the shoulder offset.
        invariant = all(np.linalg.norm(np.cross(axis, center-point)) < 1e-10
                        for point, axis in previous_axes)
        if invariant:
            radius = float(sum(np.linalg.norm(t[:3, 3]) for t in model._origins[index+1:]))
            horizontal = float(np.linalg.norm(target_xy-center[:2]))
            lateral = float(home[1]-center[1])
            upper = float(np.sqrt(max(0., radius**2-lateral**2))-(home[0]-center[0]))
            spheres.append({'joint': joint.get('name'), 'center_base_footprint_m': center.tolist(),
                'link_length_sum_m': radius, 'target_horizontal_distance_m': horizontal,
                'backward_distance_upper_bound_m': upper,
                'impossible_even_with_free_z_and_orientation': horizontal > radius+1e-10})
        axis = transform[:3, :3] @ model.axes[len(previous_axes)]
        previous_axes.append((center, axis))
    if not spheres:
        raise ValueError('no base-fixed reach sphere in audited arm chain')
    return {'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
        'frame': 'follower/base_footprint', 'home_joint_rad': q.tolist(),
        'home_tcp_m': home.tolist(), 'requested_backward_distance_m': float(distance),
        'target_xy_m': target_xy.tolist(), 'fixed_base': True,
        'free_z': True, 'free_orientation': True, 'spheres': spheres,
        'backward_distance_upper_bound_m': min(s['backward_distance_upper_bound_m'] for s in spheres),
        'geometrically_impossible': any(s['impossible_even_with_free_z_and_orientation'] for s in spheres),
        'scope': 'necessary bound only; passing does not certify joint limits, collision, load or grasp'}


def translation_progress(distance, direction, leader_dx, payload_delta):
    """Evaluate signed straight travel; sideways/vertical motion cannot pass."""
    if direction not in ('forward', 'backward') or not np.isfinite(distance) or distance <= 0:
        raise ValueError('positive distance and explicit forward/backward direction required')
    delta = finite(payload_delta, (3,))
    if not np.isfinite(leader_dx):
        raise ValueError('nonfinite measured leader displacement')
    sign = 1. if direction == 'forward' else -1.
    return (0.9*distance <= sign*leader_dx <= 1.1*distance
            and sign*delta[0] >= .9*distance)
