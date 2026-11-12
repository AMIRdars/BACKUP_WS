"""URDF-derived opt-in transport postures; no world-state input to follower."""
import xml.etree.ElementTree as ET

import numpy as np

from .core import ArmURDF, finite


HOME = np.array([0.,1.4,-1.6,.2,1.570796])


def tip_pitch(t):
    return np.arctan2(t[2,1],np.linalg.norm(t[:2,1]))


def solve_posture(arm, height, pitch, reach=None, wrist_angle=HOME[4]):
    """Solve three planar joint positions from actual URDF FK, within limits."""
    q = HOME.copy()
    if not np.isfinite(wrist_angle) or not arm.lower[4]+.08 <= wrist_angle <= arm.upper[4]-.08:
        raise ValueError('wrist angle violates actual joint margin')
    q[4] = wrist_angle
    if reach is None:
        reach = arm.evaluate(q)[0][0,3]
    target = finite([reach,height,pitch],(3,))
    def task(q_):
        t,_,_ = arm.evaluate(q_)
        return np.array([t[0,3],t[2,3],tip_pitch(t)])
    for _ in range(80):
        error = target-task(q)
        if np.linalg.norm(error) < 1e-9:
            if np.min(np.linalg.svd(arm.evaluate(q)[2],compute_uv=False)) < .02:
                raise ValueError('posture does not provide four independent task axes')
            return q
        columns = []
        for index in (1,2,3):
            perturbed = q.copy()
            perturbed[index] += 1e-6
            columns.append((task(perturbed)-task(q))/1e-6)
        delta = np.linalg.lstsq(np.array(columns).T,error,rcond=1e-8)[0]
        delta *= min(1.,.15/max(np.max(np.abs(delta)),1e-12))
        q[1:4] = np.clip(q[1:4]+delta,arm.lower[1:4]+.08,arm.upper[1:4]-.08)
    raise ValueError('no feasible grasp-compatible posture found')


def paired_postures(xml, pitch, half_span, wrist_angle=HOME[4], reach=None):
    if not .05 <= abs(pitch) <= .25 or not 0 < half_span < 1.:
        raise ValueError('posture redesign is limited to 0.05..0.25 rad and audited payload span')
    arm = ArmURDF(xml)
    home = arm.evaluate(HOME)[0]
    return [solve_posture(arm,home[2,3]+side*half_span*np.sin(pitch),-side*pitch,
                          wrist_angle=wrist_angle,reach=reach)
            for side in (-1,1)]


def with_initial_positions(xml, q):
    q = finite(q,(5,))
    arm = ArmURDF(xml)
    if np.any(q < arm.lower+.08) or np.any(q > arm.upper-.08):
        raise ValueError('initial posture violates joint margin')
    root = ET.fromstring(xml)
    for name,value in zip(arm.names,q):
        joint = root.find(f"ros2_control/joint[@name='{name}']")
        if joint is None:
            joint = next((c.find(f"joint[@name='{name}']") for c in root.findall('ros2_control')
                          if c.find(f"joint[@name='{name}']") is not None),None)
        if joint is None:
            raise ValueError(f'missing actual ros2_control joint {name}')
        initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if initial is None:
            raise ValueError('missing initial state interface')
        initial.text = format(value,'.16g')
    return ET.tostring(root,encoding='unicode')
