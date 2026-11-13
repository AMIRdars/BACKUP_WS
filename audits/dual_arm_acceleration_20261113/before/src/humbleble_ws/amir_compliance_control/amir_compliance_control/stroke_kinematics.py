"""Bounded local pose solve with measured residuals for the 5-DoF stroke.

The URDF has rounded mounting RPY values. Its arm plane is a few microradians
from footprint x. A five-joint arm cannot make all six residuals exactly zero.
Numerical acceptance (1um / 10urad) remains far inside existing pose guards.
"""
import numpy as np
from .wrench import finite
from .admittance import rotation_error


def verify_stroke_branch(model,seed,origin,rotation,distance):
    """Pure bounded preflight; run outside the ROS receive/timer callbacks."""
    q=np.array(seed,copy=True)
    for delta in np.linspace(0,distance,31):
        q=solve_stroke_pose(model,q,origin+[delta,0,0],rotation)


def stroke_pose_velocity(model, q, velocity_x):
    """Differentiate the fixed-orientation path without differencing IK errors."""
    velocity=finite([velocity_x,0.,0.,0.,0.,0.],(6,))
    weight=np.array([1.,1.,1.,.1,.1,.1])
    jacobian=model.jacobian(finite(q,(5,)))*weight[:,None]
    return np.linalg.lstsq(jacobian,velocity*weight,rcond=None)[0]


def solve_stroke_pose(model,seed,target,rotation):
    q=finite(seed,(5,)).copy();target=finite(target,(3,))
    lower,upper=model.lower+.01,model.upper-.01
    if np.any(q<=lower) or np.any(q>=upper):raise ValueError('stroke IK seed joint margin')
    weight=np.array([1.,1.,1.,.1,.1,.1])
    for _ in range(80):
        actual,sensor=model.frames(q);ep=target-actual;er=rotation_error(rotation,sensor[:3,:3])
        if np.linalg.norm(ep)<1e-6 and np.linalg.norm(er)<1e-5:return q
        j=model.jacobian(q)*weight[:,None];error=np.r_[ep,er]*weight
        step=j.T@np.linalg.solve(j@j.T+1e-6*np.eye(6),error)
        step*=min(1.,.01/max(float(np.max(abs(step))),1e-15))
        q=np.clip(q+step,lower+1e-9,upper-1e-9)
    raise ValueError('stroke pose residual outside numerical acceptance')
