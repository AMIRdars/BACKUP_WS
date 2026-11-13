"""Offline common horizontal reach from actual URDF and existing soft margins."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.stroke_kinematics import solve_stroke_pose,stroke_pose_velocity
from amir_native_arm_support import SCENE
from amir_compliance_control.arm_translation import duration,reference


def common_reach(root, step=.001):
    proof={}
    for robot,sign in (('leader',1.),('follower',-1.)):
        path=Path(root)/SCENE/(robot+'.urdf');tree=ET.parse(path);model=LocalModel(path.read_text())
        q=np.array([float(tree.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text) for n in model.names])
        home,sensor=model.frames(q);last=0.;reason='scan bound'
        for value in np.arange(step,.601,step):
            try:next_q=solve_stroke_pose(model,q,home+[sign*value,0.,0.],sensor[:3,:3])
            except ValueError as exc:
                reason=str(exc);break
            q=next_q;last=float(value)
        proof[robot]={'verified_horizontal_reach_m':last,'first_unverified_m':last+step,
                      'last_q_rad':q.tolist(),'boundary_reason':reason}
    return {'maximum_common_verified_m':min(p['verified_horizontal_reach_m'] for p in proof.values()),
            'resolution_m':step,'arms':proof,'scope':'same initial grasp, horizontal translation, fixed orientation; not collision/load qualification'}


def trajectory_rate_proof(root,distance,speed,acceleration=.0012):
    total=duration(distance,speed,acceleration);times=np.linspace(0,total,501);proof={}
    for robot,sign in (('leader',1.),('follower',-1.)):
        path=Path(root)/SCENE/(robot+'.urdf');tree=ET.parse(path);model=LocalModel(path.read_text())
        q=np.array([float(tree.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text) for n in model.names])
        home,sensor=model.frames(q);samples=[]
        for t in times:
            x,v,_=reference(t,distance,speed,acceleration)
            q=solve_stroke_pose(model,q,home+[sign*x,0,0],sensor[:3,:3])
            samples.append(stroke_pose_velocity(model,q,sign*v))
        qdot=np.array(samples)
        proof[robot]={'peak_joint_speed_rad_s':float(np.max(abs(qdot))),
            'peak_joint_acceleration_rad_s2':float(np.max(abs(np.diff(qdot,axis=0)/(total/500))))}
    return {'duration_s':total,'arms':proof,
            'peak_joint_speed_rad_s':max(p['peak_joint_speed_rad_s'] for p in proof.values()),
            'peak_joint_acceleration_rad_s2':max(p['peak_joint_acceleration_rad_s2'] for p in proof.values())}


def wait_for_arm_completion(wait,sim,completed,distance,speed,acceleration=.0012):
    """Profile/arrival budget uses sim time; all existing wall heartbeats remain."""
    budget=duration(distance,speed,acceleration)+10.
    started=sim[0]
    def ready():
        if completed():return True
        if sim[0]-started>budget:
            raise RuntimeError('leader arm simulation-time mission budget exceeded')
        return False
    # This is a batch-runner deadline, not a controller/sensor watchdog.
    # Native clock/permit/F/T/joint guards continue their original checks.
    wait(ready,max(60.,budget*10.),True)
