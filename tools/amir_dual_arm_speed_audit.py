#!/usr/bin/env python3
"""Offline speed feasibility on the existing two-arm fixed-orientation path.

This tool never starts ROS/Gazebo or changes controller/safety limits. Requested
TCP peak speed is evaluated separately from the acceleration-limited profile.
URDF velocity bounds alone do not qualify contact, torque or tracking.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from amir_compliance_control.arm_translation import duration
from amir_compliance_control.stroke_kinematics import solve_stroke_pose,stroke_pose_velocity
from amir_compliance_control.wrench import LocalModel
from amir_native_arm_support import SCENE
from dual_arm_workspace import common_reach


def path_rates(root,distance,samples=501):
    if samples<101:raise ValueError('at least 101 path samples required')
    u=np.linspace(0.,1.,samples)
    shape=10*u**3-15*u**4+6*u**5
    velocity=distance*(30*u**2-60*u**3+30*u**4)
    arms={}
    for robot,sign in (('leader',1.),('follower',-1.)):
        path=Path(root)/SCENE/(robot+'.urdf');xml=path.read_text()
        tree=ET.fromstring(xml);model=LocalModel(xml)
        q=np.array([float(tree.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text) for n in model.names])
        home,sensor=model.frames(q);rates=[]
        for x,v in zip(distance*shape,velocity):
            q=solve_stroke_pose(model,q,home+[sign*x,0.,0.],sensor[:3,:3])
            rates.append(stroke_pose_velocity(model,q,sign*v))
        rates=np.array(rates)
        acceleration=np.gradient(rates,u,axis=0,edge_order=2)
        limits=np.array([float(tree.find(f"joint[@name='{n}']/limit").get('velocity')) for n in model.names])
        arms[robot]={'joint_names':list(model.names),'URDF_velocity_rad_s':limits.tolist(),
            'unit_duration_joint_speed_peak':np.max(abs(rates),axis=0).tolist(),
            'unit_duration_joint_acceleration_peak':np.max(abs(acceleration),axis=0).tolist(),
            'urdf_sha256':hashlib.sha256(xml.encode()).hexdigest()}
    return arms


def assess(root,distance,target_speed,samples=501,joint_speed=.03,joint_acceleration=.15,tcp_acceleration=.0012):
    if not all(math.isfinite(v) and v>0 for v in (distance,target_speed)):
        raise ValueError('positive finite distance and target speed required')
    reach=common_reach(root)
    if distance>reach['maximum_common_verified_m']:
        raise ValueError('distance exceeds common verified URDF reach')
    arms=path_rates(root,distance,samples)
    from dual_arm_speed_policy import validate_speed_policy
    validate_speed_policy(root,joint_speed,joint_acceleration,tcp_acceleration)
    nominal_time=1.875*distance/target_speed
    current_accel_time=duration(distance,target_speed,tcp_acceleration)
    joint_time=0.;accel_time=0.;urdf_time=0.
    for arm in arms.values():
        qv=np.array(arm['unit_duration_joint_speed_peak'])
        qa=np.array(arm['unit_duration_joint_acceleration_peak'])
        arm['requested_peak_joint_speed_rad_s']=(qv/nominal_time).tolist()
        arm['requested_peak_joint_acceleration_rad_s2']=(qa/nominal_time**2).tolist()
        arm['requested_URDF_velocity_ratio_peak']=float(np.max(qv/nominal_time/arm['URDF_velocity_rad_s']))
        joint_time=max(joint_time,float(np.max(qv))/(joint_speed*.95))
        accel_time=max(accel_time,math.sqrt(float(np.max(qa))/(joint_acceleration*.95)))
        urdf_time=max(urdf_time,float(np.max(qv/arm['URDF_velocity_rad_s'])))
    bounded_time=max(current_accel_time,joint_time,accel_time)
    return {'scope':'offline same-grasp horizontal fixed-orientation arm path; no contact/load qualification',
        'distance_m':distance,'target_TCP_peak_speed_m_s':target_speed,
        'target_nominal_duration_s':nominal_time,
        'target_required_TCP_peak_acceleration_m_s2':10/math.sqrt(3)*distance/nominal_time**2,
        'launch_accepts_requested_speed':target_speed<=.2,
        'target_respects_URDF_velocity':all(a['requested_URDF_velocity_ratio_peak']<=1 for a in arms.values()),
        'current_acceleration_limited_duration_s':current_accel_time,
        'current_acceleration_limited_TCP_peak_speed_m_s':1.875*distance/current_accel_time,
        'existing_guard_bounded_duration_s':bounded_time,
        'existing_guard_bounded_TCP_peak_speed_m_s':1.875*distance/bounded_time,
        'URDF_velocity_only_TCP_peak_ceiling_m_s':1.875*distance/urdf_time,
        'URDF_ceiling_is_not_a_qualified_command':True,
        'experiment_limits':{'joint_speed_rad_s':joint_speed,'joint_acceleration_rad_s2':joint_acceleration,
            'TCP_acceleration_m_s2':tcp_acceleration,'launch_requested_speed_cap_m_s':.2},
        'samples':samples,'arms':arms}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance',type=float,default=.213)
    parser.add_argument('--target-speed',type=float,default=1.)
    parser.add_argument('--joint-speed',type=float,default=.03)
    parser.add_argument('--joint-acceleration',type=float,default=.15)
    parser.add_argument('--acceleration',type=float,default=.0012)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('output already exists')
    root=Path(__file__).resolve().parents[1]
    result=assess(root,args.distance,args.target_speed,joint_speed=args.joint_speed,
        joint_acceleration=args.joint_acceleration,tcp_acceleration=args.acceleration)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='arms'},indent=2))


if __name__=='__main__':main()
