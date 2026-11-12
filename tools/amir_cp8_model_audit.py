#!/usr/bin/env python3
"""Reproducible actual-URDF CP-8 kinematics and bounded allocation evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from amir_compliance_control.wrench import LocalModel,rotation
from amir_compliance_control.whole_body import whole_jacobian,WholeBodyAllocator
from amir_compliance_control.admittance import solve_pose


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--urdf',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    xml=a.urdf.read_bytes();m=LocalModel(xml);q=np.array([0,1.4,-1.6,.2,1.570796]);rng=np.random.default_rng(81)
    errors={};samples=[]
    for sensor_point in (False,True):
        peaks=[]
        for pose in [q,*rng.uniform(m.lower+.1,m.upper-.1,(50,5))]:
            r=rotation([0,0,1],rng.uniform(-3,3))
            def world(d):
                tcp,s=m.frames(pose+d[:5]);point=s[:3,3] if sensor_point else tcp;rb=r@rotation([0,0,1],d[7])
                return rb@point+r@np.r_[d[5:7],0],rb@s[:3,:3]
            j=whole_jacobian(m,pose,at_sensor=sensor_point);j[:3]=r@j[:3];j[3:]=r@j[3:]
            numeric=np.zeros((6,8));_,r0=world(np.zeros(8));h=1e-6
            for i in range(8):
                d=np.eye(8)[i]*h;p1,r1=world(d);p0,rm=world(-d);numeric[:3,i]=(p1-p0)/(2*h)
                skew=(r1-rm)/(2*h)@r0.T;numeric[3:,i]=[skew[2,1],skew[0,2],skew[1,0]]
            peaks.append(float(np.max(abs(j-numeric))))
        errors['sensor' if sensor_point else 'tcp']=max(peaks)
    position,sensor=m.frames(q)
    feasibility=[]
    for label,delta in [('backward',[-.0001,0,0]),('lateral',[0,.0001,0]),('yaw',[0,0,0])]:
        target_r=rotation([0,0,1],.0002)@sensor[:3,:3] if label=='yaw' else sensor[:3,:3]
        try:solve_pose(m,q,position+delta,target_r);old=True
        except ValueError:old=False
        desired=np.r_[np.array(delta),[0,0,.00005 if label=='yaw' else 0]]
        if label!='yaw':desired[:3]*=.5
        allocator=WholeBodyAllocator(m)
        for _ in range(50):u=allocator.step(q,q,desired,[0,0,0],1.,.01)
        residual=float(np.linalg.norm((whole_jacobian(m,q)@u-desired)*[1,1,1,.1,.1,.1]))
        feasibility.append({'task':label,'old_locked_pose_ik_feasible':old,'whole_body_residual_metric_m_s':residual,'whole_velocity':u.tolist()})
    allocator=WholeBodyAllocator(m);latencies=[]
    for _ in range(300):
        start=time.perf_counter();allocator.step(q,q,[1e-8,1e-8,0,0,0,0],[0,0,0],.00001,.02);latencies.append((time.perf_counter()-start)*1000)
    for label,pose in [('home',q),('near_joint_1_upper',np.r_[m.upper[0]-.011,q[1:]]),('near_arm_singularity',np.r_[q[0],[.021,-.042,.021],q[4]])]:
        allocator=WholeBodyAllocator(m);u=allocator.step(pose,pose,[0,.00005,0,0,0,0],[0,0,0],1.,.02)
        samples.append({'pose':label,'q':pose.tolist(),**allocator.last})
    report={'urdf':str(a.urdf.resolve()),'sha256':hashlib.sha256(xml).hexdigest(),'joint_names':m.names,'home_q':q.tolist(),
        'tcp_xyz':position.tolist(),'sensor_xyz':sensor[:3,3].tolist(),'sensor_to_tcp_norm_m':float(np.linalg.norm(position-sensor[:3,3])),
        'arm_rank':int(np.linalg.matrix_rank(m.jacobian(q))),'whole_rank':int(np.linalg.matrix_rank(whole_jacobian(m,q))),
        'arm_singular_values':np.linalg.svd(m.jacobian(q),compute_uv=False).tolist(),
        'whole_singular_values':np.linalg.svd(whole_jacobian(m,q),compute_uv=False).tolist(),
        'finite_difference_samples_per_point':51,'finite_difference_max_error':errors,'feasibility':feasibility,
        'adaptive_samples':samples,'allocator_wall_ms_p50_p95_max':np.percentile(latencies,[50,95,100]).tolist(),
        'scope':'kinematic feasibility; not a dynamic carry or certified safety limit'}
    assert max(errors.values())<2e-9 and report['whole_rank']==6
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
