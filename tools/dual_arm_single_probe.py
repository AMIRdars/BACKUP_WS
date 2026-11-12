"""Before grasp, qualify a 10 mm leader Cartesian arm path and return."""
import json
import time
import numpy as np
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.stroke_kinematics import solve_stroke_pose
from amir_compliance_control.arm_translation import reference,duration


def ungrasped_arm_probe(*,node,folder,out,latest,sim,wait,phase,poses):
    model=LocalModel((folder/'leader.urdf').read_text())
    message=latest['leader','q'];mapping=dict(zip(message.name,message.position))
    home_q=np.array([mapping[name] for name in model.names]);home,sensor=model.frames(home_q)
    period=duration(.01,.003,.0012);samples=[];q=home_q.copy()
    target_topic='/leader/arm_controller/joint_trajectory'
    if node.get_publishers_info_by_topic(target_topic):raise RuntimeError('single arm probe writer conflict')
    pub=node.create_publisher(JointTrajectory,target_topic,1)
    phase[0]='leader_ungrasped_cartesian_probe'
    body_home={r:np.array([poses[r].translation.x,poses[r].translation.y]) for r in ('leader','follower')}
    msg=JointTrajectory(joint_names=list(model.names))
    for t in np.linspace(0,2*period+1.,251):
        if t<=period:x=reference(t,.01,.003,.0012)[0]
        elif t<period+1.:x=.01
        else:x=.01-reference(t-period-1.,.01,.003,.0012)[0]
        q=solve_stroke_pose(model,q,home+[x,0,0],sensor[:3,:3])
        seconds=float(t)+.004
        point=JointTrajectoryPoint(positions=q.tolist())
        sec,ns=divmod(round(seconds*1e9),1000000000)
        point.time_from_start=Duration(sec=sec,nanosec=ns);msg.points.append(point)
    try:
        def fresh_ready():
            m=latest['leader','q'];age=sim[0]-m.header.stamp.sec-m.header.stamp.nanosec*1e-9
            return pub.get_subscription_count()>=1 and -.02<=age<=.02
        # Offline IK planning must not consume a command epoch. Drain feedback
        # after planning, then stamp the trajectory from the refreshed clock.
        wait(fresh_ready,5.)
        started=sim[0];msg.header.stamp=Time(seconds=started+.1).to_msg()
        pub.publish(msg)
        last_coherent=[time.monotonic()]
        def done():
            m=latest['leader','q'];state=dict(zip(m.name,m.position));velocity=dict(zip(m.name,m.velocity))
            actual=model.frames([state[name] for name in model.names])[0]
            if max(abs(velocity[name]) for name in model.names)>.03+1e-6:
                raise RuntimeError('single arm probe measured velocity')
            age=sim[0]-m.header.stamp.sec-m.header.stamp.nanosec*1e-9
            if not -.02<=age<=.05:
                if time.monotonic()-last_coherent[0]>.15:
                    raise RuntimeError('single arm probe state stale: '+str(age))
                return False
            last_coherent[0]=time.monotonic()
            drift={r:float(np.linalg.norm(np.array([poses[r].translation.x,poses[r].translation.y])-body_home[r])) for r in body_home}
            if max(drift.values())>=.001:raise RuntimeError('single arm probe base drift')
            samples.append({'sim_time':sim[0],'p_actual_m':actual.tolist(),'body_drift_m':drift})
            return sim[0]-started>2*period+1.5 and np.linalg.norm(actual-home)<.0001
        wait(done,80.)
        peak=float(max(sample['p_actual_m'][0]-home[0] for sample in samples))
        passed=bool(.0095<peak<.0105)
        (out/'leader_ungrasped_cartesian_probe.json').write_text(json.dumps({'passed':passed,
            'ungrasped':True,'payload_contact':False,'scope':'arm/controller Cartesian qualification only; not single-arm object handling',
            'requested_distance_m':.01,'measured_peak_x_m':peak,'returned_home':True,'samples':samples},indent=2))
        if not passed:raise RuntimeError('leader Cartesian probe did not reach measured 10mm')
    finally:node.destroy_publisher(pub)
