"""Physical base-only calibration; GT is confined to this experiment process."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def pose(msg):
    p=msg.pose.pose
    q=p.orientation
    return np.array([p.position.x,p.position.y,
                     np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])


def delta(start,end):
    c,s=np.cos(start[2]),np.sin(start[2])
    xy=np.array([[c,s],[-s,c]])@(end[:2]-start[:2])
    yaw=np.arctan2(np.sin(end[2]-start[2]),np.cos(end[2]-start[2]))
    return np.r_[xy,yaw]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-directory',required=True)
    args=parser.parse_args()
    out=Path(args.output_directory)
    out.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Node('base_axis_experiment',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(Twist,'/amir2/rover_twist',1)
    latest={}
    log=(out/'samples.jsonl').open('w',buffering=1)
    stage='idle'
    command=np.zeros(3)
    def receive(key,msg):
        latest[key]=pose(msg)
        log.write(json.dumps({'sim_time':node.get_clock().now().nanoseconds*1e-9,
                             'stage':stage,'command':command.tolist(),'kind':key,
                             'pose':latest[key].tolist(),
                             'twist':[msg.twist.twist.linear.x,msg.twist.twist.linear.y,
                                      msg.twist.twist.angular.z]})+'\n')
    node.create_subscription(Odometry,'/amir2/odom',lambda m:receive('GT',m),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/amir2/mecanum_drive_controller/odometry',
                             lambda m:receive('encoder',m),qos_profile_sensor_data)
    def wait(duration):
        start=node.get_clock().now().nanoseconds*1e-9
        wall=time.monotonic()
        while node.get_clock().now().nanoseconds*1e-9-start<duration:
            if time.monotonic()-wall>20+duration*10:
                raise RuntimeError('clock stopped during base test')
            m=Twist()
            m.linear.x,m.linear.y,m.angular.z=map(float,command)
            pub.publish(m)
            until=time.monotonic()+.02
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.005)
    summaries={}
    try:
        deadline=time.monotonic()+10
        while (len(latest)<2 or node.get_clock().now().nanoseconds==0) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if len(latest)<2:
            raise RuntimeError('missing base-only experiment observations')
        for axis,label in enumerate(('vx','vy','wz')):
            stage='settle_'+label
            command[:]=0
            wait(1.)
            before={k:v.copy() for k,v in latest.items()}
            stage=label
            command[axis]=.01 if axis<2 else .02
            wait(2.)
            command[:]=0
            stage='stop_'+label
            wait(.5)
            changes={k:delta(before[k],latest[k]).tolist() for k in latest}
            expected=.02 if axis<2 else .04
            gt=np.array(changes['GT'])
            positive=gt[axis]>.2*expected
            cross=np.linalg.norm(gt[:2] if axis==2 else np.delete(gt[:2],axis))
            accurate=abs(gt[axis]-expected)<.5*expected
            passed=bool(positive and accurate and cross<(.01 if axis==2 else .005))
            summaries[label]={'expected':expected,'measured_body_delta':changes,'passed':passed}
        (out/'summary.json').write_text(json.dumps(summaries,indent=2))
        print(json.dumps(summaries,indent=2))
    finally:
        pub.publish(Twist())
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
