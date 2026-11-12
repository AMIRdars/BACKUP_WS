"""Known bounded physical wrench pulses; run only in the ungrasped fixture.

This is an experiment process, never imported by the follower. Does not
publish motion, attachment, or gripper commands. Needs external_wrench_test.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',required=True)
    args=p.parse_args()
    rclpy.init()
    node=Node('wrist_excitation_experiment',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(Wrench,'/experiment/amir2/wrench_base',1)
    path=Path(args.output)
    path.parent.mkdir(parents=True,exist_ok=True)
    out=path.open('w',buffering=1)
    stage={'name':'settle','input':[0.]*4}
    def log(kind,value):
        out.write(json.dumps({'sim_time':node.get_clock().now().nanoseconds*1e-9,
                             'wall_time':time.monotonic(),'stage':stage['name'],
                             'input':stage['input'],'kind':kind,'value':value})+'\n')
    for topic,kind in [('/amir2/ft_sensor',WrenchStamped),('/amir2/joint_states',JointState)]:
        node.create_subscription(kind,topic,
            lambda m,t=topic: log(t,message_to_ordereddict(m)),qos_profile_sensor_data)
    for _ in range(100):
        rclpy.spin_once(node,timeout_sec=.05)
        if node.get_clock().now().nanoseconds>0 and pub.get_subscription_count()>0:
            break
    if pub.get_subscription_count()==0:
        raise RuntimeError('calibration bridge is missing')
    def run(name,command,duration):
        stage['name'],stage['input']=name,command
        begin=node.get_clock().now().nanoseconds*1e-9
        wall=time.monotonic()
        while rclpy.ok() and node.get_clock().now().nanoseconds*1e-9-begin<duration:
            if time.monotonic()-wall>duration*10+5:
                raise RuntimeError('paused or too slow simulation')
            msg=Wrench()
            msg.force.x,msg.force.y,msg.force.z,msg.torque.z=map(float,command)
            pub.publish(msg)
            log('command',{})
            until=time.monotonic()+.04
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.005)
    try:
        run('settle',[0.]*4,2.)
        for axis,name in enumerate(('fx','fy','fz','tz')):
            for sign in (1,-1):
                run('bias_'+name+str(sign),[0.]*4,1.5)
                cmd=[0.]*4
                cmd[axis]=sign*(.5 if axis<3 else .05)
                run(name+str(sign),cmd,1.5)
        run('final',[0.]*4,1.)
    finally:
        pub.publish(Wrench())
        out.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
