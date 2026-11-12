"""Separate experiment supervisor and GT logger; no data fed into follower.

Leader motion is open-loop and bounded. Only enable/disable services reach the
follower. An external pulse is an alternative to a leader-motion trial.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Wrench, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from amir_follower_control.experiment_support import StationaryWindow, ExperimentGrasps


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output-directory',required=True)
    p.add_argument('--axis',choices=['fx','fy','fz','tz'],default='fx')
    p.add_argument('--stimulus',choices=['force','leader','hold'],default='force')
    p.add_argument('--delay',type=float,default=.0)
    p.add_argument('--duration',type=float,default=4.)
    p.add_argument('--speed',type=float,default=.005)
    p.add_argument('--acceleration',type=float,default=.001)
    p.add_argument('--polarity',type=int,choices=[-1,1],default=1)
    p.add_argument('--amplitude',type=float)
    p.add_argument('--wait-startup-s',type=float,default=30.)
    p.add_argument('--settle-seconds',type=float,default=1.)
    p.add_argument('--hold-arm-before-calibration',action='store_true')
    a=p.parse_args()
    if a.stimulus=='leader' and a.axis!='fx':
        raise ValueError('initial leader-motion trial is Fx only')
    if not 0<a.duration<=15 or not 0<=a.delay<=.5 or not 0<a.speed<=.02:
        raise ValueError('bounded low-speed experiment required')
    if not .00001<=a.acceleration<=.01:
        raise ValueError('bounded leader acceleration required')
    amplitude=a.amplitude if a.amplitude is not None else (.05 if a.axis=='tz' else .5)
    if not 0<amplitude<=(.1 if a.axis=='tz' else 1.) or not 0<=a.settle_seconds<=30:
        raise ValueError('bounded external wrench and settling interval required')
    out=Path(a.output_directory)
    out.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Node('local_follow_experiment',parameter_overrides=[Parameter('use_sim_time',value=True)])
    log=(out/'evaluation.jsonl').open('w',buffering=1)
    state=['UNKNOWN']
    support_removed=[False]
    stage=['startup']
    start_wall=time.monotonic()
    seen=set()
    last_recorded={}
    own_joints=[None]
    own_odom=[None]
    grasps=ExperimentGrasps()
    def now():
        return node.get_clock().now().nanoseconds*1e-9
    def record(topic,msg):
        seen.add(topic)
        if topic=='/amir2/joint_states':
            own_joints[0]=msg
        elif topic=='/amir2/mecanum_drive_controller/odometry':
            own_odom[0]=msg
        elif topic in ('/amir1/grasp/state','/amir2/grasp/state'):
            grasps.update(topic.split('/')[1],msg.data,now())
        # Startup can take several minutes. Preserve its context at 5 Hz,
        # then record every received sample throughout the actual trial.
        current=now()
        if stage[0]!='excitation' and topic!='follower/status':
            if current-last_recorded.get(topic,-float('inf'))<.2:
                return
        last_recorded[topic]=current
        log.write(json.dumps({'sim_time':now(),'wall_elapsed':time.monotonic()-start_wall,
                             'stage':stage[0],'topic':topic,
                             'message':message_to_ordereddict(msg)})+'\n')
    def on_status(msg):
        state[0]=msg.data
        record('follower/status',msg)
    node.create_subscription(String,'/amir2/follower/status',on_status,10)
    node.create_subscription(Bool,'/cooperative_transport/support_removed',
        lambda m:support_removed.__setitem__(0,m.data),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for robot in ('amir1','amir2'):
        for suffix,kind in [('ft_sensor',WrenchStamped),('joint_states',JointState),
                            ('finger_left_wrench',WrenchStamped),('finger_right_wrench',WrenchStamped),
                            ('mecanum_drive_controller/odometry',Odometry),('grasp/state',String)]:
            topic=f'/{robot}/{suffix}'
            node.create_subscription(kind,topic,lambda m,t=topic:record(t,m),qos_profile_sensor_data)
    node.create_subscription(JointTrajectory,'/amir2/arm_controller/joint_trajectory',
                             lambda m:record('follower_arm_command',m),10)
    node.create_subscription(Twist,'/amir2/rover_twist',
                             lambda m:record('follower_base_command',m),10)
    node.create_subscription(JointTrajectoryControllerState,'/amir2/arm_controller/controller_state',
                             lambda m:record('follower_arm_state',m),qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/experiment/model_poses',
        lambda m:record('world_GT',m),qos_profile_sensor_data)
    leader_pub=node.create_publisher(Twist,'/amir1/rover_twist',1)
    force_pub=node.create_publisher(Wrench,'/experiment/amir2/wrench_base',1)
    active_pub=node.create_publisher(Bool,'/cooperative_transport/active',1)
    calibrate=node.create_client(Trigger,'/amir2/follower/calibrate')
    enable=node.create_client(SetBool,'/amir2/follower/enable')
    def spin_until(predicate,timeout=15.):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and not predicate():
            if time.monotonic()>deadline:
                raise RuntimeError('experiment readiness timeout: '+state[0])
            rclpy.spin_once(node,timeout_sec=.02)
    def service(client,req):
        spin_until(client.service_is_ready)
        deadline=time.monotonic()+30.
        while True:
            future=client.call_async(req)
            spin_until(future.done)
            result=future.result()
            if result.success:
                return
            if (result.message=='audited controllers not ready; retry calibration after discovery'
                    and time.monotonic()<deadline):
                wait(.2)
                continue
            raise RuntimeError(result.message)
    def wait(duration):
        begin=now()
        spin_until(lambda:now()-begin>=duration,timeout=max(10.,duration*20.))
    enabled=False
    result={'axis':a.axis,'stimulus':a.stimulus,'delay_s':a.delay,
            'speed_mps':a.speed,'acceleration_mps2':a.acceleration,
            'force_amplitude_N_or_Nm':a.polarity*amplitude if a.stimulus=='force' else 0.,
            'duration_s':a.duration,'success':False}
    try:
        spin_until(lambda:state[0]!='UNKNOWN' and now()>0 and support_removed[0] and grasps.ready(now()),
                   timeout=a.wait_startup_s)
        stage[0]='handoff'
        active_pub.publish(Bool(data=True))
        wait(.5)
        if a.hold_arm_before_calibration:
            spin_until(lambda:own_joints[0] is not None)
            hold=node.create_publisher(JointTrajectory,'/amir2/arm_controller/joint_trajectory',1)
            wait(.2)
            mapping=dict(zip(own_joints[0].name,own_joints[0].position))
            msg=JointTrajectory()
            msg.header.stamp=node.get_clock().now().to_msg()
            msg.joint_names=[f'Joint_{i}' for i in range(1,6)]
            point=JointTrajectoryPoint(positions=[mapping[n] for n in msg.joint_names],velocities=[0.]*5)
            point.time_from_start.nanosec=100000000
            msg.points=[point]
            hold.publish(msg)
            wait(.3)
            node.destroy_publisher(hold)
            wait(.3)
        stage[0]='settling'
        stationary=StationaryWindow()
        spin_until(lambda:stationary.ready(now(),own_joints[0],own_odom[0]),timeout=30.)
        stage[0]='calibrating'
        service(calibrate,Trigger.Request())
        spin_until(lambda:state[0] in ('READY','FAULT'),timeout=30.)
        if state[0]=='FAULT':
            raise RuntimeError('follower FAULT during calibration')
        stage[0]='pre-motion'
        wait(.5)
        start=now()
        result['excitation_start_sim_time']=start
        stage[0]='excitation'
        seen.clear()
        while now()-start<a.duration+a.settle_seconds:
            elapsed=now()-start
            if not grasps.ready(now()):
                raise RuntimeError('experiment local grasp heartbeat missing, stale or detached')
            if not enabled and elapsed>=a.delay:
                service(enable,SetBool.Request(data=True))
                enabled=True
                result['follower_enable_sim_time']=now()
            if state[0]=='FAULT':
                raise RuntimeError('follower FAULT during excitation')
            command=Twist()
            force=Wrench()
            if elapsed<a.duration:
                if a.stimulus=='leader':
                    command.linear.x=float(min(a.speed,elapsed*a.acceleration,
                                               (a.duration-elapsed)*a.acceleration))
                elif a.stimulus=='force':
                    force_value=a.polarity*amplitude*min(1.,elapsed/.2)
                    if a.axis=='tz':
                        force.torque.z=force_value
                    else:
                        setattr(force.force,{'fx':'x','fy':'y','fz':'z'}[a.axis],force_value)
            leader_pub.publish(command)
            force_pub.publish(force)
            record('leader_command',command)
            record('external_force_command',force)
            wait(.02)
        if 'world_GT' not in seen:
            raise RuntimeError('world GT logger received no data; trial incomplete')
        result['success']=True
        result['final_state']=state[0]
        result['sim_duration_s']=now()-start
    except (Exception,KeyboardInterrupt) as exc:
        result['failure']=str(exc) or 'experiment interrupted'
        result['final_state']=state[0]
    finally:
        result['end_sim_time']=now()
        # SIGINT may have already shut the ROS context down. Always preserve
        # the result even when publishing stop or calling disable is unavailable.
        try:
            if rclpy.ok():
                leader_pub.publish(Twist())
                force_pub.publish(Wrench())
            if rclpy.ok() and enable.service_is_ready():
                future=enable.call_async(SetBool.Request(data=False))
                spin_until(future.done,timeout=3.)
                wait(.5)
        except Exception:
            pass
        (out/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result['success'] else 1)


if __name__=='__main__':
    main()
