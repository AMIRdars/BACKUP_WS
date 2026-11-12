#!/usr/bin/env python3
"""CP-9 guarded preparation, derived from amir_cp9_local_grasp.py.

Explicit writer release -> native monitor takeover -> retire -> process exit.
The original static commissioning tool remains unchanged.

All force policies below are inherited simulation experiment settings from
friction_transport.yaml, not hardware ratings. Arm home is local preparation;
there are no base or transport commands.
Contact Bool comes from a separate simulator contact aggregator.
"""
import json
import math
import os
import hashlib
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def home_from_urdf(path):
    root=ET.parse(path).getroot()
    plugins=root.findall('ros2_control/hardware/plugin')
    if not plugins or any(v.text!='gz_ros2_control/GazeboSimSystem' for v in plugins):
        raise ValueError('only audited Gazebo hardware is allowed for this preparation tool')
    home={}
    for joint in root.findall('ros2_control/joint'):
        param=joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if joint.get('name')=='Gripper' or joint.find("param[@name='mimic']") is not None or param is None or joint.find("command_interface[@name='position']") is None:continue
        value=float(param.text)
        limit=root.find("joint[@name='"+joint.get('name')+"']/limit")
        if limit is None or not math.isfinite(value) or not float(limit.get('lower'))<=value<=float(limit.get('upper')):
            raise ValueError('invalid model-derived home')
        home[joint.get('name')]=value
    if len(home)!=5:raise ValueError('expected audited five arm joints')
    return home


def position_step(position, forces, bilateral, had_contact):
    """Bounded 20 N simulation search; inherited .002/.0002/.006 rad steps."""
    if len(forces)!=2 or not all(math.isfinite(v) and v>=0 for v in forces):
        raise ValueError('two finite nonnegative finger forces required')
    if not math.isfinite(position) or not -1.047198<=position<=.261799:
        raise ValueError('measured gripper outside model range')
    if max(forces)>26:
        delta=max(-.006,.0001*(20-max(forces)))
    elif not bilateral:
        delta=.0002 if had_contact else .002
    elif min(forces)<17:
        delta=min(.002,max(0.,.0001*max(20-sum(forces)/2,17-min(forces))))
    elif max(forces)>23:
        # HOLDING requires EACH finger in [17,23] N. An average below
        # 23 N must not leave an overloaded finger frozen in CLOSING.
        delta=max(-.002,.0001*(20-sum(forces)/2))
    else:
        delta=0.
    target=max(-1.,min(.261799,position+delta))
    return max(position-.006,min(position+.002,target))


def mimic_measurement_ok(master,values,specs):
    return master is not None and math.isfinite(master) and bool(specs) and set(values)==set(specs) and all(
        math.isfinite(q) and specs[n]['lower']-1e-5<=q<=specs[n]['upper']+1e-5 and
        abs(q-(master*specs[n]['multiplier']+specs[n]['offset']))<=.01 for n,q in values.items())


def main():
    if os.getenv('ROS_LOCALHOST_ONLY')!='1' or not os.getenv('IGN_PARTITION','').startswith('amir_cp9_'):
        raise ValueError('isolated CP-9 simulation environment required')
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from control_msgs.action import GripperCommand
    from controller_manager_msgs.srv import SwitchController
    from geometry_msgs.msg import WrenchStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool,String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

    class LocalGrasp(Node):
        def __init__(self):
            super().__init__('cp9_contact_preparation')
            if self.get_namespace() not in ('/leader','/follower') or not self.get_parameter('use_sim_time').value:
                raise ValueError('explicit simulated leader/follower namespace required')
            self.state='IDLE';self.reason='';self.q=None;self.qtime=0.;self.data={};self.started=0.;self.stable=None
            self.home=home_from_urdf(self.declare_parameter('urdf_file','').value);self.arm_q={}
            model=ET.parse(self.get_parameter('urdf_file').value).getroot()
            self.mimics={j.get('name')+'_mimic':{'multiplier':float(j.find('mimic').get('multiplier','1')),
                'offset':float(j.find('mimic').get('offset','0')),'lower':float(j.find('limit').get('lower')),
                'upper':float(j.find('limit').get('upper'))} for j in model.findall('joint') if j.find('mimic') is not None and j.find('mimic').get('joint')=='Gripper'}
            self.measured_mimics={}
            if len(self.mimics)!=5:raise ValueError('expected five audited gripper mimic states')
            self.arm_limits={n:(float(model.find("joint[@name='"+n+"']/limit").get('lower')),
                float(model.find("joint[@name='"+n+"']/limit").get('upper'))) for n in self.home}
            limit=model.find("joint[@name='Gripper']/limit")
            self.gripper_limits=(float(limit.get('lower')),float(limit.get('upper')))
            self.arm_pub=self.create_publisher(JointTrajectory,'arm_controller/joint_trajectory',10)
            self.loss=None;self.command=None;self.had_contact=False;self.last_goal=-1.;self.last_clock=time.monotonic();self.last_sim=-1.
            self.qstamp=0.;self.sent_goals=0;self.accepted_goals=0
            self.source_hash=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            self.client=ActionClient(self,GripperCommand,'gripper_controller/gripper_cmd')
            self.manager=self.create_client(SwitchController,'controller_manager/switch_controller')
            self.stop_result=None;self.first_fault_context={}
            self.native_monitor=None;self.native_monitor_wall=0.
            self.create_subscription(String,'grasp/monitor_debug',self.monitor,10)
            self.state_pub=self.create_publisher(String,'grasp/state',10)
            self.debug_pub=self.create_publisher(String,'grasp/debug',10)
            self.stop_pub=self.create_publisher(Bool,'/transport/emergency_stop',
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(JointState,'joint_states',self.joints,qos_profile_sensor_data)
            self.create_subscription(Bool,'/transport/emergency_stop',lambda m:self.fail('shared stop') if m.data and self.state!='FAULT' else None,10)
            for side in ('left','right'):
                self.create_subscription(Bool,'finger_'+side+'_contact_detected',lambda m,s=side:self.receive(s,'contact',m.data),10)
                self.create_subscription(WrenchStamped,'finger_'+side+'_wrench',lambda m,s=side:self.force(s,m),qos_profile_sensor_data)
            self.create_service(Trigger,'grasp/close',self.close)
            self.create_service(Trigger,'grasp/prepare',self.prepare)
            self.create_service(Trigger,'grasp/release_arm_writer',self.release_writer)
            self.create_service(Trigger,'grasp/retire_preparation',self.retire)
            self.timer=self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def monitor(self,msg):
            try:self.native_monitor=json.loads(msg.data);self.native_monitor_wall=time.monotonic()
            except (ValueError,TypeError):self.native_monitor=None

        def release_writer(self,request,response):
            response.success=self.state=='HOLDING' and self.ready() and self.arm_healthy() and self.mimic_healthy() and self.arm_pub is not None
            if response.success:
                self.destroy_publisher(self.arm_pub);self.arm_pub=None
            response.message='arm writer released; grip watchdog remains active' if response.success else self.state
            return response

        def retire(self,request,response):
            writers=self.get_publishers_info_by_topic(self.get_namespace()+'/grasp/monitor_debug')
            own=len(writers)==1 and writers[0].node_name=='native_grasp_monitor' and writers[0].node_namespace==self.get_namespace()
            response.success=(own and self.state=='HOLDING' and self.arm_pub is None and self.ready()
                and self.arm_healthy() and self.mimic_healthy() and self.native_monitor is not None
                and time.monotonic()-self.native_monitor_wall<.25 and self.native_monitor.get('healthy')
                and self.native_monitor.get('state')=='HOLDING')
            if response.success:
                self.state='RELEASED';self.reason='explicit retirement after live local monitor takeover'
                self.timer.cancel()
                self.debug_pub.publish(String(data=json.dumps({'state':self.state,'reason':self.reason,
                    'sim_time':self.now_s(),'source_sha256':self.source_hash,'arm_writer_released':True,
                    'replacement_monitor_holding':True})))
            response.message=self.reason if response.success else 'released writer and fresh local HOLDING monitor required'
            return response

        def now_s(self):return self.get_clock().now().nanoseconds/1e9

        def receive(self,side,kind,value,stamp=None):self.data[side,kind]=(value,self.now_s() if stamp is None else stamp,time.monotonic())

        def force(self,side,msg):
            w=msg.wrench;values=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.receive(side,'force',abs(values[0]) if all(math.isfinite(v) for v in values) else float('nan'),stamp)
            if self.state in ('HOMING','READY_TO_CLOSE','CLOSING','HOLDING') and not all(math.isfinite(v) for v in values):self.fail('nonfinite local force')
            elif self.state in ('HOMING','READY_TO_CLOSE','CLOSING','HOLDING') and abs(values[0])>26:self.fail('simulation grip overforce')

        def joints(self,msg):
            positions=dict(zip(msg.name,msg.position));self.measured_mimics={n:positions[n] for n in self.mimics if n in positions}
            self.arm_q={name:value for name,value in zip(msg.name,msg.position) if name in self.home}
            if 'Gripper' in msg.name:
                self.q=msg.position[msg.name.index('Gripper')];self.qtime=time.monotonic()
                self.qstamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if self.state in ('HOMING','READY_TO_CLOSE','CLOSING','HOLDING') and (not self.arm_healthy() or not self.mimic_healthy()):self.fail('local joint limit/tracking fault')

        def arm_healthy(self):
            return set(self.arm_q)==set(self.arm_limits) and all(math.isfinite(q) and self.arm_limits[n][0]-1e-5<=q<=self.arm_limits[n][1]+1e-5 for n,q in self.arm_q.items())

        def mimic_healthy(self):
            return self.q is not None and self.gripper_limits[0]-1e-5<=self.q<=self.gripper_limits[1]+1e-5 and mimic_measurement_ok(self.q,self.measured_mimics,self.mimics)

        def ready(self):
            now=self.now_s();wall=time.monotonic()
            return self.q is not None and math.isfinite(self.q) and self.qstamp>0 and -.02<=now-self.qstamp<=.25 and wall-self.qtime<=.25 and self.client.server_is_ready() and self.manager.service_is_ready() and all(
                k in self.data and self.data[k][1]>0 and -.02<=now-self.data[k][1]<=.25 and wall-self.data[k][2]<=.5
                for k in ((s,k) for s in ('left','right') for k in ('contact','force')))

        def prepare(self,request,response):
            response.success=self.state=='IDLE' and self.ready() and self.mimic_healthy() and len(self.arm_q)==5 and all(math.isfinite(v) for v in self.arm_q.values()) and max(abs(v-self.arm_q[n]) for n,v in self.home.items())*.5<=.03
            if response.success:
                self.state='HOMING';self.started=self.now_s();self.command=self.q
                message=JointTrajectory();message.header.stamp=self.get_clock().now().to_msg();message.joint_names=list(self.home)
                point=JointTrajectoryPoint();point.positions=list(self.home.values());point.velocities=[0.]*5;point.time_from_start.sec=3
                message.points=[point];self.arm_pub.publish(message)
            response.message=self.state;return response

        def close(self,request,response):
            response.success=self.state=='READY_TO_CLOSE' and self.ready() and self.mimic_healthy()
            if response.success:self.state='CLOSING';self.started=self.now_s();self.command=self.q
            response.message=self.state;return response

        def goal(self,position):
            goal=GripperCommand.Goal();goal.command.position=float(position);goal.command.max_effort=1.2
            self.sent_goals+=1
            future=self.client.send_goal_async(goal)
            future.add_done_callback(self.accepted)

        def accepted(self,future):
            try:
                if not future.result().accepted:self.fail('gripper action rejected')
                else:self.accepted_goals+=1
            except Exception as error:self.fail('gripper action: '+str(error))

        def fail(self,reason):
            if self.state in ('FAULT','RELEASED'):return
            now=self.now_s();wall=time.monotonic()
            self.first_fault_context={'sim_time':now,'previous_state':self.state,
                'joint_source_age_s':now-self.qstamp,'joint_receipt_age_wall_s':wall-self.qtime,
                'source_ages_s':{s+'_'+k:now-v[1] for (s,k),v in self.data.items()},
                'receipt_ages_wall_s':{s+'_'+k:wall-v[2] for (s,k),v in self.data.items()},
                'clock_age_wall_s':wall-self.last_clock,'gripper_action_ready':self.client.server_is_ready(),
                'controller_service_ready':self.manager.service_is_ready()}
            self.state='FAULT';self.reason=reason
            self.stop_pub.publish(Bool(data=True))
            request=SwitchController.Request();request.deactivate_controllers=['arm_controller','mecanum_drive_controller']
            request.strictness=SwitchController.Request.BEST_EFFORT;request.timeout.sec=1
            future=self.manager.call_async(request)
            future.add_done_callback(self.stopped)
            # Retain the last bounded position; never open on shared stop.
            # Actuator/power-stop verification is a separate requirement.

        def stopped(self,future):
            try:self.stop_result=bool(future.result().ok)
            except Exception as error:self.stop_result=str(error)

        def tick(self):
            now=self.now_s();wall=time.monotonic()
            if now>self.last_sim:self.last_sim=now;self.last_clock=wall
            if self.state in ('HOMING','READY_TO_CLOSE','CLOSING','HOLDING'):
                if wall-self.last_clock>.5 or not self.ready():self.fail('local sensor/clock freshness')
                elif not self.arm_healthy() or not self.mimic_healthy():self.fail('local joint limit/tracking fault')
                else:
                    forces=[self.data[s,'force'][0] for s in ('left','right')]
                    contacts=[self.data[s,'contact'][0] for s in ('left','right')]
                    bilateral=all(contacts)
                    if not all(math.isfinite(f) for f in forces):self.fail('nonfinite local force')
                    elif max(forces)>26:self.fail('simulation grip overforce')
                    elif self.state=='HOMING':
                        if now-self.started>=3 and len(self.arm_q)==5 and max(abs(v-self.arm_q[n]) for n,v in self.home.items())<.0001:
                            self.state='READY_TO_CLOSE';self.started=now
                        elif now-self.started>5:self.fail('local home failed to converge')
                    elif self.state=='READY_TO_CLOSE':pass
                    elif self.state=='CLOSING':
                        self.had_contact=self.had_contact or bilateral
                        reference=self.command if self.had_contact else self.q
                        self.command=position_step(reference,forces,bilateral,self.had_contact)
                        self.goal(self.command)
                        good=bilateral and all(17<=f<=23 for f in forces)
                        self.stable=now if good and self.stable is None else self.stable if good else None
                        if self.stable is not None and now-self.stable>=.3:self.state='HOLDING'
                        elif now-self.started>45:self.fail('no stable bilateral grasp within simulation envelope')
                    elif not bilateral or min(forces)<8:
                        self.loss=now if self.loss is None else self.loss
                        if now-self.loss>=.3:self.fail('local contact/holding force lost')
                    else:self.loss=None
            self.state_pub.publish(String(data='attached' if self.state=='HOLDING' else 'detached'))
            self.debug_pub.publish(String(data=json.dumps({'state':self.state,'reason':self.reason,'sim_time':now,'q':self.q,
                'local_ready':self.ready() and self.arm_healthy() and self.mimic_healthy(),
                'command':self.command,'arm_q':self.arm_q,'home_from_actual_urdf':self.home,'own_sensors':{s+'_'+k:v[0] for (s,k),v in self.data.items()},
                'mimic_healthy':self.mimic_healthy(),'measured_mimics':self.measured_mimics,
                'sent_gripper_goals':self.sent_goals,'accepted_gripper_goals':self.accepted_goals,'source_sha256':self.source_hash,
                'base_arm_deactivation_response':self.stop_result,
                'first_fault_context':self.first_fault_context,
                'simulation_only':True,'normal_force_target_N':20.,'force_limits_hardware_verified':False})))

    rclpy.init(args=None);node=LocalGrasp()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
