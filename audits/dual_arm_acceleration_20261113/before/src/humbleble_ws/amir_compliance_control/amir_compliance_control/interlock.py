"""Local final actuator watchdog and separate controller/graph audit process.

No leader state, goal, TF, simulator or ground truth dependencies.
"""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray, GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .safety import LATCHED
from .wrench import LocalModel, finite
from .sensor_coherence import local_sensor_qos


def wall_timer(node, period, callback):
    return node.create_timer(period,callback,clock=Clock(clock_type=ClockType.STEADY_TIME))


class ControllerAudit(Node):
    """Discovery/service work is isolated from final actuator timer callbacks."""
    def __init__(self, **kwargs):
        super().__init__('controller_audit', **kwargs)
        self.robot = self.get_namespace().strip('/')
        if self.robot not in ('leader','follower'): raise ValueError('local role namespace required')
        self.declare_parameter('request_writer_name', '')
        self.declare_parameter('require_gripper_active', False)
        self.request_writer_name = self.get_parameter('request_writer_name').value
        self.required_controllers = ['joint_state_broadcaster','mecanum_drive_controller','arm_controller']
        if self.get_parameter('require_gripper_active').value:
            self.required_controllers.append('gripper_controller')
        self.client = self.create_client(ListControllers,'controller_manager/list_controllers')
        self.future, self.controller_wall, self.controllers = None, 0., {}
        self.request_wall=0.
        self.action_active = False
        self.live_action = False
        self.permit = False
        self.create_subscription(GoalStatusArray,'arm_controller/follow_joint_trajectory/_action/status',
                                 self.action,10)
        self.create_subscription(Bool,'/transport/safety_permit',lambda m:setattr(self,'permit',m.data),1)
        self.create_service(Trigger,'controller_audit/reset',self.reset)
        self.pub = self.create_publisher(String,'interlock/audit',10)
        self.timer = wall_timer(self,.1,self.tick)

    def action(self,msg):
        self.live_action = any(v.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                              GoalStatus.STATUS_CANCELING) for v in msg.status_list)
        self.action_active = self.action_active or self.live_action

    def reset(self,request,response):
        response.success = not self.permit and not self.live_action
        if response.success:self.action_active=False
        response.message='audit reset; explicit rearm required' if response.success else 'disarm and cancel action first'
        return response

    def tick(self):
        if self.future is not None and not self.future.done() and time.monotonic()-self.request_wall>.5:
            self.client.remove_pending_request(self.future)
            self.future.cancel()
            self.future=None
        if self.future is not None and self.future.done():
            try:
                self.controllers = {c.name:c.state for c in self.future.result().controller}
                self.controller_wall = time.monotonic()
            except Exception: self.controllers = {}
            self.future = None
        if self.future is None and self.client.service_is_ready():
            self.future = self.client.call_async(ListControllers.Request())
            self.request_wall=time.monotonic()
        ready = (time.monotonic()-self.controller_wall < .5 and all(self.controllers.get(n)=='active'
                  for n in self.required_controllers))
        reason = '' if ready else 'controller state missing/stale/inactive'
        if self.action_active: ready,reason = False,'external arm action is active'
        for suffix,expected in [('rover_twist','actuator_interlock'),('arm_controller/joint_trajectory','actuator_interlock'),
                ('interlock/base_request','safety_gate'),('interlock/arm_request','safety_gate'),
                ('mecanum_drive_controller/reference_unstamped','rover_twist_relay_ign')]:
            writers = self.get_publishers_info_by_topic('/'+self.robot+'/'+suffix)
            if len(writers)!=1 or writers[0].node_name!=expected or writers[0].node_namespace!='/'+self.robot:
                ready,reason = False,'command publisher ownership changed: '+suffix
        # The installed mecanum controller also accepts stamped references.
        if self.get_publishers_info_by_topic('/'+self.robot+'/mecanum_drive_controller/reference'):
            ready,reason = False,'unexpected stamped base reference publisher'
        if self.request_writer_name:
            for suffix in ('base_cmd', 'arm_joint_cmd'):
                writers = self.get_publishers_info_by_topic('/'+self.robot+'/'+suffix)
                if (len(writers) != 1 or writers[0].node_name != self.request_writer_name
                        or writers[0].node_namespace != '/'+self.robot):
                    ready,reason = False,'command request ownership changed: '+suffix
        self.pub.publish(String(data=json.dumps({'robot':self.robot,'ready':ready,'reason':reason,
            'controllers':self.controllers,'action_active':self.action_active,
            'required_controllers':self.required_controllers,
            'request_writer_name':self.request_writer_name})))


class ActuatorInterlock(Node):
    def __init__(self, **kwargs):
        super().__init__('actuator_interlock', **kwargs)
        self.robot = self.get_namespace().strip('/')
        if self.robot not in ('leader','follower') or not self.get_parameter('use_sim_time').value:
            raise ValueError('local role namespace and sim clock required')
        self.declare_parameter('urdf_file','')
        self.declare_parameter('profile_file','')
        self.declare_parameter('base_zero_only', False)
        self.declare_parameter('simulation_source_watchdog',False)
        self.source_watchdog=self.get_parameter('simulation_source_watchdog').value
        self.joint_stamp=0.
        self.base_zero_only = self.get_parameter('base_zero_only').value
        if not isinstance(self.base_zero_only, bool): raise ValueError('boolean base_zero_only required')
        self.model = LocalModel(Path(self.get_parameter('urdf_file').value).read_text())
        profile=json.loads(Path(self.get_parameter('profile_file').value).read_text())
        self.max_base=finite(profile['max_base_velocity'],(3,))
        self.margin=float(profile['joint_margin_rad'])
        self.gate,self.audit,self.q,self.hold = None,None,None,None
        self.gate_wall=self.audit_wall=self.joint_wall=self.clock_wall=self.base_wall=self.relay_wall=0.
        self.permit,self.emergency,self.ever,self.active = False,True,False,False
        self.permit_wall=0.;self.fault='';self.sequence=0;self.sim=0.
        self.command=Twist()
        self.pending_arm=None;self.pending_arm_wall=0.;self.arm_deliveries=0
        self.base_pub=self.create_publisher(Twist,'rover_twist',1)
        self.arm_pub=self.create_publisher(JointTrajectory,'arm_controller/joint_trajectory',1)
        self.status_pub=self.create_publisher(String,'/transport/interlock/'+self.robot,10)
        self.debug_pub=self.create_publisher(String,'interlock/debug',10)
        self.create_subscription(String,'/transport/safety/'+self.robot,self.gate_status,10)
        self.create_subscription(String,'interlock/audit',self.audit_status,10)
        self.create_subscription(JointState,'joint_states',self.joints,local_sensor_qos(self.source_watchdog))
        self.create_subscription(ClockMessage,'/clock',self.clock,local_sensor_qos(self.source_watchdog))
        self.create_subscription(Bool,'/transport/safety_permit',self.allow,1)
        self.create_subscription(Bool,'/transport/emergency_stop',self.stop,LATCHED)
        self.create_subscription(Twist,'interlock/base_request',self.base,1)
        self.create_subscription(JointTrajectory,'interlock/arm_request',self.arm,1)
        # Observe fresh relay deliveries without reliable-reader retransmission
        # backlog. The actuator writer/driver QoS and 150 ms watchdog stay intact.
        self.create_subscription(Twist,'mecanum_drive_controller/reference_unstamped',
                                 lambda m:setattr(self,'relay_wall',time.monotonic()),qos_profile_sensor_data)
        expected={'/clock','/transport/safety/'+self.robot,'/transport/safety_permit','/transport/emergency_stop',
            *['/'+self.robot+'/'+s for s in ('interlock/audit','joint_states','interlock/base_request',
                                           'interlock/arm_request','mecanum_drive_controller/reference_unstamped')]}
        if {s.topic_name for s in self.subscriptions}!=expected: raise ValueError('interlock local boundary remapped')
        self.create_service(Trigger,'interlock/reset',self.reset)
        self.timer=wall_timer(self,.02,self.tick)

    def trip(self,reason):
        if self.ever and not self.fault: self.fault=reason
        self.command=Twist()
        self.pending_arm=None

    def gate_status(self,msg):
        try:
            value=json.loads(msg.data)
            if (value['robot']!=self.robot or not isinstance(value['ready'],bool)
                    or (self.gate is not None and value['sequence']<=self.gate['sequence'])):
                raise ValueError('invalid local gate heartbeat')
            self.gate,self.gate_wall=value,time.monotonic()
        except (ValueError,KeyError,TypeError): self.trip('invalid local gate heartbeat')

    def audit_status(self,msg):
        try:
            value=json.loads(msg.data)
            if value['robot']!=self.robot or not isinstance(value['ready'],bool):raise ValueError()
            self.audit,self.audit_wall=value,time.monotonic()
        except (ValueError,KeyError,TypeError): self.trip('invalid controller audit')

    def clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if value<self.sim:self.trip('clock reversed at actuator interlock')
        if value>self.sim:self.clock_wall=time.monotonic()
        self.sim=value

    def joints(self,msg):
        try:
            q=dict(zip(msg.name,msg.position))
            q=finite([q[n] for n in self.model.names],(5,))
            if np.any(q<self.model.lower) or np.any(q>self.model.upper):raise ValueError()
            self.q=q
            self.joint_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.joint_wall=time.monotonic()
        except (ValueError,KeyError):self.trip('invalid local joints at actuator interlock')

    def allow(self,msg):
        rising=msg.data and not self.permit
        self.permit,self.permit_wall=msg.data,time.monotonic()
        if rising:
            self.ever=True;self.command=Twist();self.hold=None
            self.pending_arm=None

    def stop(self,msg):
        self.emergency=msg.data
        if msg.data:self.command=Twist();self.pending_arm=None

    def healthy(self):
        now=time.monotonic()
        if self.gate is None or now-self.gate_wall>.15:return False,'gate heartbeat missing at actuator interlock'
        if not self.gate['ready'] or self.gate['fault']:return False,'local safety gate not ready'
        if self.audit is None or now-self.audit_wall>.3:return False,'controller audit heartbeat missing'
        if not self.audit['ready']:return False,self.audit['reason']
        ages={k:now-getattr(self,k+'_wall') for k in ('joint','clock','base','relay')}
        missing=[k for k,v in ages.items() if v>.15 and not (k=='joint' and self.source_watchdog)]
        if self.source_watchdog and not -.02<=self.sim-self.joint_stamp<=.05:
            return False,'local joint source timestamp stale/future'
        if self.q is None or missing:
            return False,'local actuator heartbeat missing: '+','.join(missing)
        return True,''

    def allowed(self):
        return self.permit and not self.emergency and not self.fault and time.monotonic()-self.permit_wall<=.15

    def base(self,msg):
        self.base_wall=time.monotonic()
        try:
            v=finite([msg.linear.x,msg.linear.y,msg.angular.z],(3,))
            if self.base_zero_only and np.any(v):
                self.trip('ARM_ONLY rejected nonzero final base request');return
            if np.any(np.abs(v)>self.max_base+1e-9) or any(abs(v)>1e-12 for v in
                    finite([msg.linear.z,msg.angular.x,msg.angular.y],(3,))):raise ValueError()
        except ValueError:
            self.trip('invalid final base request');return
        if self.allowed() and self.gate is not None and self.gate.get('active'):
            self.command=msg
        else:self.command=Twist()

    def arm(self,msg):
        # A disarmed gate publishes zero-stamped holds. The final interlock
        # generates its own measured hold; these are never pending live commands.
        if msg.header.stamp.sec==0 and msg.header.stamp.nanosec==0:return
        if self.allowed():
            try:
                if tuple(msg.joint_names)!=self.model.names or not msg.points:raise ValueError()
                for p in msg.points:
                    q=finite(p.positions,(5,))
                    if np.any(q<=self.model.lower+self.margin) or np.any(q>=self.model.upper-self.margin):raise ValueError()
                self.pending_arm,self.pending_arm_wall=msg,time.monotonic()
                # A healthy, active final writer need not add another 20 ms
                # timer period after the upstream gate's own timer. Keep the
                # same freshness/expiry checks and retain pre-ack one-shots.
                if self.healthy()[0] and self.gate.get('active',False):
                    self.deliver_arm()
            except ValueError:self.trip('invalid final arm request')

    def deliver_arm(self):
        msg=self.pending_arm
        end=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        end+=msg.points[-1].time_from_start.sec+msg.points[-1].time_from_start.nanosec*1e-9
        if time.monotonic()-self.pending_arm_wall>.15 or end<=self.sim:
            self.trip('pending arm trajectory expired at final interlock')
            return False
        self.arm_pub.publish(msg);self.arm_deliveries+=1;self.pending_arm=None
        return True

    def reset(self,request,response):
        good,reason=self.healthy()
        response.success=good and not self.permit
        if response.success:self.fault='';self.ever=False;self.command=Twist();self.pending_arm=None
        response.message='explicit arm required' if response.success else reason or 'disarm first'
        return response

    def tick(self):
        good,reason=self.healthy()
        if self.ever and not good:self.trip(reason)
        if self.ever and self.permit and time.monotonic()-self.permit_wall>.15:self.trip('shared permit heartbeat missing')
        active=good and self.allowed() and self.gate.get('active',False)
        if active and self.pending_arm is not None:
            active=self.deliver_arm()
        self.base_pub.publish(self.command if active else Twist())
        if not active and self.q is not None:
            if self.hold is None or self.active:self.hold=self.q.copy()
            point=JointTrajectoryPoint(positions=self.hold.tolist());point.time_from_start.nanosec=100000000
            self.arm_pub.publish(JointTrajectory(joint_names=list(self.model.names),points=[point]))
        self.active=active;self.sequence+=1
        value={'robot':self.robot,'ready':good,'fault':self.fault,'reason':reason,'active':active,'sequence':self.sequence,
               'base_zero_only':self.base_zero_only,
               'request_writer_name':(self.audit or {}).get('request_writer_name','')}
        self.status_pub.publish(String(data=json.dumps(value)))
        value['ages_wall_s']={k:time.monotonic()-getattr(self,k+'_wall') for k in ('joint','clock','base','relay','gate','audit')}
        value['arm_deliveries']=self.arm_deliveries
        self.debug_pub.publish(String(data=json.dumps(value)))


def run(kind):
    rclpy.init();node=kind()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()


def interlock_main():run(ActuatorInterlock)
def audit_main():run(ControllerAudit)
