"""Chapter 23 opt-in, observer-gated A/B/C and whole-body follower.

Legacy straight nodes stay intact. This node is the sole arm/base writer in
the integrated launch; it inherits the existing wrench and shared stop path.
"""
import json
import time
import hashlib
from collections import deque,OrderedDict
from pathlib import Path
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .admittance import rotation_error
from .allocation_budget import TaskComplianceBudget
from .local_yaw import ArmSoftTask
from .rotation_observer import ProjectedYaw,wrap_pi
from .rotation_recentering import MODES, RotationRecentering, whole_body_task
from .safety import stamp
from .wrench import finite
from .yaw_arm_base import YawArmBase
from .yaw_whole_body import YawWholeBody
from .whole_body import WholeBodyAllocator


def own_geometry(node,msg):
    """Memoize immutable own source samples, never a goal or prior result."""
    cache=getattr(node,'yaw_geometry_cache',None)
    if cache is None:node.yaw_geometry_cache=cache=OrderedDict()
    key=(round(stamp(msg)*1e9),tuple(zip(msg.name,msg.position)))
    if key not in cache:
        mapping=dict(zip(msg.name,msg.position))
        q=finite([mapping[n] for n in node.model.names],(5,))
        cache[key]=node.yaw_geometry.measure(q,compute_jacobian=False)
        if len(cache)>64:cache.popitem(last=False)
    return cache[key]


class RotationFollower(YawWholeBody):
    def __init__(self,**kwargs):
        self.yaw_joint_history=deque(maxlen=32)
        super().__init__(**kwargs)
        self.role=self.mode
        self.declare_parameter('arm_yaw_admittance_enabled',False)
        self.declare_parameter('passive_yaw_calibration',False)
        self.passive_calibration=bool(self.p('passive_yaw_calibration'))
        # Straight XY transport keeps its existing enable contract. A rotation
        # ablation can require the same qualified observation/reference as B/C
        # while deliberately commanding zero base yaw in A.
        self.declare_parameter('rotation_observation_required',False)
        self.rotation_observation_required=bool(self.p('rotation_observation_required'))
        if self.passive_xy_allocation and (not self.passive_calibration or self.mode!='XY_ONLY'
                or self.rotation_observation_required or not self.p('arm_yaw_admittance_enabled')
                or max(self.p('base_velocity_limits')[:2])>.0001):
            raise ValueError('passive XY allocation requires calibration-only XY_ONLY, arm compliance and stationary-envelope XY limits')
        self.declare_parameter('allocation_task_compliance_budget',False)
        use_task_budget=bool(self.p('allocation_task_compliance_budget'))
        if use_task_budget and self.mode!='WHOLE_BODY' and not self.passive_xy_allocation:
            raise ValueError('task compliance authority requires the sole WHOLE_BODY allocator')
        if self.passive_xy_allocation and not use_task_budget:
            raise ValueError('passive XY allocation requires the existing bounded task compliance budget')
        self.task_compliance_budget=(TaskComplianceBudget(
            [*self.p('displacement_limit_m'),self.p('yaw_displacement_limit_rad')],self.p('base_cutoff_hz'))
            if use_task_budget else None)
        self.passive_calibration=bool(self.p('passive_yaw_calibration'))
        if self.passive_calibration and (self.mode!='XY_ONLY' or np.any(np.array(self.p('base_gains'))!=0.)):
            raise ValueError('passive calibration requires XY_ONLY and zero base recentering gains')
        self.declare_parameter('yaw_heading_frame','tcp_link')
        xml=Path(self.p('urdf_file')).read_text()
        self.urdf_hash=hashlib.sha256(xml.encode()).hexdigest()
        self.yaw_geometry=ProjectedYaw(xml,orientation_envelope_rad=self.p('rotation_error_limit_rad'),
                                      joint_margin_rad=self.p('joint_margin_rad'),heading_frame=self.p('yaw_heading_frame'))
        self.observation,self.observation_wall={},0.
        self.observation_reference=None
        # Joint positions and odometry are snapshots. Queueing five old
        # samples at 250 Hz can put our FK behind the independent observer.
        # Raw/filtered F/T subscriptions retain reliable event delivery.
        for topic,typ,callback in (('joint_states',JointState,self.joint_state),
                ('mecanum_drive_controller/odometry',Odometry,self.odometry)):
            old=[s for s in self.subscriptions if s.topic_name=='/follower/'+topic]
            if len(old)!=1:raise ValueError('single own state snapshot subscription required')
            self.destroy_subscription(old[0])
            self.create_subscription(typ,topic,callback,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        sub=self.create_subscription(String,'rotation/observation',self.observe,1)
        if sub.topic_name!='/follower/rotation/observation':raise ValueError('own yaw input boundary remapped')

    def mode_value(self):
        if not self.has_parameter('rotation_follow_mode'):
            self.declare_parameter('rotation_follow_mode','XY_ONLY')
        mode=self.p('rotation_follow_mode')
        if mode not in MODES:raise ValueError('exclusive chapter 23 mode required')
        # Snapshot once: changing a ROS parameter while armed cannot switch
        # the controller or create another writer.
        if not hasattr(self,'mode'):self.mode=mode
        return self.mode

    def new_arm(self):
        if not self.has_parameter('passive_xy_allocation_enabled'):
            self.declare_parameter('passive_xy_allocation_enabled',False)
        if not hasattr(self,'passive_xy_allocation'):
            self.passive_xy_allocation=bool(self.p('passive_xy_allocation_enabled'))
        if self.passive_xy_allocation:
            for key,value in (('allocation_base_filter_hz',0.),('allocation_base_regularization',1000.)):
                if not self.has_parameter(key):self.declare_parameter(key,value)
            return WholeBodyAllocator(self.model,margin=self.p('joint_margin_rad'),
                arm_speed=self.p('joint_request_speed_rad_s'),arm_acceleration=self.p('joint_acceleration_rad_s2'),
                base_speed=self.p('base_velocity_limits'),base_acceleration=self.p('base_acceleration_limits'),
                base_filter_hz=self.p('allocation_base_filter_hz'),task_metric=self.p('arm_task_metric'),
                base_regularization=self.p('allocation_base_regularization'),lock_base_yaw=True)
        if self.mode_value()=='WHOLE_BODY':return super().new_arm()
        return ArmSoftTask(self.model,metric=self.p('arm_task_metric'),speed=self.p('joint_request_speed_rad_s'),
            acceleration=self.p('joint_acceleration_rad_s2'),margin=self.p('joint_margin_rad'))

    def new_base(self):
        return RotationRecentering(mode=self.mode_value(),gains=self.p('base_gains'),
            velocity=self.p('base_velocity_limits'),acceleration=self.p('base_acceleration_limits'),
            cutoff=self.p('base_cutoff_hz'))

    def observe(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('observer object required')
            self.observation,self.observation_wall=value,time.monotonic()
            self.observation_receive_ros_s=self.get_clock().now().nanoseconds*1e-9
        except (ValueError,TypeError):self.observation={}
        # Do not wait for a control tick or let a later valid snapshot hide
        # an invalidity already delivered to an active rotation controller.
        if (self.core is not None and self.requires_rotation_observation()
                and self.observation.get('yaw_valid') is not True):
            self.trip('local yaw invalid: '+str(self.observation.get('valid_reason','malformed observation')))

    def joint_state(self,msg):
        super().joint_state(msg)
        self.yaw_joint_history.append(msg)

    def require_yaw(self):
        o=self.observation
        if time.monotonic()-self.observation_wall>self.p('timeout_s'):
            raise ValueError('local yaw observer receipt watchdog')
        if (o.get('frame_id')!='follower/base_link' or o.get('urdf_sha256')!=self.urdf_hash
                or o.get('yaw_valid') is not True):
            raise ValueError('local yaw invalid: '+str(o.get('valid_reason','missing qualification')))
        # Subscription callbacks can run after /clock advances but before
        # the next endpoint tick refreshes self.sim. Use our current ROS clock
        # for freshness, retaining the same past/future bounds.
        now=self.get_clock().now().nanoseconds*1e-9 if hasattr(self,'get_clock') else self.sim
        age_ns=round((now-o.get('joint_stamp',-1.))*1e9)
        received=getattr(self,'observation_receive_ros_s',None)
        self.rotation_diagnostics={'consumed_observation':dict(
            publication_sequence=o.get('publication_sequence'),
            sender_ros_s=o.get('timestamp'),joint_source_s=o.get('joint_stamp'),
            receive_ros_s=received,use_ros_s=now,
            source_age_ns=age_ns,receipt_age_wall_s=time.monotonic()-self.observation_wall,
            delivery_age_wall_s=None if 'publication_wall_s' not in o else self.observation_wall-o['publication_wall_s'],
            observation_duration_wall_s=o.get('observation_duration_wall_s'))}
        if not -20_000_000<=age_ns<=50_000_000:
            raise ValueError('local yaw observer source timestamp watchdog: '+str(age_ns)+' ns')
        actual=own_geometry(self,self.joints)
        if not actual['geometry_valid']:raise ValueError('local yaw geometry invalid: '+actual['valid_reason'])
        if (o.get('yaw_heading_frame','tcp_link')!=self.yaw_geometry.heading_frame
                or (self.yaw_geometry.heading_frame!='tcp_link'
                    and o.get('geometry_signature')!=self.yaw_geometry.geometry_signature)):
            raise ValueError('observer/controller yaw measurement frame mismatch')
        # Align two independently scheduled own-data paths at their SOURCE
        # timestamps. Freshness remains <=50 ms; alignment stays <=20 ms.
        candidates=[self.joints,*getattr(self,'yaw_joint_history',())]
        matched=min(candidates,key=lambda msg:abs(stamp(msg)-o['joint_stamp']))
        skew_ns=abs(round((stamp(matched)-o['joint_stamp'])*1e9))
        if skew_ns>20_000_000:
            raise ValueError('local yaw joint timestamp skew '+str(skew_ns)+' ns exceeds 20000000 ns')
        actual=own_geometry(self,matched)
        if not actual['geometry_valid']:raise ValueError('aligned local yaw geometry invalid')
        self.yaw_alignment_ns=skew_ns
        if np.linalg.norm(np.array(actual['p_ee_footprint'])-finite(o['p_ee_footprint'],(3,)))>self.p('height_error_limit_m'):
            raise ValueError('observer/local FK position mismatch')
        if not np.isfinite([o['e_yaw_rad'],o['psi_ee_B'],o['psi_home_B']]).all():
            raise ValueError('nonfinite measured local yaw')
        if abs(wrap_pi(o['psi_ee_B']-o['psi_home_B'])-o['e_yaw_rad'])>1e-12:
            raise ValueError('measured yaw error/reference mismatch')
        reference=np.r_[finite(o['p_home_footprint'],(3,)),o['psi_home_B']]
        if self.core is not None and self.observation_reference is not None and not np.allclose(reference,self.observation_reference,rtol=0.,atol=1e-12):
            raise ValueError('observer reference changed during active transport')
        return o

    def trip(self,reason):
        if not self.latched and hasattr(self,'rotation_diagnostics'):
            self.get_logger().error('Consumed yaw observation at first stop: '+json.dumps(self.rotation_diagnostics))
        super().trip(reason)
        if getattr(self,'task_compliance_budget',None) is not None:self.task_compliance_budget.reset()

    def reset(self,request,response):
        response=super().reset(request,response)
        if response.success and self.task_compliance_budget is not None:self.task_compliance_budget.reset()
        return response

    def prepare(self,request,response):
        response=super().prepare(request,response)
        writers=self.get_publishers_info_by_topic('/follower/rotation/observation')
        if len(writers)!=1 or writers[0].node_name!='rotation_observer' or writers[0].node_namespace!='/follower':
            self.prepared=response.success=False;response.message='single verified own rotation observer required'
        return response

    def enable(self,request,response):
        if self.requires_rotation_observation():
            try:self.require_yaw()
            except (ValueError,KeyError,TypeError) as exc:
                response.success,response.message=False,str(exc);return response
        response=super().enable(request,response)
        if response.success:
            if self.task_compliance_budget is not None:self.task_compliance_budget.reset()
            if self.requires_rotation_observation():
                self.reference=finite(self.observation['p_home_footprint'],(3,)).copy()
                self.observation_reference=np.r_[self.reference,self.observation['psi_home_B']]
            response.message='chapter 23 '+self.mode+' enabled; one arm/base writer'
        return response

    def requires_rotation_observation(self):
        return self.mode!='XY_ONLY' or self.rotation_observation_required

    def update_local(self):
        if self.core is None or self.latched:return
        try:
            observation=self.require_yaw() if self.requires_rotation_observation() else None
            if not self.wrench_ready():raise ValueError('local filtered wrench watchdog')
            if self.raw_wrench is None or time.monotonic()-self.raw_wall>.05 or not -.02<=self.sim-self.raw_stamp<=.05:
                raise ValueError('local unfiltered wrench watchdog')
            raw=self.raw_wrench-self.raw_bias
            self.check_wrench(raw);self.check_wrench(self.wrench6)
            if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall>1.):
                raise ValueError('own grasp confirmation lost')
            dt=self.force_stamp-self.last_step_stamp
            if dt==0.:return
            if not 0.<dt<=.05:raise ValueError('local update interval outside envelope')
            mapping=dict(zip(self.joints.name,self.joints.position))
            q=finite([mapping[n] for n in self.model.names],(5,))
            position,sensor=self.model.frames(q)
            angular=rotation_error(sensor[:3,:3],self.reference_rotation)
            error=np.r_[position-self.reference,angular]
            limits=np.array(self.p('displacement_limit_m'))
            if np.any(abs(error[:2])>limits+1e-7) or abs(error[2])>self.p('height_error_limit_m') or np.linalg.norm(angular)>self.p('rotation_error_limit_rad'):
                raise ValueError('measured grasp pose envelope exceeded')
            measured_yaw=0. if observation is None else observation['e_yaw_rad']
            self.measured_error=np.r_[error[:2],measured_yaw]
            utilization=max(np.linalg.norm(raw[:3])/self.p('wrench_force_limit_N'),
                np.linalg.norm(raw[3:])/self.p('wrench_torque_limit_Nm'),
                float(np.max(abs(error[:2])/limits)),np.linalg.norm(angular)/self.p('rotation_error_limit_rad'))
            if self.p('native_grasp_monitor_required'):utilization=max(utilization,self.native_risk())
            self.grasp_warning=bool(utilization>=self.p('grasp_warning_fraction'))
            xy=self.core.step(self.force,dt)
            target=np.r_[xy,0.,0.,0.,0.];virtual=np.r_[self.core.v,0.,0.,0.,0.]
            # This optional force-driven arm compliance is independent of the
            # base ablation. Keep identical arm mechanics for A, B and C.
            if self.p('arm_yaw_admittance_enabled'):
                target[5]=self.yaw_core.step([self.wrench6[5],0.],dt)[0]
                virtual[5]=self.yaw_core.v[0]
            if self.relief is not None:
                target[3:5]=self.relief.step(self.wrench6[3:5],dt);virtual[3:5]=self.relief.v
            preferred=self.base.step(self.measured_error,position,dt,yaw_valid=observation is not None)
            desired=whole_body_task(virtual,target,error,self.p('pose_gain_s_inv'))
            if self.mode=='WHOLE_BODY' or self.passive_xy_allocation:
                credit=max(float(np.linalg.norm(self.base.low/np.r_[limits,self.p('yaw_displacement_limit_rad')])),
                           float(np.max(abs(preferred)/self.base.vmax)))
                measured_credit=credit
                task_credit=(self.task_compliance_budget.step(np.r_[target[:2],target[5]],dt)
                             if self.task_compliance_budget is not None else 0.)
                credit=max(credit,task_credit)
                velocity=self.arm_task.step(q,self.q_request,desired,preferred,credit,dt)
                self.base_twist=velocity[5:].copy()
                self.arm_task.last.update(measured_allocation_budget=measured_credit,
                    task_compliance_allocation_budget=task_credit)
            else:
                velocity=self.arm_task.step(q,self.q_request,desired,dt)
                self.base_twist=preferred.copy()
            self.q_request += velocity[:5]*dt
            self.arm_task.last.update(grasp_utilization=float(utilization),
                observer_joint_alignment_ns=getattr(self,'yaw_alignment_ns',None),
                measured_projected_yaw_error_rad=measured_yaw,internal_yaw_target_rad=float(target[5]),
                recentering_position_footprint_m=position.tolist(),
                explicit_compensation_xy=self.base.compensation.tolist(),task_velocity=desired.tolist())
            self.last_step_stamp,self.base_wall=self.force_stamp,time.monotonic()
            msg=JointTrajectory(joint_names=list(self.model.names))
            msg.header.stamp,msg.header.frame_id=Time(seconds=self.sim).to_msg(),'follower/base_footprint'
            msg.points=[JointTrajectoryPoint(positions=self.q_request.tolist())]
            msg.points[0].time_from_start.nanosec=40000000
            self.publish_command(self.arm_pub,msg)
        except (ValueError,KeyError,TypeError) as exc:self.trip(str(exc))

    def tick(self):
        # Call the shared single output path without the legacy unconditional
        # ready publication. XY_ONLY may serve straight transport, but cannot
        # authorize the leader's autonomous rotation without a valid gate.
        YawArmBase.tick(self)
        if not hasattr(self,'ready_pub'):return
        valid=False
        try:self.require_yaw();valid=True
        except (ValueError,KeyError,TypeError):pass
        enabled=self.core is not None and not self.latched and self.permit and not self.emergency
        self.warning_pub.publish(Bool(data=self.grasp_warning if enabled else False))
        self.ready_pub.publish(Bool(data=bool(enabled and valid and self.may_authorize_rotation())))

    def may_authorize_rotation(self):
        return not self.passive_calibration


def main(args=None):
    rclpy.init(args=args);node=RotationFollower()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()
