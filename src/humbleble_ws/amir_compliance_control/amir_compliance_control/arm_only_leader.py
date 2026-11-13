"""Leader's own arm drives the payload; every base command remains zero."""
import json
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from rclpy.time import Time
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .transport import TransportEndpoint
from .safety import stamp
from .wrench import finite
from .admittance import rotation_error
from .arm_translation import reference
from .stroke_kinematics import solve_stroke_pose,stroke_pose_velocity,verify_stroke_branch
from .arm_servo_response import validate_response
from .arm_trajectory_bounds import trajectory_times


class ArmOnlyLeader(TransportEndpoint):
    def __init__(self,**kwargs):
        super().__init__(leader=True,node_name='leader_trajectory_node',base_trajectory=False,**kwargs)
        self.declare_parameter('arm_servo_response_file','')
        self.declare_parameter('arm_acceleration_m_s2',.0012)
        self.declare_parameter('arm_joint_speed_rad_s',.03)
        self.declare_parameter('arm_joint_acceleration_rad_s2',.15)
        self.declare_parameter('arm_hold_after_s',2.)
        self.declare_parameter('arm_trajectory_start_delay_s',0.)
        if not np.isfinite(self.p('arm_trajectory_start_delay_s')) or not 0<=self.p('arm_trajectory_start_delay_s')<=.1:
            raise ValueError('bounded arm trajectory start delay required')
        for key in ('arm_acceleration_m_s2','arm_joint_speed_rad_s','arm_joint_acceleration_rad_s2'):
            if not np.isfinite(self.p(key)) or self.p(key)<=0:
                raise ValueError('positive finite arm motion policy required: '+key)
        if self.p('arm_joint_speed_rad_s')>float(np.min(self.model.velocity)):
            raise ValueError('arm motion policy exceeds unchanged URDF velocity')
        path=Path(self.p('urdf_file'))
        cert=json.loads(Path(self.p('arm_servo_response_file')).read_text())
        self.tau=validate_response(cert,path.read_text(),path.parent)
        self.motion=None;self.origin=None;self.rotation_home=None;self.base_home=None
        self.previous_knots=None;self.previous_stamp=None;self.motion_done=False
        self.previous_qdot=np.zeros(5)
        self.preflight_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='arm_preflight')
        self.pending_motion=None
        self.preflight_wall_s=None
        self.arm_debug=self.create_publisher(String,'arm_compensation/debug',10)

    def q_actual(self):
        values=dict(zip(self.joints.name,self.joints.position))
        return finite([values[name] for name in self.model.names],(5,))

    def local_health(self):
        good,reason=super().local_health()
        if good and self.p('actuator_ack_required') and (
                self.gate.get('base_zero_only') is not True or self.actuator.get('base_zero_only') is not True
                or self.actuator.get('request_writer_name')!=self.get_name()):
            return False,'leader ARM_ONLY zero lock and sole writer audit required'
        return good,reason

    def base_pose(self):
        pose=self.encoder.pose.pose
        return np.array([pose.position.x,pose.position.y,2*np.arctan2(pose.orientation.z,pose.orientation.w)])

    def goal(self,msg):
        try:
            if self.state!='READY' or self.motion is not None or self.pending_motion is not None or not self.homed or not self.local_health()[0]:
                raise ValueError('one arm trajectory requires local homed READY state')
            value=finite([msg.vector.x,msg.vector.y,msg.vector.z],(3,))
            if (msg.header.frame_id!='leader/base_footprint' or not 0<abs(value[0])<=self.p('max_distance_m')
                    or np.any(value[1:]!=0) or stamp(msg)<=self.last_goal_stamp
                    or not -.02<=self.sim-stamp(msg)<=self.p('timeout_s')):
                raise ValueError('fresh bounded local arm translation required')
            q=self.q_actual();self.origin,sensor=self.model.frames(q)
            self.rotation_home=sensor[:3,:3].copy();self.base_home=self.base_pose()
            # Verify the full branch before accepting a motion, never move a
            # base to recover an unreachable five-joint arm pose.
            # A full 31-pose check can exceed the 150ms receive watchdog.
            # The worker uses only immutable model data and copied local state;
            # acceptance remains on the ROS thread after fresh health checks.
            future=self.preflight_executor.submit(verify_stroke_branch,self.model,q.copy(),
                self.origin.copy(),self.rotation_home.copy(),float(value[0]))
            self.pending_motion=(future,time.monotonic(),float(value[0]),stamp(msg),q.copy())
        except (ValueError,np.linalg.LinAlgError) as exc:
            self.rejected+=1;self.last_rejection=str(exc)

    def finish_preflight(self):
        if self.pending_motion is None:return
        future,started,distance,goal_stamp,initial_q=self.pending_motion
        if self.latched or self.emergency or not self.permit:
            future.cancel();self.pending_motion=None
            self.rejected+=1;self.last_rejection='arm preflight cancelled by shared/local stop'
            return
        if not future.done() and time.monotonic()-started<=2.:return
        self.pending_motion=None
        try:
            if time.monotonic()-started>2.:
                future.cancel();raise ValueError('arm preflight computation timeout')
            future.result()
            if not self.local_health()[0] or not self.homed or self.state!='READY':
                raise ValueError('fresh local READY state required after arm preflight')
            if np.max(abs(self.q_actual()-initial_q))>=.001:
                raise ValueError('local arm moved during preflight')
            self.preflight_wall_s=time.monotonic()-started
            self.motion=(self.sim+.05,distance);self.last_goal_stamp=goal_stamp
            self.accepted+=1;self.state='TRANSPORT';self.reason=''
        except (ValueError,np.linalg.LinAlgError) as exc:
            self.rejected+=1;self.last_rejection=str(exc)

    def base_request(self,command):
        self.finish_preflight()
        if self.motion is None or self.latched:
            return np.zeros(3)
        try:
            drift=self.base_pose()-self.base_home
            drift[2]=np.arctan2(np.sin(drift[2]),np.cos(drift[2]))
            twist=self.encoder.twist.twist
            if np.linalg.norm(drift[:2])>=.001 or abs(drift[2])>=.002 or max(
                    abs(twist.linear.x),abs(twist.linear.y),abs(twist.angular.z))>=.001:
                raise ValueError('leader ARM_ONLY measured base drift/speed')
            start,distance=self.motion
            x,v,total=reference(self.sim-start,distance,self.p('speed_m_s'),self.p('arm_acceleration_m_s2'))
            q=self.q_actual();actual,sensor=self.model.frames(q)
            target=self.origin+[x,0,0]
            if np.linalg.norm(actual-target)>.005 or np.linalg.norm(rotation_error(sensor[:3,:3],self.rotation_home))>.01:
                raise ValueError('leader arm-only measured tracking/orientation')
            desired=solve_stroke_pose(self.model,q,target,self.rotation_home)
            ahead,_,_=reference(self.sim-start+.08,distance,self.p('speed_m_s'),self.p('arm_acceleration_m_s2'))
            future=solve_stroke_pose(self.model,desired,self.origin+[ahead,0,0],self.rotation_home)
            # Pose IK accepts a 1um residual. Differencing two independently
            # accepted solves turns that tolerance into spurious acceleration
            # at small velocities. Differentiate the same constrained path.
            qdot=stroke_pose_velocity(self.model,desired,v)
            knots=np.vstack((desired+self.tau*qdot,future+self.tau*qdot))
            joint_speed=self.p('arm_joint_speed_rad_s')
            if np.max(abs(qdot))>joint_speed or np.any(knots<=self.model.lower+.01) or np.any(knots>=self.model.upper-.01):
                raise ValueError('leader arm-only joint speed/position bound')
            if self.previous_stamp is not None:
                dt=self.sim-self.previous_stamp
                if dt==0:return np.zeros(3)
                if not 0<dt<=.05:raise ValueError('leader arm-only dt abnormal')
                if np.max(abs(knots-self.previous_knots))/dt>joint_speed+1e-6:
                    raise ValueError('leader arm-only trajectory knot rate')
                if np.max(abs(qdot-self.previous_qdot))/dt>self.p('arm_joint_acceleration_rad_s2')+1e-6:
                    raise ValueError('leader arm-only joint acceleration experiment bound')
            msg=JointTrajectory(joint_names=list(self.model.names))
            delay=self.p('arm_trajectory_start_delay_s')
            # At higher speeds a compensated servo reference is not reachable
            # in one 4ms tick. Give BOTH knots actual future times. Keep the
            # unchanged gate's q_actual -> first-point URDF rate validation;
            # do not backdate a larger duration while retaining a "now" knot.
            header,first,end=trajectory_times(self.sim,.004,.08,delay)
            msg.header.stamp=Time(seconds=header).to_msg()
            msg.header.frame_id='leader/base_footprint'
            for knot,seconds in zip(knots,(first,end)):
                point=JointTrajectoryPoint(positions=knot.tolist())
                point.time_from_start=Duration(sec=0,nanosec=round(seconds*1e9));msg.points.append(point)
            self.publish_command(self.arm_pub,msg)
            self.previous_stamp,self.previous_knots=self.sim,knots
            self.previous_qdot=qdot.copy()
            if (not self.motion_done and self.sim-start>=total+self.p('arm_hold_after_s')
                    and np.linalg.norm(actual-(self.origin+[distance,0,0]))<.0001):
                self.completed+=1;self.state='READY';self.motion_done=True
            if not self.motion_done and self.sim-start>total+self.p('arm_hold_after_s')+3:
                raise ValueError('leader arm-only measured arrival timeout')
            self.arm_debug.publish(String(data=json.dumps({'mode':'LEADER_ARM_ONLY','p_target_m':target.tolist(),
                'p_actual_m':actual.tolist(),'p_home_m':self.origin.tolist(),'q_actual_rad':q.tolist(),
                'q_cmd_rad':knots[-1].tolist(),'qdot_rad_s':qdot.tolist(),'base_drift_xy_yaw':drift.tolist(),
                'preflight_wall_s':self.preflight_wall_s,
                'joint_speed_policy_rad_s':joint_speed,
                'joint_acceleration_policy_rad_s2':self.p('arm_joint_acceleration_rad_s2'),
                'trajectory_start_delay_s':delay,
                'arm_reference_velocity_m_s':v,'sim_time':self.sim})))
        except (ValueError,np.linalg.LinAlgError) as exc:
            self.trip(str(exc))
        return np.zeros(3)

    def publish_command(self,publisher,message):
        if publisher is self.base_pub:
            message.twist.linear.x=message.twist.linear.y=message.twist.linear.z=0.
            message.twist.angular.x=message.twist.angular.y=message.twist.angular.z=0.
        super().publish_command(publisher,message)

    def tick(self):
        if self.latched:self.finish_preflight()
        super().tick()
        # Continuous zero output includes INIT/disarmed intervals.
        msg=TwistStamped();msg.header.stamp=Time(seconds=self.sim).to_msg()
        msg.header.frame_id='leader/base_footprint';self.publish_command(self.base_pub,msg)


def main(args=None):
    rclpy.init(args=args);node=ArmOnlyLeader()
    try:rclpy.spin(node)
    finally:
        node.preflight_executor.shutdown(wait=False,cancel_futures=True)
        node.destroy_node();rclpy.try_shutdown()
