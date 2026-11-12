"""Leader base stroke with local arm retraction; no reference sent to follower."""
import json
from pathlib import Path
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .transport import TransportEndpoint
from .admittance import rotation_error
from .stroke_kinematics import solve_stroke_pose as solve_pose
from .arm_servo_response import validate_response
from .base_odometry_calibration import validate_distance_calibration,scaled_odometry


class CompensatingLeader(TransportEndpoint):
    def __init__(self, **kwargs):
        super().__init__(leader=True,node_name='leader_trajectory_node',**kwargs)
        self.declare_parameter('arm_retraction_ratio',.5)
        self.declare_parameter('arm_servo_response_file','')
        self.declare_parameter('base_distance_calibration_file','')
        self.ratio=float(self.p('arm_retraction_ratio'))
        if self.ratio!=.5:raise ValueError('only the audited half-stroke arm compensation is supported')
        path=Path(self.p('urdf_file'))
        calibration=self.p('base_distance_calibration_file')
        self.odom_scale=(validate_distance_calibration(json.loads(Path(calibration).read_text()),path.parent)
                         if calibration else 1.)
        if calibration and abs(self.p('speed_m_s')/self.odom_scale-.005)>1e-12:
            raise ValueError('measured native distance calibration requires raw 0.005 m/s cap')
        cert=json.loads(Path(self.p('arm_servo_response_file')).read_text())
        self.tau=validate_response(cert,path.read_text(),path.parent)
        self.arm_origin=None;self.arm_rotation=None;self.arm_distance=0.
        self.previous_knots=None;self.previous_stamp=None
        self.arm_debug=self.create_publisher(String,'arm_compensation/debug',10)

    def odometry(self,msg):
        super().odometry(scaled_odometry(msg,self.odom_scale) if self.odom_scale!=1. else msg)

    def q_actual(self):
        mapping=dict(zip(self.joints.name,self.joints.position))
        return np.array([mapping[n] for n in self.model.names])

    def goal(self,msg):
        if self.arm_distance+msg.vector.x>self.p('max_distance_m')+1e-12:
            self.rejected+=1;self.last_rejection='cumulative compensated stroke exceeds configured total';return
        if msg.vector.x<=0:
            self.rejected+=1;self.last_rejection='compensated stroke is forward only';return
        before=self.accepted
        super().goal(msg)
        if self.accepted!=before:
            if self.arm_origin is None:
                q=self.q_actual();self.arm_origin,sensor=self.model.frames(q)
                self.arm_rotation=sensor[:3,:3].copy()
                self.base_origin=self.goal_data[2:5]
            self.arm_distance+=float(msg.vector.x)

    def base_request(self,command):
        if self.arm_origin is None or self.latched:return super().base_request(command/self.odom_scale)
        try:
            x0,y0,yaw=self.base_origin;p=self.encoder.pose.pose.position
            travel=(p.x-x0)*np.cos(yaw)+(p.y-y0)*np.sin(yaw)
            q=self.q_actual();actual,sensor=self.model.frames(q)
            target=self.arm_origin-np.array([self.ratio*np.clip(travel,0.,self.arm_distance),0.,0.])
            if np.linalg.norm(actual-target)>.01 or np.linalg.norm(rotation_error(sensor[:3,:3],self.arm_rotation))>.01:
                raise ValueError('leader arm measured tracking/orientation envelope')
            horizon,period=.08,.004
            desired=solve_pose(self.model,q,target,self.arm_rotation)
            future=solve_pose(self.model,desired,target-np.array([self.ratio*command*horizon,0.,0.]),self.arm_rotation)
            qdot=(future-desired)/horizon
            if np.max(np.abs(qdot))>.03 or np.min(np.minimum(q-self.model.lower,self.model.upper-q))<=.01:
                raise ValueError('leader compensation joint speed/margin')
            start=desired+self.tau*qdot;end=future+self.tau*qdot
            knots=np.vstack((start,end))
            if np.any(knots<=self.model.lower+.01) or np.any(knots>=self.model.upper-.01):
                raise ValueError('leader compensation command soft limit')
            if self.previous_knots is not None:
                dt=self.sim-self.previous_stamp
                if dt<=0:return super().base_request(command/self.odom_scale)
                if np.max(abs(knots-self.previous_knots))/dt>.03+1e-6:
                    raise ValueError('leader compensation command rate')
            msg=JointTrajectory(joint_names=list(self.model.names));msg.header.stamp=self.get_clock().now().to_msg()
            msg.header.frame_id='leader/base_footprint'
            for value,t in ((start,period),(end,period+horizon)):
                point=JointTrajectoryPoint(positions=value.tolist())
                point.time_from_start=Duration(sec=int(t),nanosec=int(round(t%1*1e9)));msg.points.append(point)
            self.publish_command(self.arm_pub,msg)
            self.previous_knots,self.previous_stamp=knots,self.sim
            self.arm_debug.publish(String(data=json.dumps({'base_travel_m':travel,'arm_retraction_ratio':self.ratio,
                'base_travel_raw_odom_m':travel/self.odom_scale,'metres_per_raw_odom_metre':self.odom_scale,
                'p_target_m':target.tolist(),'p_actual_m':actual.tolist(),'q_actual_rad':q.tolist(),
                'q_cmd_rad':end.tolist(),'qdot_rad_s':qdot.tolist(),'sim_time':self.sim})))
            return super().base_request(command/self.odom_scale)
        except (ValueError,np.linalg.LinAlgError) as exc:
            self.trip(str(exc));return np.zeros(3)


def main(args=None):
    rclpy.init(args=args);node=CompensatingLeader()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()
