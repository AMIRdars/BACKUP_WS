"""CP-8R-3: a single 8-variable allocator, local xy/yaw and grasp reserve.

The inherited CP-8R-2 raw/filtered wrench guards and stop path remain active.
The slow recentering output is a preference INSIDE allocation, never a second
base command. No goal, centre, leader state or evaluation input is accepted.
"""
import time
import json

import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .admittance import rotation_error, PlanarAdmittance
from .whole_body import WholeBodyAllocator, base_jacobian
from .wrench import finite
from .yaw_arm_base import YawArmBase
from .grasp_reserve import native_reserve_risk


class YawWholeBody(YawArmBase):
    def __init__(self, **kwargs):
        super().__init__(node_name='follower_yaw_whole_body', **kwargs)
        # The preprocessor offers reliable delivery for BOTH local streams.
        # Retain independent raw and filtered guards, but request reliable
        # delivery instead of accepting best-effort loss of raw stop samples.
        # Timestamp, source-age and 50 ms receipt guards remain unchanged.
        for topic,callback in (('wrench_at_grasp_raw',self.raw),('wrench_filtered',self.filtered)):
            previous=[s for s in self.subscriptions if s.topic_name=='/follower/'+topic]
            if len(previous)!=1:
                raise ValueError('exactly one local F/T stream required')
            self.destroy_subscription(previous[0])
            self.create_subscription(WrenchStamped,topic,callback,10)
        self.role = 'WHOLE_BODY_XY_YAW'
        self.declare_parameter('grasp_warning_fraction', .6)
        if not 0. < self.p('grasp_warning_fraction') < 1.:
            raise ValueError('warning must precede the hard envelope')
        self.grasp_warning = False
        self.declare_parameter('native_grasp_monitor_required', False)
        self.native_grasp, self.native_wall = {}, 0.
        self.declare_parameter('rotational_relief_enabled', False)
        for key,value in {'relief_inertia':[.2,.2], 'relief_damping':[12.,12.], 'relief_stiffness':[4.,4.],
            'relief_displacement_rad':[.001,.001], 'relief_velocity_rad_s':[.002,.002],
            'relief_acceleration_rad_s2':[.02,.02]}.items():self.declare_parameter(key,value)
        self.new_relief()
        self.relief = None
        if self.p('native_grasp_monitor_required'):
            sub = self.create_subscription(String, 'grasp/monitor_debug', self.native_sample, 10)
            if sub.topic_name != '/follower/grasp/monitor_debug':
                raise ValueError('native local grasp reserve boundary violated')
        self.warning_pub = self.create_publisher(Bool, '/transport/rotation_grasp_warning', 10)
        self.ready_pub = self.create_publisher(Bool, '/transport/rotation_follower_ready', 10)
        if (self.warning_pub.topic_name != '/transport/rotation_grasp_warning'
                or self.ready_pub.topic_name != '/transport/rotation_follower_ready'):
            raise ValueError('boolean grasp warning output boundary violated')

    def new_arm(self):
        if not self.has_parameter('allocation_base_filter_hz'):
            self.declare_parameter('allocation_base_filter_hz', 0.)
            self.declare_parameter('allocation_base_regularization', 1000.)
        return WholeBodyAllocator(self.model, margin=self.p('joint_margin_rad'),
            arm_speed=self.p('joint_request_speed_rad_s'), arm_acceleration=self.p('joint_acceleration_rad_s2'),
            base_speed=self.p('base_velocity_limits'), base_acceleration=self.p('base_acceleration_limits'),
            base_filter_hz=self.p('allocation_base_filter_hz'), task_metric=self.p('arm_task_metric'),
            base_regularization=self.p('allocation_base_regularization'))

    def enable(self, request, response):
        if self.p('native_grasp_monitor_required'):
            try:
                self.native_risk()
            except ValueError as exc:
                response.success, response.message = False, str(exc)
                return response
        response = super().enable(request, response)
        if response.success:
            self.grasp_warning = False
            self.relief = self.new_relief() if self.p('rotational_relief_enabled') else None
            response.message = 'local WHOLE_BODY xy+yaw enabled; exactly one allocated base/arm output'
        return response

    def new_relief(self):
        return PlanarAdmittance(*(self.p(key) for key in ('relief_inertia','relief_damping','relief_stiffness',
            'relief_displacement_rad','relief_velocity_rad_s','relief_acceleration_rad_s2')))

    def prepare(self, request, response):
        response = super().prepare(request, response)
        if self.p('native_grasp_monitor_required'):
            writers = self.get_publishers_info_by_topic('/follower/grasp/monitor_debug')
            if len(writers) != 1 or writers[0].node_name != 'native_grasp_monitor' or writers[0].node_namespace != '/follower':
                self.prepared = response.success = False
                response.message = 'verified own native grasp monitor required'
        return response

    def native_sample(self, msg):
        try:
            self.native_grasp = json.loads(msg.data)
            self.native_wall = time.monotonic()
        except (ValueError, TypeError):
            self.native_grasp = {}

    def native_risk(self):
        return native_reserve_risk(self.native_grasp, self.sim, time.monotonic()-self.native_wall)

    def trip(self, reason):
        if not self.latched and hasattr(self, 'raw_stamp'):
            self.get_logger().error('First whole-body stop context: '+json.dumps({'reason':reason,'sim_time':self.sim,
                'raw_receipt_age_wall_s':time.monotonic()-self.raw_wall,'raw_source_age_sim_s':self.sim-self.raw_stamp,
                'filtered_receipt_age_wall_s':time.monotonic()-self.force_wall,
                'filtered_source_age_sim_s':None if self.force_stamp is None else self.sim-self.force_stamp}))
        super().trip(reason)
        self.relief = None

    def update_local(self):
        if self.core is None or self.latched:
            return
        try:
            if not self.wrench_ready():
                raise ValueError('whole-body local wrench watchdog')
            if (self.raw_wrench is None or time.monotonic()-self.raw_wall > .05
                    or not -.02 <= self.sim-self.raw_stamp <= .05):
                raise ValueError('whole-body unfiltered TCP wrench watchdog')
            raw = self.raw_wrench-self.raw_bias
            self.check_wrench(self.wrench6)
            self.check_wrench(raw)
            if not self.p('allow_ungrasped_test') and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                raise ValueError('whole-body local grasp confirmation lost')
            dt = self.force_stamp-self.last_step_stamp
            if dt == 0.:
                return
            if dt < 0.:
                raise ValueError('whole-body local time moved backwards')
            mapping = dict(zip(self.joints.name, self.joints.position))
            q = finite([mapping[n] for n in self.model.names], (5,))
            position, sensor = self.model.frames(q)
            angular = rotation_error(sensor[:3, :3], self.reference_rotation)
            error = np.r_[position-self.reference, angular]
            xy_limits = np.array(self.p('displacement_limit_m'))
            if (np.any(abs(error[:2]) > xy_limits+1e-7)
                    or abs(error[2]) > self.p('height_error_limit_m')
                    or np.linalg.norm(angular) > self.p('rotation_error_limit_rad')):
                raise ValueError('whole-body measured grasp pose envelope exceeded')
            self.measured_error = error[[0, 1, 5]]
            utilization = max(np.linalg.norm(raw[:3])/self.p('wrench_force_limit_N'),
                np.linalg.norm(raw[3:])/self.p('wrench_torque_limit_Nm'),
                float(np.max(abs(error[:2])/xy_limits)),
                abs(error[2])/self.p('height_error_limit_m'),
                np.linalg.norm(angular)/self.p('rotation_error_limit_rad'))
            if self.p('native_grasp_monitor_required'):
                utilization = max(utilization, self.native_risk())
            self.grasp_warning = bool(utilization >= self.p('grasp_warning_fraction'))
            self.warning_pub.publish(Bool(data=self.grasp_warning))
            xy = self.core.step(self.force, dt)
            yaw = self.yaw_core.step([self.wrench6[5], 0.], dt)[0]
            target = np.r_[xy, 0., 0., 0., yaw]
            virtual_velocity = np.r_[self.core.v, 0., 0., 0., self.yaw_core.v[0]]
            if self.relief is not None:
                target[3:5] = self.relief.step(self.wrench6[3:5],dt)
                virtual_velocity[3:5] = self.relief.v
            preferred = self.base.step(self.measured_error, position, self.wrench6[[0, 1, 5]], dt)
            desired = virtual_velocity+self.p('pose_gain_s_inv')*(target-error)+base_jacobian(position)@preferred
            credit = max(float(np.linalg.norm(self.base.low/np.r_[xy_limits, self.p('yaw_displacement_limit_rad')])),
                         float(np.max(abs(preferred)/self.base.vmax)))
            velocity = self.arm_task.step(q, self.q_request, desired, preferred, credit, dt)
            self.q_request += velocity[:5]*dt
            self.base_twist = velocity[5:].copy()
            self.arm_task.last.update(grasp_utilization=float(utilization), grasp_warning=self.grasp_warning,
                allocated_base_preference=preferred.tolist(),relief_roll_pitch_rad=target[3:5].tolist())
            self.last_step_stamp, self.base_wall = self.force_stamp, time.monotonic()
            msg = JointTrajectory(joint_names=list(self.model.names))
            msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'follower/base_footprint'
            msg.points = [JointTrajectoryPoint(positions=self.q_request.tolist())]
            msg.points[0].time_from_start.nanosec = 40000000
            self.publish_command(self.arm_pub, msg)
        except (ValueError, KeyError) as exc:
            self.trip(str(exc))

    def tick(self):
        super().tick()
        if hasattr(self, 'warning_pub'):
            self.warning_pub.publish(Bool(data=self.grasp_warning if self.core is not None else False))
            self.ready_pub.publish(Bool(data=self.core is not None and not self.latched and self.permit and not self.emergency))


def main(args=None):
    rclpy.init(args=args)
    node = YawWholeBody()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
