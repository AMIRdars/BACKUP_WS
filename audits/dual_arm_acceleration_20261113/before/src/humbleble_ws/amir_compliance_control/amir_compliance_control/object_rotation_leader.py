"""Opt-in CP-8R-1 leader: calibrated centre, exclusive straight/rotation goals.

No simulator/evaluator/peer input. All actuation goes through TransportEndpoint
and the existing local safety gate/final interlock. This stage holds the initial
arm pose; local leader compliance is a later extension.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from .object_center_rotation_trajectory import (
    ObjectCenterRotation, initialize_object_pose, rigid_transform, wrap_pi,
    world_velocity_to_body)
from .planar_leader import PlanarLeader, yaw_of
from .safety import stamp
from .wrench import finite, origin, rotation


def minimum_duration(delta, base_radius, grasp_radius, *, angular_speed,
                     base_speed, grasp_speed, angular_acceleration, base_acceleration):
    """Conservative quintic envelope, including base centripetal acceleration."""
    values = finite([delta, base_radius, grasp_radius, angular_speed, base_speed,
                     grasp_speed, angular_acceleration, base_acceleration], (8,))
    if np.any(values[1:3] < 0.) or np.any(values[3:] <= 0.):
        raise ValueError('positive speed/acceleration envelope required')
    angle = abs(delta)
    return max(1.875*angle/angular_speed, 1.875*angle*base_radius/base_speed,
               1.875*angle*grasp_radius/grasp_speed,
               math.sqrt((10.*math.sqrt(3.)/3.)*angle/angular_acceleration),
               math.sqrt(base_radius*((10.*math.sqrt(3.)/3.)*angle+(1.875*angle)**2)/base_acceleration))


class ObjectRotationLeader(PlanarLeader):
    def __init__(self, **kwargs):
        super().__init__(node_name='leader_object_rotation', **kwargs)
        defaults = {'grasp_calibration_file': '', 'rotation_duration_s': 0.,
                    'rotation_max_yaw_rad': 0., 'rotation_max_angular_speed': 0.,
                    'rotation_max_base_speed': 0., 'rotation_max_grasp_speed': 0.,
                    'rotation_max_angular_acceleration': 0., 'rotation_max_base_acceleration': 0.,
                    'rotation_yaw_gain': 0., 'rotation_yaw_tolerance_rad': 0.,
                    'rotation_position_tolerance_m': 0., 'rotation_terminal_timeout_s': 0.,
                    'rotation_tracking_limit_m': 0., 'rotation_tracking_limit_rad': 0.,
                    'rotation_hold_s': 0., 'rotation_arm_position_limit_m': 0.,
                    'rotation_arm_angle_limit_rad': 0., 'rotation_ungrasped_test': False}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        # Snapshot explicit policies: runtime parameter edits cannot change an
        # accepted trajectory or widen its envelope while armed.
        self.policy = {k: self.p(k) for k in defaults}
        for key, value in self.policy.items():
            if key not in ('grasp_calibration_file', 'rotation_ungrasped_test'):
                if not math.isfinite(value) or value <= 0.:
                    raise ValueError('explicit positive rotation policy required: '+key)
        file = Path(self.policy['grasp_calibration_file'])
        certificate = json.loads(file.read_text())
        xml = Path(self.p('urdf_file')).read_bytes()
        if (certificate.get('urdf_sha256') != hashlib.sha256(xml).hexdigest()
                or certificate.get('robot') != 'leader' or certificate.get('grasp_frame') != 'tcp_link'
                or certificate.get('object_frame') != 'geometric_center_z_up'
                or certificate.get('base_frame') != 'base_footprint'
                or not certificate.get('calibration_verified')
                or (certificate.get('scope') == 'virtual_ungrasped_fixture'
                    and not self.policy['rotation_ungrasped_test'])
                or certificate.get('scope') not in ('virtual_ungrasped_fixture', 'fixed_fixture_verified', 'native_contact_verified')
                or (certificate.get('scope') == 'native_contact_verified' and
                    not (certificate.get('native_contact_only') is True and certificate.get('both_native_monitors_holding') is True))):
            raise ValueError('matching verified full grasp calibration required')
        self.object_T_grasp = rigid_transform(certificate['object_T_grasp'])
        self.calibration_scope = certificate['scope']
        self.calibration_sha256 = hashlib.sha256(file.read_bytes()).hexdigest()
        root = ET.fromstring(xml)
        joint = root.find("joint[@name='tcp_fixed']")
        if (joint is None or joint.find('parent').get('link') != 'gripper_base_1'
                or joint.find('child').get('link') != 'tcp_link'):
            raise ValueError('audited wrist-to-TCP transform required')
        self.sensor_T_tcp = origin(joint.find('origin'))
        self.rotation_session = None
        self.rotation_velocity = np.zeros(3)
        self.last_rotation_value = None
        self.rotation_hold_start = None
        self.rotation_debug = self.create_publisher(String, 'rotation/debug', 10)
        sub = self.create_subscription(Float64, 'object_rotation_delta', self.rotation_goal, 1)
        if sub.topic_name != '/leader/object_rotation_delta' or self.rotation_debug.topic_name != '/leader/rotation/debug':
            raise ValueError('rotation input/output boundary violated')
        self.create_service(Trigger, 'rotation/rearm_same_delta', self.rearm_same_delta)

    def base_pose(self):
        msg = self.encoder.pose.pose
        if abs(msg.orientation.x)+abs(msg.orientation.y) > 1e-8:
            raise ValueError('planar encoder rotation required')
        result = np.eye(4)
        result[:3, :3] = rotation([0., 0., 1.], yaw_of(msg))
        result[:3, 3] = [msg.position.x, msg.position.y, msg.position.z]
        return result

    def grasp_pose(self):
        mapping = dict(zip(self.joints.name, self.joints.position))
        q = finite([mapping[n] for n in self.model.names], (5,))
        tcp, sensor = self.model.frames(q)
        result = sensor @ self.sensor_T_tcp
        if not np.allclose(result[:3, 3], tcp, atol=1e-10, rtol=0.):
            raise ValueError('wrist/TCP FK mismatch')
        return result

    def goal(self, msg):
        if self.rotation_session is not None:
            self.rejected += 1
            self.last_rejection = 'rotation active; straight goal rejected'
            return
        super().goal(msg)

    def rotation_goal(self, msg):
        try:
            delta = float(msg.data)
            if (self.state != 'READY' or self.rotation_session is not None or self.path_goal is not None
                    or self.latched or not self.homed or not self.prepared or not self.local_health()[0]
                    or not self.permit or self.emergency
                    or time.monotonic()-self.permit_wall > self.p('timeout_s')):
                raise ValueError('leader is not READY for rotation')
            if (not math.isfinite(delta) or not 0. < abs(delta) <= self.policy['rotation_max_yaw_rad']
                    or delta == self.last_rotation_value):
                raise ValueError('invalid/excessive or duplicate rotation delta; explicit rearm required')
            if not self.policy['rotation_ungrasped_test'] and (not self.grasp or time.monotonic()-self.grasp_wall > 1.):
                raise ValueError('fresh local grasp required')
            if self.policy['rotation_ungrasped_test'] and self.grasp:
                raise ValueError('virtual ungrasped calibration cannot drive an attached payload')
            if abs(stamp(self.encoder)-stamp(self.joints)) > .02:
                raise ValueError('odom/FK timestamp skew')
            twist = self.encoder.twist.twist
            if max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z)) >= .0001:
                raise ValueError('stationary start required')
            base0, local_grasp0 = self.base_pose(), self.grasp_pose()
            object0 = initialize_object_pose(base0, local_grasp0, self.object_T_grasp)
            object0 = self.planning_object_pose(object0)
            grasp0 = base0 @ local_grasp0
            radius_b = np.linalg.norm(base0[:2, 3]-object0[:2, 3])
            radius_g = np.linalg.norm(grasp0[:2, 3]-object0[:2, 3])
            minimum = minimum_duration(delta, radius_b, radius_g,
                angular_speed=self.policy['rotation_max_angular_speed'],
                base_speed=self.policy['rotation_max_base_speed'], grasp_speed=self.policy['rotation_max_grasp_speed'],
                angular_acceleration=self.policy['rotation_max_angular_acceleration'],
                base_acceleration=self.policy['rotation_max_base_acceleration'])
            duration = max(self.policy['rotation_duration_s'], minimum)
            trajectory = ObjectCenterRotation(object0, base0, grasp0, duration=duration, delta_yaw_rad=delta)
            self.rotation_session = (self.sim+.05, trajectory, duration, local_grasp0)
            self.rotation_velocity[:] = 0.
            self.rotation_hold_start = None
            self.last_rotation_value = delta
            self.accepted += 1
            self.state, self.reason = 'ROTATE_OBJECT_CENTER', ''
        except (ValueError, KeyError) as exc:
            self.rejected += 1
            self.last_rejection = str(exc)

    def rearm_same_delta(self, request, response):
        response.success = self.state == 'READY' and self.rotation_session is None and not self.latched
        if response.success:
            self.last_rotation_value = None
        response.message = 'same delta explicitly rearmed' if response.success else 'READY required'
        return response

    def base_request(self, command):
        if self.rotation_session is None:
            return super().base_request(command)
        try:
            start, trajectory, duration, grasp0 = self.rotation_session
            elapsed, ref = self.rotation_reference(start, trajectory)
            actual, grasp = self.base_pose(), self.grasp_pose()
            yaw = yaw_of(self.encoder.pose.pose)
            error = ref.base_pose[:3, 3]-actual[:3, 3]
            yaw_error = wrap_pi(ref.base_yaw_unwrapped-yaw)
            arm_error = float(np.linalg.norm(grasp[:3, 3]-grasp0[:3, 3]))
            arm_angle = math.acos(float(np.clip((np.trace(grasp[:3, :3]@grasp0[:3, :3].T)-1.)/2., -1., 1.)))
            if (np.linalg.norm(error[:2]) > self.policy['rotation_tracking_limit_m']
                    or abs(yaw_error) > self.policy['rotation_tracking_limit_rad']
                    or arm_error > self.policy['rotation_arm_position_limit_m']
                    or arm_angle > self.policy['rotation_arm_angle_limit_rad']):
                raise ValueError('rotation tracking or local grasp pose envelope exceeded')
            v_world = ref.base_velocity_world+self.p('position_gain_s_inv')*error
            v_body = world_velocity_to_body(v_world, actual[:3, :3])[:2]
            speed = np.linalg.norm(v_body)
            if speed > self.policy['rotation_max_base_speed']:
                v_body *= self.policy['rotation_max_base_speed']/speed
            wz = np.clip(ref.omega+self.policy['rotation_yaw_gain']*yaw_error,
                         -self.policy['rotation_max_angular_speed'], self.policy['rotation_max_angular_speed'])
            desired = np.r_[v_body, wz]
            dt = self.sim-self.last_tick_sim if self.last_tick_sim is not None else 0.
            if not 0. <= dt <= .05:
                raise ValueError('rotation timestep outside local envelope')
            change = desired-self.rotation_velocity
            length = np.linalg.norm(change[:2])
            bound = self.policy['rotation_max_base_acceleration']*dt
            if length > bound:
                change[:2] *= bound/max(length, 1e-15)
            change[2] = np.clip(change[2], -self.policy['rotation_max_angular_acceleration']*dt,
                                self.policy['rotation_max_angular_acceleration']*dt)
            self.rotation_velocity += change
            at_target = (elapsed >= duration and np.linalg.norm(error[:2]) <= self.policy['rotation_position_tolerance_m']
                         and abs(yaw_error) <= self.policy['rotation_yaw_tolerance_rad'])
            if at_target:
                self.state = 'ROTATION_HOLD'
                if self.rotation_hold_start is None:
                    self.rotation_hold_start = self.sim
                measured = self.encoder.twist.twist
                stopped = max(abs(measured.linear.x), abs(measured.linear.y), abs(measured.angular.z)) < .0001
                if self.sim-self.rotation_hold_start >= self.policy['rotation_hold_s'] and stopped:
                    self.rotation_velocity[:] = 0.
                    self.rotation_session = None
                    self.state = 'READY'
                    self.completed += 1
            else:
                self.rotation_hold_start = None
                self.state = 'ROTATE_OBJECT_CENTER'
            self.rotation_debug.publish(String(data=json.dumps({
                'sim_time': self.sim, 'state': self.state, 'phi_rad': ref.phi, 'omega_rad_s': ref.omega,
                'alpha_rad_s2': ref.alpha, 'duration_s': duration,
                'center_odom_xyz': ref.object_pose[:3, 3].tolist(),
                'base_desired_odom': ref.base_pose.tolist(), 'grasp_desired_odom': ref.grasp_pose.tolist(),
                'base_error_xy_m': error[:2].tolist(), 'yaw_error_rad': yaw_error,
                'arm_position_error_m': arm_error, 'arm_angle_error_rad': arm_angle,
                'command_body_vx_vy_wz': self.rotation_velocity.tolist(),
                'calibration_sha256': self.calibration_sha256})))
            if elapsed > duration+self.policy['rotation_terminal_timeout_s']:
                raise ValueError('rotation terminal tracking timeout')
            return self.rotation_velocity.copy()
        except (ValueError, KeyError) as exc:
            self.trip(str(exc))
            return np.zeros(3)

    def rotation_reference(self, start, trajectory):
        elapsed = self.sim-start
        return elapsed, trajectory.sample(elapsed)

    def planning_object_pose(self, object_pose):
        if getattr(self, 'calibration_scope', None) == 'native_contact_verified':
            # The passive calibration executable uses this base class before
            # follower yaw can be qualified. Match native operational planning:
            # reconstruct C with the FULL calibration, then gravity-align only
            # planning axes. The measured grasp pose is kept by rotation_goal.
            tilt = math.acos(float(np.clip(object_pose[2, 2], -1., 1.)))
            if tilt > self.policy['rotation_arm_angle_limit_rad']:
                raise ValueError('native initial tilt exceeds the existing pose envelope')
            result = object_pose.copy()
            heading = math.atan2(object_pose[1, 0], object_pose[0, 0])
            result[:3, :3] = rotation([0., 0., 1.], heading)
            self.initial_object_tilt_rad = tilt
            return result
        return object_pose

    def tick(self):
        if (getattr(self, 'rotation_session', None) is not None
                and not self.policy['rotation_ungrasped_test']
                and (not self.grasp or time.monotonic()-self.grasp_wall > 1.)):
            self.trip('local rotation grasp confirmation lost')
        super().tick()

    def trip(self, reason):
        super().trip(reason)
        self.rotation_session = None
        self.rotation_hold_start = None
        if hasattr(self, 'rotation_velocity'):
            self.rotation_velocity[:] = 0.

    def reset(self, request, response):
        response = super().reset(request, response)
        if response.success:
            self.rotation_session = None
            self.rotation_velocity[:] = 0.
            self.rotation_hold_start = None
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRotationLeader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
