"""Simulation-only native grasp monitor after an explicit preparation handoff.

No arm/gripper motion publisher, transport goal, leader state, TF or GT input.
8/26 N and mimic tolerance inherit the CP-9 numerical experiment policy;
they are not measured hardware ratings. The last gripper goal remains held.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import WrenchStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from .safety import LATCHED
from .sensor_coherence import local_sensor_qos


def measured_joints_ok(positions, limits, mimics):
    if any(n not in positions or not math.isfinite(positions[n]) or
           not lo-1e-5 <= positions[n] <= hi+1e-5 for n, (lo, hi) in limits.items()):
        return False
    return bool(mimics) and all(n in positions and math.isfinite(positions[n]) and
        lo-1e-5 <= positions[n] <= hi+1e-5 and
        abs(positions[n]-(positions['Gripper']*mult+offset)) <= .01
        for n, (lo, hi, mult, offset) in mimics.items())


def released_arm_writer(writers, robot):
    return not writers or writers == [('actuator_interlock', '/'+robot)]


class GraspLatch:
    """Initial disarm holds the grip; after first permit every stop latches."""
    def __init__(self):
        self.state, self.reason, self.ever = 'WAITING', '', False

    def takeover(self, healthy, exclusive):
        if self.state != 'WAITING' or not healthy or not exclusive:
            return False
        self.state = 'HOLDING'
        return True

    def trip(self, reason):
        if self.state == 'HOLDING':
            self.state, self.reason = 'FAULT', reason

    def permit(self, value):
        if value and self.state == 'HOLDING':
            self.ever = True

    def stop(self, value):
        if value and self.ever:
            self.trip('shared emergency stop')

    def sensor_freshness(self, healthy):
        # No motion has ever been permitted during initial discovery. Withhold
        # attached/ready then; after first permit every outage latches forever.
        if not healthy and self.ever:
            self.trip('local sensor/clock freshness or invalid measurement')


class NativeGraspMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__('native_grasp_monitor', **kwargs)
        self.robot = self.get_namespace().strip('/')
        if self.robot not in ('leader', 'follower') or not self.get_parameter('use_sim_time').value:
            raise ValueError('explicit simulation robot namespace required')
        root = ET.parse(self.declare_parameter('urdf_file', '').value).getroot()
        plugins = root.findall('ros2_control/hardware/plugin')
        if not plugins or any(p.text != 'gz_ros2_control/GazeboSimSystem' for p in plugins):
            raise ValueError('Gazebo hardware only')
        names = ['Joint_'+str(i) for i in range(1, 6)]+['Gripper']
        self.limits = {}
        for n in names:
            j = root.find("joint[@name='"+n+"']/limit")
            self.limits[n] = (float(j.get('lower')), float(j.get('upper')))
        self.mimics = {}
        for j in root.findall('joint'):
            m, limit = j.find('mimic'), j.find('limit')
            if m is not None and m.get('joint') == 'Gripper':
                self.mimics[j.get('name')+'_mimic'] = (float(limit.get('lower')),
                    float(limit.get('upper')), float(m.get('multiplier', '1')), float(m.get('offset', '0')))
        if len(self.mimics) != 5:
            raise ValueError('audited five mimic measurements required')
        self.latch = GraspLatch()
        self.data, self.positions, self.forces, self.contacts = {}, {}, {}, {}
        self.last_sim, self.clock_wall = -1., 0.
        self.permit_wall = 0.
        self.current_permit = False
        self.audit_wall, self.gripper_active = 0., False
        self.source_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.loss_since = None
        self.fault_context = {}
        self.stop_future, self.stop_ok = None, None
        self.maintenance_required = self.declare_parameter('maintenance_required', False).value
        self.maintenance_wall, self.maintenance_ready = 0., False
        self.declare_parameter('simulation_source_watchdog',False)
        self.source_watchdog=self.get_parameter('simulation_source_watchdog').value
        self.state_pub = self.create_publisher(String, 'grasp/state', 10)
        self.debug_pub = self.create_publisher(String, 'grasp/monitor_debug', 10)
        self.stop_pub = self.create_publisher(Bool, '/transport/emergency_stop', LATCHED)
        self.manager = self.create_client(SwitchController, 'controller_manager/switch_controller')
        input_qos=local_sensor_qos(self.source_watchdog)
        self.create_subscription(JointState, 'joint_states', self.joints, input_qos)
        for side in ('left', 'right'):
            self.create_subscription(WrenchStamped, 'finger_'+side+'_wrench',
                lambda msg, s=side: self.force(s, msg), input_qos)
            self.create_subscription(Bool, 'finger_'+side+'_contact_detected',
                lambda msg, s=side: self.contact(s, msg), 10)
        self.create_subscription(Bool, '/transport/emergency_stop', self.stop, LATCHED)
        self.create_subscription(Bool, '/transport/safety_permit', self.permit, 1)
        self.create_subscription(String, 'interlock/audit', self.audit, 10)
        if self.maintenance_required:
            self.create_subscription(String, 'grasp/maintenance_debug', self.maintenance, 10)
        expected = {'/clock', '/transport/emergency_stop', '/transport/safety_permit',
            *['/'+self.robot+'/'+s for s in ('joint_states', 'finger_left_wrench', 'finger_right_wrench',
                'finger_left_contact_detected', 'finger_right_contact_detected', 'interlock/audit')]}
        if self.maintenance_required:
            expected.add('/'+self.robot+'/grasp/maintenance_debug')
        if {s.topic_name for s in self.subscriptions} != expected:
            raise ValueError('local grasp boundary remapped')
        self.create_service(Trigger, 'grasp/takeover', self.takeover)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def receive(self, key, stamp):
        if not math.isfinite(stamp) or stamp <= 0 or (key in self.data and stamp <= self.data[key][0]):
            self.trip(key+': timestamp did not advance')
            return
        self.data[key] = (stamp, time.monotonic())

    def joints(self, msg):
        self.positions = dict(zip(msg.name, msg.position))
        if not measured_joints_ok(self.positions, self.limits, self.mimics):
            self.trip('local joint limit/tracking fault')
        self.receive('joints', msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def force(self, side, msg):
        w = msg.wrench
        values = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
        self.forces[side] = abs(values[0]) if all(math.isfinite(v) for v in values) else math.nan
        expected = self.robot+'/'+side+'_inner_front_joint/finger_'+side+'_force_sensor'
        if msg.header.frame_id != expected:
            self.forces[side] = math.nan
            self.trip('unexpected local finger sensor frame')
        elif not all(math.isfinite(v) for v in values):
            self.trip('nonfinite local finger force')
        elif self.forces[side] > 26.:
            self.trip('simulation grip overforce')
        self.receive(side, msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def contact(self, side, msg):
        self.contacts[side] = msg.data
        # Bool contact API has no source stamp; receipt freshness only.
        self.data[side+'_contact'] = (self.now(), time.monotonic())

    def healthy(self):
        now, wall = self.now(), time.monotonic()
        return (wall-self.clock_wall <= .5 and measured_joints_ok(self.positions, self.limits, self.mimics)
            and all(k in self.data and -.02 <= now-self.data[k][0] <= .25
                and (self.source_watchdog or wall-self.data[k][1] <= (.5 if k.endswith('_contact') else .25))
                for k in ('joints', 'left', 'right', 'left_contact', 'right_contact'))
            and all(s in self.forces and math.isfinite(self.forces[s]) and self.forces[s] <= 26.
                for s in ('left', 'right')))

    def holding(self):
        return all(self.contacts.get(s) and 8. <= self.forces.get(s, 0.) <= 26. for s in ('left', 'right'))

    def takeover(self, request, response):
        writers = self.get_publishers_info_by_topic('/'+self.robot+'/arm_controller/joint_trajectory')
        exclusive = released_arm_writer([(w.node_name, w.node_namespace) for w in writers], self.robot)
        response.success = self.latch.takeover(self.healthy() and self.holding() and not self.current_permit, exclusive)
        response.message = self.latch.state if response.success else 'fresh held grasp and preparation writer release required'
        return response

    def permit(self, msg):
        self.permit_wall = time.monotonic()
        self.current_permit = msg.data
        self.latch.permit(msg.data)

    def audit(self, msg):
        try:
            value = json.loads(msg.data)
            if value['robot'] != self.robot:
                raise ValueError('wrong local audit')
            self.gripper_active = value['controllers'].get('gripper_controller') == 'active'
            self.audit_wall = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.gripper_active = False
        if self.latch.ever and not self.gripper_active:
            self.trip('local gripper controller inactive/unknown')

    def stop(self, msg):
        if msg.data and self.latch.ever:
            self.trip('shared emergency stop')

    def maintenance(self, msg):
        try:
            value = json.loads(msg.data)
            self.maintenance_ready = value['enabled'] is True and value['state'] in ('READY', 'ACTIVE')
        except (ValueError, KeyError, TypeError):
            self.maintenance_ready = False
        self.maintenance_wall = time.monotonic()
        if self.latch.ever and not self.maintenance_ready:
            self.trip('local grip maintenance stopped')

    def trip(self, reason):
        previous = self.latch.state
        self.latch.trip(reason)
        if previous != self.latch.state:
            self.capture_fault()

    def capture_fault(self):
        now, wall = self.now(), time.monotonic()
        self.fault_context = {'sim_time': now, 'reason': self.latch.reason,
            'source_ages_s': {k: now-v[0] for k,v in self.data.items()},
            'receipt_ages_wall_s': {k: wall-v[1] for k,v in self.data.items()},
            'clock_age_wall_s': wall-self.clock_wall,
            'maintenance_heartbeat_age_wall_s':wall-self.maintenance_wall,
            'maintenance_ready':self.maintenance_ready,
            'joint_model_ok': measured_joints_ok(self.positions,self.limits,self.mimics),
            'forces_N': dict(self.forces), 'contacts': dict(self.contacts)}

    def tick(self):
        now, wall = self.now(), time.monotonic()
        if now > self.last_sim:
            self.clock_wall, self.last_sim = wall, now
        if self.latch.state == 'HOLDING':
            if not self.healthy():
                self.latch.sensor_freshness(False)
                if self.latch.state == 'FAULT':
                    self.capture_fault()
            elif self.latch.ever and wall-self.permit_wall > .15:
                self.trip('safety permit heartbeat lost')
            elif self.latch.ever and (wall-self.audit_wall > .5 or not self.gripper_active):
                self.trip('local gripper controller audit lost/inactive')
            elif self.maintenance_required and self.latch.ever and (wall-self.maintenance_wall > .15 or not self.maintenance_ready):
                self.trip('local grip maintenance heartbeat lost')
            elif not self.holding():
                self.loss_since = now if self.loss_since is None else self.loss_since
                if now-self.loss_since >= .3:
                    self.trip('local contact/holding force lost')
            else:
                self.loss_since = None
        if self.latch.state == 'FAULT':
            self.stop_pub.publish(Bool(data=True))
            if self.stop_future is None and self.manager.service_is_ready():
                req = SwitchController.Request(deactivate_controllers=['arm_controller', 'mecanum_drive_controller'],
                    strictness=SwitchController.Request.BEST_EFFORT)
                req.timeout.sec = 1
                self.stop_future = self.manager.call_async(req)
            if self.stop_future is not None and self.stop_future.done():
                try:
                    self.stop_ok = bool(self.stop_future.result().ok)
                except Exception:
                    self.stop_ok = False
        sensor_ready = self.healthy()
        self.state_pub.publish(String(data='attached' if self.latch.state == 'HOLDING' and sensor_ready else 'detached'))
        self.debug_pub.publish(String(data=json.dumps({'state': self.latch.state, 'reason': self.latch.reason,
            'sim_time': now, 'ever_permitted': self.latch.ever, 'own_forces_N': self.forces,
            'own_contacts': self.contacts, 'positions': self.positions, 'healthy': sensor_ready,
            'readiness': 'READY' if sensor_ready and self.latch.state=='HOLDING' else 'NOT_READY',
            'first_fault_context': self.fault_context,
            'base_arm_deactivation_response': self.stop_ok, 'arm_motion_publisher_count': 0,
            'maintenance_required':self.maintenance_required,'maintenance_ready':self.maintenance_ready,
            'maintenance_heartbeat_age_wall_s':time.monotonic()-self.maintenance_wall,
            'source_sha256': self.source_hash, 'simulation_only': True})))


def main(args=None):
    rclpy.init(args=args)
    node = NativeGraspMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
