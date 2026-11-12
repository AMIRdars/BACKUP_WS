"""Opt-in simulation grip maintenance from the local independent monitor.

The 17--23 N operating band, .0002 rad step and 1.2 action effort inherit
CP-9 preparation. They are numerical experiment settings, not hardware ratings.
No arm/base commands, transport goals, TF, leader state or ground truth input.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from control_msgs.action import GripperCommand
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor
from . import native_grasp
from .safety import LATCHED

CORRECTION_PERIOD_S = .5
LOW_FORCE_CONFIRM_S = .3


def closing_target(position, forces, contacts, limits, previous=None, hold_safe_force_imbalance=False):
    """One measured-position correction; never integrate an unachieved goal."""
    if (not math.isfinite(position) or not limits[0] <= position <= limits[1]
            or len(forces) != 2 or not all(math.isfinite(f) and 0. <= f <= 26. for f in forces)
            or len(contacts) != 2):
        raise ValueError('local contact/force/joint outside held-grasp envelope')
    # Never regrasp during a contact/low-force transient. The independent
    # monitor retains its original .3 simulated-second loss confirmation.
    if not all(contacts) or min(forces) < 8.:
        return None
    if min(forces) >= 17.:
        return None
    if max(forces) >= 23.:
        if hold_safe_force_imbalance:return None
        raise ValueError('finger force imbalance prevents bounded closing correction')
    target = position + .0002
    if previous is not None:
        # Passive position excursions must never lower the held action target.
        # Bound changes relative to both the measurement and the last goal.
        target = min(target, previous + .0002)
        if target <= max(position, previous) + 1e-12:
            return None
    if target > limits[1]:
        raise ValueError('gripper closing correction would cross joint limit')
    return target


class GripMaintenance(Node):
    def __init__(self, **kwargs):
        super().__init__('local_grip_maintenance', **kwargs)
        self.robot = self.get_namespace().strip('/')
        if self.robot not in ('leader', 'follower') or not self.get_parameter('use_sim_time').value:
            raise ValueError('explicit simulation namespace and clock required')
        self.assert_local_names()
        model = ET.parse(self.declare_parameter('urdf_file', '').value).getroot()
        plugins = model.findall('ros2_control/hardware/plugin')
        if not plugins or any(p.text != 'gz_ros2_control/GazeboSimSystem' for p in plugins):
            raise ValueError('Gazebo hardware only')
        limit = model.find("joint[@name='Gripper']/limit")
        self.limits = (float(limit.get('lower')), float(limit.get('upper')))
        if float(limit.get('velocity')) < .0002 / CORRECTION_PERIOD_S:
            raise ValueError('model does not support the inherited small correction rate')
        self.expected_monitor_hash = hashlib.sha256(Path(native_grasp.__file__).read_bytes()).hexdigest()
        self.source_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.hold_safe_force_imbalance=self.declare_parameter('hold_safe_force_imbalance',False,
            ParameterDescriptor(read_only=True)).value
        if not isinstance(self.hold_safe_force_imbalance,bool):raise ValueError('boolean imbalance hold policy required')
        self.state, self.reason = 'DISABLED', ''
        self.fault_context = {}
        self.sample, self.sample_wall = {}, 0.
        self.permitted, self.ever_permitted, self.permit_wall = False, False, 0.
        self.last_sim, self.clock_wall = -1., 0.
        self.last_command_sim = -math.inf
        self.pending, self.pending_wall = None, 0.
        self.sent, self.accepted, self.command = 0, 0, None
        self.initial_hold = None
        self.low_force_since = None
        self.stop_pub = self.create_publisher(Bool, '/transport/emergency_stop', LATCHED)
        self.debug_pub = self.create_publisher(String, 'grasp/maintenance_debug', 10)
        self.client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.create_subscription(String, 'grasp/monitor_debug', self.monitor, 10)
        self.create_subscription(Bool, '/transport/safety_permit', self.permit, 1)
        self.create_subscription(Bool, '/transport/emergency_stop', self.stop, LATCHED)
        self.create_service(Trigger, 'grasp/enable_maintenance', self.enable)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def assert_local_names(self):
        topics = ['/clock', '/transport/safety_permit', '/transport/emergency_stop',
                  '/'+self.robot+'/grasp/monitor_debug']
        action = '/'+self.robot+'/gripper_controller/gripper_cmd'
        topics += [action, action+'/_action/status', action+'/_action/feedback']
        if any(self.resolve_topic_name(name) != name for name in topics):
            raise ValueError('local grip input/action boundary remapped')
        if any(self.resolve_service_name(action+'/_action/'+suffix) != action+'/_action/'+suffix
               for suffix in ('send_goal', 'get_result', 'cancel_goal')):
            raise ValueError('local gripper action service boundary remapped')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def monitor(self, msg):
        try:
            sample = json.loads(msg.data)
            if sample['source_sha256'] != self.expected_monitor_hash:
                raise ValueError('unexpected local monitor source')
            self.sample, self.sample_wall = sample, time.monotonic()
            if self.state not in ('DISABLED', 'FAULT') and sample['state'] != 'HOLDING':
                self.trip('local independent grasp monitor stopped')
        except (ValueError, KeyError, TypeError):
            if self.state != 'DISABLED':
                self.trip('invalid local monitor measurement')

    def fresh(self):
        s, now, wall = self.sample, self.now(), time.monotonic()
        return (wall-self.clock_wall <= .5 and wall-self.sample_wall <= .25
            and s.get('state') == 'HOLDING' and s.get('healthy') is True
            and math.isfinite(s.get('sim_time', math.nan))
            and -.02 <= now-s['sim_time'] <= .25)

    def enable(self, request, response):
        writers = self.get_publishers_info_by_topic('/'+self.robot+'/grasp/monitor_debug')
        own_monitor = len(writers) == 1 and writers[0].node_name == 'native_grasp_monitor' and writers[0].node_namespace == '/'+self.robot
        preparations = any(name == 'cp9_contact_preparation' and namespace == '/'+self.robot
                           for name, namespace in self.get_node_names_and_namespaces())
        response.success = (self.state == 'DISABLED' and not self.permitted and not self.ever_permitted and not preparations
            and not self.sample.get('ever_permitted', True) and self.fresh()
            and own_monitor and self.client.server_is_ready())
        if response.success:
            try:
                if not all(self.sample['own_contacts'][s] and 8. <= self.sample['own_forces_N'][s] <= 26.
                           for s in ('left', 'right')):
                    raise ValueError('held grasp required before enable')
                closing_target(self.sample['positions']['Gripper'],
                    [self.sample['own_forces_N'][s] for s in ('left', 'right')],
                    [self.sample['own_contacts'][s] for s in ('left', 'right')], self.limits,
                    hold_safe_force_imbalance=self.hold_safe_force_imbalance)
            except (ValueError, KeyError, TypeError):
                response.success = False
        if response.success:
            self.initial_hold = self.sample['positions']['Gripper']
            self.command = self.initial_hold
            self.state = 'READY'
        response.message = self.state if response.success else 'fresh local grasp, retired preparation and explicit unarmed enable required'
        return response

    def permit(self, msg):
        self.permitted, self.permit_wall = msg.data, time.monotonic()
        self.ever_permitted = self.ever_permitted or msg.data
        if not msg.data and self.ever_permitted and self.state != 'DISABLED':
            self.trip('shared safety permit revoked')

    def stop(self, msg):
        if msg.data and self.ever_permitted and self.state != 'DISABLED':
            self.trip('shared emergency stop')

    def trip(self, reason):
        if self.state != 'FAULT':
            self.state, self.reason = 'FAULT', reason
            wall = time.monotonic()
            self.fault_context = {'sim_time': self.now(),
                'monitor_sim_time': self.sample.get('sim_time'),
                'monitor_receipt_age_wall_s': wall-self.sample_wall,
                'monitor_healthy': self.sample.get('healthy'),
                'clock_receipt_age_wall_s': wall-self.clock_wall,
                'permit_receipt_age_wall_s': wall-self.permit_wall}
        self.stop_pub.publish(Bool(data=True))

    def acknowledge(self, future):
        self.pending = None
        try:
            if not future.result().accepted:
                self.trip('local gripper action rejected')
            else:
                self.accepted += 1
        except Exception as error:
            self.trip('local gripper action error: '+str(error))

    def tick(self):
        now, wall = self.now(), time.monotonic()
        if now > self.last_sim:
            self.last_sim, self.clock_wall = now, wall
        if self.state in ('READY', 'ACTIVE'):
            if not self.fresh():
                self.trip('local monitor/clock freshness lost')
            elif self.ever_permitted and (not self.permitted or wall-self.permit_wall > .15):
                self.trip('shared permit heartbeat lost')
            elif self.pending is not None and wall-self.pending_wall > .15:
                self.trip('local gripper action acknowledgement timeout')
            elif self.permitted:
                self.state = 'ACTIVE'
                try:
                    target = closing_target(self.sample['positions']['Gripper'],
                        [self.sample['own_forces_N'][s] for s in ('left', 'right')],
                        [self.sample['own_contacts'][s] for s in ('left', 'right')], self.limits,
                        previous=self.command,hold_safe_force_imbalance=self.hold_safe_force_imbalance)
                    if target is None:
                        self.low_force_since = None
                    elif self.low_force_since is None:
                        self.low_force_since = now
                    confirmed = self.low_force_since is not None and now-self.low_force_since >= LOW_FORCE_CONFIRM_S
                    if target is not None and confirmed and self.pending is None and now-self.last_command_sim >= CORRECTION_PERIOD_S:
                        if not self.client.server_is_ready():
                            raise ValueError('local gripper action server unavailable')
                        goal = GripperCommand.Goal()
                        goal.command.position, goal.command.max_effort = target, 1.2
                        self.command, self.last_command_sim = target, now
                        self.sent += 1
                        self.pending_wall = wall
                        self.pending = self.client.send_goal_async(goal)
                        self.pending.add_done_callback(self.acknowledge)
                except (ValueError, KeyError, TypeError) as error:
                    self.trip(str(error))
        if self.state == 'FAULT':
            self.stop_pub.publish(Bool(data=True))
        self.debug_pub.publish(String(data=json.dumps({'state': self.state, 'reason': self.reason,
            'tick_gap_wall_s':wall-getattr(self,'last_tick_wall',wall),
            'enabled': self.state in ('READY', 'ACTIVE'), 'sim_time': now,
            'sent_goals': self.sent, 'accepted_goals': self.accepted,
            'last_command_rad': self.command, 'last_command_sim_s': self.last_command_sim if self.sent else None,
            'initial_held_position_rad': self.initial_hold,
            'low_force_since_sim_s': self.low_force_since,
            'hold_safe_force_imbalance': self.hold_safe_force_imbalance,
            'first_fault_context': self.fault_context,
            'source_sha256': self.source_hash, 'simulation_only': True})))
        self.last_tick_wall=wall


def main(args=None):
    rclpy.init(args=args)
    node = GripMaintenance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
