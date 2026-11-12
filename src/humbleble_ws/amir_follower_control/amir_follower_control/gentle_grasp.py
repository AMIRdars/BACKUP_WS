"""Contact-limited startup; fixed attachment and local friction-holding adapters."""
import math
from collections import deque

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import SetParametersResult

from cooperative_transport_control.attach_manager import AttachManager
from cooperative_transport_control.grip_force_control import next_gripper_position,holding_recovery_position


class ManagedAttach(AttachManager):
    def __init__(self):
        super().__init__()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.verified_pub=self.create_publisher(Bool,'/cooperative_transport/contact_closure_verified',qos)
        self.create_timer(.1,self._publish_verified)

    def _publish_verified(self):
        self.verified_pub.publish(Bool(data=self._gripper_close_complete and
                                      all(self._states.get(r) is True for r in self._names)))


class GentleAttach(ManagedAttach):
    completion_text='attaching fixed joints'
    def __init__(self):
        super().__init__()
        self.declare_parameter('contact_force_n', 3.)
        self.positions = {}
        self.forces = {}
        self.raw_forces = {}
        self.force_peaks = deque()
        self.local_force_peaks = {}
        self.bias = {}
        self.contact_current = {}
        self.ready_since = None
        self.searching = False
        self.verified_ready = False
        self.pending_goals = {}
        self.last_log = -math.inf
        for robot in self._names:
            self.create_subscription(JointState, f'/{robot}/joint_states',
                lambda m, r=robot: self._joints(r,m), qos_profile_sensor_data)
            for side in ('left','right'):
                self.create_subscription(WrenchStamped, f'/{robot}/finger_{side}_wrench',
                    lambda m,r=robot,s=side: self._force(r,s,m), qos_profile_sensor_data)
        self.create_timer(.05, self._regulate)

    def _now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def _joints(self, robot, msg):
        mapping=dict(zip(msg.name,msg.position))
        q = mapping.get('Gripper')
        if q is not None and math.isfinite(q):
            self.positions[robot] = (q,self._now())

    def _force(self, robot, side, msg):
        key = (robot,side)
        value = msg.wrench.force.x
        if not math.isfinite(value):
            return
        if not self.searching and not self._gripper_close_complete:
            count,mean = self.bias.get(key,(0,0.))
            count += 1
            self.bias[key] = (count,mean+(value-mean)/count)
        now=self._now()
        raw=abs(value-self.bias.get(key,(0,0.))[1])
        self.raw_forces[key]=(raw,now)
        previous,previous_time=self.forces.get(key,(raw,now))
        alpha=1-math.exp(-max(0.,now-previous_time)/.03)
        self.forces[key] = (previous+alpha*(raw-previous),now)
        self.force_peaks.append((now,raw))
        peaks=self.local_force_peaks.setdefault(robot,deque())
        if peaks and now<peaks[-1][0]:peaks.clear()
        peaks.append((now,raw))
        while peaks and now-peaks[0][0]>.3:peaks.popleft()
        while self.force_peaks and now-self.force_peaks[0][0]>.3:
            self.force_peaks.popleft()

    def _on_finger_contact(self, robot, side, msg):
        payload = self.get_parameter('payload_collision_name').value
        if any(payload in (c.collision1.name,c.collision2.name) for c in msg.contacts):
            # Multiple collision sensors share this logical finger topic.
            # An empty segment cannot clear another segment's recent contact.
            self.contact_current[(robot,side)] = (True,self._now())
        super()._on_finger_contact(robot,side,msg)

    def _close_grippers_then_attach(self):
        if self._gripper_close_complete:
            return
        if not self.searching:
            self.searching = True
            self._gripper_close_pending = True
            self.get_logger().info('Starting contact-limited closure; target='
                f'{self.get_parameter("contact_force_n").value} N per finger')

    def _try_finish_gripper_close(self):
        if getattr(self,'verified_ready',False):
            super()._try_finish_gripper_close()

    def _regulate(self):
        if not self.searching or self._gripper_close_complete:
            return
        now = self._now()
        target = float(self.get_parameter('contact_force_n').value)
        keys = [(r,s) for r in self._names for s in ('left','right')]
        fresh = all(k in self.forces and now-self.forces[k][1]<.15 for k in keys)
        fresh = fresh and all(r in self.positions and now-self.positions[r][1]<.15 for r in self._names)
        if not fresh:
            self.ready_since = None
            return
        contact = {k: self.contact_current.get(k,(False,-math.inf)) for k in keys}
        ready = all(contact[k][0] and now-contact[k][1]<.15 and
                    1. <= self.forces[k][0] <= 8. for k in keys)
        ready = ready and all(force<=8. for _,force in self.force_peaks)
        if ready:
            self.ready_since = now if self.ready_since is None else self.ready_since
            if now-self.ready_since >= .3:
                self.verified_ready = True
                self.searching = False
                self.get_logger().info(f'Gentle contact verified: {self.forces}; {self.completion_text}')
                self._try_finish_gripper_close()
                return
        else:
            self.ready_since = None
        for robot in self._names:
            if robot in self.pending_goals and not self.pending_goals[robot].done():
                continue
            if not self._gripper_clients[robot].server_is_ready():
                continue
            forces = [self.forces[(robot,s)][0] for s in ('left','right')]
            bilateral = all(contact[(robot,s)][0] for s in ('left','right'))
            position = next_gripper_position(self.positions[robot][0],forces,bilateral,
                target,1.,.00005,.0002,.0005,-1.,.20)
            if max(self.raw_forces[(robot,s)][0] for s in ('left','right'))>8.:
                position=max(-1.,self.positions[robot][0]-.0002)
            self._send_position(robot,position)
        if now-self.last_log>1.:
            self.last_log = now
            self.get_logger().info(f'Contact search q={self.positions}, filtered={self.forces}, '
                                   f'raw={self.raw_forces}, contact={contact}')

    def _send_position(self,robot,position):
        goal=GripperCommand.Goal()
        goal.command.position=position
        goal.command.max_effort=.8
        self.pending_goals[robot]=self._gripper_clients[robot].send_goal_async(goal)


class GentleFrictionGrasp(GentleAttach):
    """Contact-only variant using each robot's own contact/load heartbeat."""
    completion_text='holding by contact and friction only'
    def __init__(self):
        super().__init__()
        self.declare_parameter('holding_force_n',5.)
        self.declare_parameter('holding_position_gain',.00015)
        self.declare_parameter('holding_max_step_rad',.0003)
        self.declare_parameter('holding_interval_s',.1)
        valid=self._validate_holding_parameters([])
        if not valid.successful:
            raise ValueError(valid.reason)
        self.add_on_set_parameters_callback(self._validate_holding_parameters)
        self.holding_commands={}
        self.last_hold=-math.inf
        self.armed={robot:False for robot in self._names}
        self.local_states={robot:self.create_publisher(String,f'/{robot}/grasp/state',10)
                           for robot in self._names}
        self.create_timer(.1,self._local_heartbeat)
        # Independent from the 20 Hz startup closure timer. The interval gate
        # below permits 50 Hz holding with smaller per-update commands.
        self.create_timer(.01,self._holding_tick)

    def _validate_holding_parameters(self,proposed):
        names=('contact_force_n','holding_force_n','holding_position_gain',
               'holding_max_step_rad','holding_interval_s')
        try:
            values={name:float(self.get_parameter(name).value) for name in names}
            values.update({p.name:float(p.value) for p in proposed if p.name in names})
            good=all(math.isfinite(value) for value in values.values())
            good=good and 0<values['contact_force_n']<=7 and 3<=values['holding_force_n']<=6
            good=good and 0<values['holding_position_gain']<=.0003
            good=good and .02<=values['holding_interval_s']<=.2 and values['holding_max_step_rad']>0
            good=good and values['holding_max_step_rad']/values['holding_interval_s']<=.004+1e-12
        except (ValueError,TypeError):
            good=False
        return SetParametersResult(successful=good,
            reason='' if good else 'finite staged grip gains and holding speed <= 0.004 rad/s required')

    def _publish_command(self, robot, attach):
        # No fixed constraint is created. The existing friction payload and
        # finger contacts provide all support; reported state is measured.
        self.armed[robot]=bool(attach)

    def _try_finish_gripper_close(self):
        if getattr(self,'verified_ready',False) and self._gripper_close_pending:
            self._gripper_close_pending=False
            self._gripper_close_complete=True
            self._publish_command_pair(True)

    def _regulate(self):
        if not self._gripper_close_complete:
            return super()._regulate()
        self._holding_tick()

    def _holding_tick(self):
        if not self._gripper_close_complete:return
        now=self._now()
        if now-self.last_hold+1e-9<float(self.get_parameter('holding_interval_s').value):
            return
        self.last_hold=now
        for robot in self._names:
            keys=[(robot,side) for side in ('left','right')]
            if robot not in self.positions or not 0<=now-self.positions[robot][1]<.15:
                continue
            if not all(key in self.forces and 0<=now-self.forces[key][1]<.15 and
                       key in self.raw_forces and 0<=now-self.raw_forces[key][1]<.15 and
                       self.contact_current.get(key,(False,-math.inf))[0] and
                       0<=now-self.contact_current[key][1]<.15 for key in keys):
                continue
            if robot in self.pending_goals and not self.pending_goals[robot].done():
                continue
            if not self._gripper_clients[robot].server_is_ready():
                continue
            q=self.positions[robot][0]
            loads=[self.forces[key][0] for key in keys]
            position=holding_recovery_position(q,self.holding_commands.get(robot,q),loads,
                float(self.get_parameter('holding_force_n').value),1.5,7.0,
                float(self.get_parameter('holding_position_gain').value),
                float(self.get_parameter('holding_max_step_rad').value),
                -1.,.20,allow_release=True,maintain_target=True,force_tolerance=.3)
            if max(self.raw_forces[key][0] for key in keys)>8.:
                position=max(-1.,q-float(self.get_parameter('holding_max_step_rad').value))
            self.holding_commands[robot]=position
            if abs(position-q)>1e-7:
                self._send_position(robot,position)

    def _local_heartbeat(self):
        now=self._now()
        for robot in self._names:
            good=self.armed[robot]
            good=good and all(force<=8. for stamp,force in self.local_force_peaks.get(robot,())
                              if 0<=now-stamp<=.3)
            for side in ('left','right'):
                key=(robot,side)
                contact,stamp=self.contact_current.get(key,(False,-math.inf))
                load,load_stamp=self.forces.get(key,(0.,-math.inf))
                raw,raw_stamp=self.raw_forces.get(key,(0.,-math.inf))
                good=good and contact and 0<=now-stamp<.15 and 0<=now-load_stamp<.15
                good=good and 0<=now-raw_stamp<.15 and .5<=load<=8. and raw<=8.
            if self._states.get(robot) is True and not good:
                keys=[(robot,side) for side in ('left','right')]
                self.get_logger().warning(f'{robot} local contact/load guard failed; '
                    f'contacts={[self.contact_current.get(k) for k in keys]}, '
                    f'filtered={[self.forces.get(k) for k in keys]}, '
                    f'raw={[self.raw_forces.get(k) for k in keys]}')
            self._states[robot]=bool(good)
            self.local_states[robot].publish(String(data='attached' if good else 'detached'))
            self._publish_state(robot)


def run(kind,args=None):
    rclpy.init(args=args)
    node = kind()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    run(GentleAttach,args)


def managed_main(args=None):
    run(ManagedAttach,args)


def friction_main(args=None):
    run(GentleFrictionGrasp,args)
