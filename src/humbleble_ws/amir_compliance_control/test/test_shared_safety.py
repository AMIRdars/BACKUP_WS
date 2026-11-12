"""Real ROS nodes with fault-injected local sensors; no Gazebo or GT dependency."""
import hashlib
import json
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from amir_compliance_control.safety import RobotSafetyGate, SafetySupervisor


class Rig:
    def __init__(self, tmp_path, coherent=False, reaction_delay=0.,source_watchdog=False,slow=False):
        rclpy.init(args=[])
        self.executor = SingleThreadedExecutor()
        self.driver = rclpy.create_node('synthetic_local_sensors')
        self.executor.add_node(self.driver)
        self.gates = {}
        self.pubs = {}
        self.output, self.seen_motion = {}, set()
        self.sim, self.inject, self.send_commands = 10., None, False
        self.reaction_delay=reaction_delay
        self.slow=slow;self.iteration=0
        joints = []
        parent = 'base_footprint'
        for i, effort in enumerate((33, 22, 26, 13, 4), 1):
            child = 'gripper_base_1' if i == 5 else 'link'+str(i)
            joints.append(f'<joint name="Joint_{i}" type="revolute"><parent link="{parent}"/>'
                f'<child link="{child}"/><origin xyz=".1 0 0"/><axis xyz="0 0 1"/>'
                f'<limit lower="-2" upper="2" velocity="1" effort="{effort}"/></joint>')
            parent = child
        xml = '<robot>'+''.join(joints)+('<joint name="tcp_fixed" type="fixed">'
            '<parent link="gripper_base_1"/><child link="tcp_link"/></joint></robot>')
        urdf = tmp_path/'test.urdf'
        urdf.write_text(xml)
        for robot in ('leader', 'follower'):
            profile = tmp_path/(robot+'.json')
            profile.write_text(json.dumps({'robot': robot, 'urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
                'simulation_only': True, 'joint_reaction_projection_verified': True,
                'max_base_velocity': [.01, .01, .02], 'joint_margin_rad': .01}))
            gate = RobotSafetyGate(namespace='/'+robot, parameter_overrides=[
                Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(urdf)),
                Parameter('coherent_sensor_history',value=coherent),
                Parameter('simulation_source_watchdog',value=source_watchdog),
                Parameter('profile_file', value=str(profile))])
            self.gates[robot] = gate
            self.executor.add_node(gate)
            for key, suffix, typ in [('joints', 'joint_states', JointState),
                ('encoder', 'mecanum_drive_controller/odometry', Odometry), ('cmd', 'base_cmd', TwistStamped),
                *[(f'Joint_{i}', f'joint_load/Joint_{i}' if i < 5 else 'ft_sensor', WrenchStamped)
                  for i in range(1, 6)]]:
                self.pubs[(robot, key)] = self.driver.create_publisher(typ, '/'+robot+'/'+suffix, 10)
            self.pubs[(robot, 'grasp')] = self.driver.create_publisher(String, '/'+robot+'/grasp/state', 10)
            self.driver.create_subscription(Twist, '/'+robot+'/rover_twist',
                lambda m, r=robot: self.record_output(r, m), 10)
        self.clock_pub = self.driver.create_publisher(Clock, '/clock', 10)
        self.supervisor = SafetySupervisor(parameter_overrides=[Parameter('use_sim_time', value=True),
            Parameter('check_writers_on_arm', value=False)])
        self.executor.add_node(self.supervisor)

    def record_output(self, robot, msg):
        self.output[robot] = msg.linear.x
        if abs(msg.linear.x) > .001:
            self.seen_motion.add(robot)

    def pump(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.iteration+=1
            if self.inject != 'clock_freeze':
                self.sim += .004 if self.slow else .01
                self.clock_pub.publish(Clock(clock=Time(seconds=self.sim).to_msg()))
            for robot, gate in self.gates.items():
                q = [0.]*5
                if robot == 'follower' and self.inject == 'joint_limit':
                    q[1] = 2.1
                if not (robot == 'follower' and self.inject == 'missing_joints'):
                    m = JointState(name=list(gate.model.names), position=q, velocity=[0.]*5)
                    m.header.stamp = Time(seconds=self.sim).to_msg()
                    self.pubs[(robot, 'joints')].publish(m)
                o = Odometry()
                o.header.stamp, o.header.frame_id = Time(seconds=self.sim).to_msg(), robot+'/odom'
                o.child_frame_id = robot+'/base_footprint'
                o.pose.pose.orientation.w = 1.
                if robot == 'follower' and self.inject == 'bad_encoder':
                    o.pose.pose.position.x = float('nan')
                self.pubs[(robot, 'encoder')].publish(o)
                if not (robot == 'follower' and self.inject == 'grasp_missing'):
                    self.pubs[(robot, 'grasp')].publish(String(data=(
                        'detached' if robot == 'follower' and self.inject == 'grasp_detached' else 'attached')))
                for i, name in enumerate(gate.model.names):
                    if self.slow and self.iteration%3:continue
                    if robot == 'follower' and i == 4 and self.inject == 'missing_ft':
                        continue
                    m = WrenchStamped()
                    m.header.stamp = Time(seconds=self.sim-self.reaction_delay).to_msg()
                    m.header.frame_id = (robot+'/Joint_5/wrist_ft_sensor' if i == 4 else
                                        robot+'/'+name+'/joint_load_'+name)
                    if robot == 'follower' and i == 1 and self.inject == 'overload':
                        m.wrench.torque.z = 22.1
                    if robot == 'follower' and i == 4 and self.inject == 'nonfinite':
                        m.wrench.force.x = float('nan')
                    self.pubs[(robot, name)].publish(m)
                if self.send_commands:
                    m = TwistStamped()
                    m.header.stamp = Time(seconds=self.sim-(0. if self.slow else .02)).to_msg()
                    m.header.frame_id = robot+'/base_footprint'
                    m.twist.linear.x = .005
                    self.pubs[(robot, 'cmd')].publish(m)
            # Process all callbacks between 100 Hz sensor publications.
            deadline = time.monotonic()+(.06 if self.slow else .01)
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=.0001)

    def service(self, name):
        client = self.driver.create_client(Trigger, name)
        deadline = time.monotonic()+2.
        while not client.service_is_ready() and time.monotonic() < deadline:
            self.pump(.02)
        assert client.service_is_ready()
        future = client.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < deadline:
            self.pump(.02)
        assert future.done()
        result = future.result()
        self.driver.destroy_client(client)
        return result

    def start(self):
        self.pump(.5)
        assert self.service('/transport/safety/arm').success, self.supervisor.healthy()
        self.pump(.08)
        self.send_commands = True
        self.pump(.15)
        assert self.seen_motion == {'leader', 'follower'}, (self.supervisor.fault, [g.fault for g in self.gates.values()])

    def close(self):
        for node in (*self.gates.values(), self.supervisor, self.driver):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()


@pytest.fixture
def rig(tmp_path):
    instance = Rig(tmp_path)
    yield instance
    instance.close()


@pytest.mark.parametrize('fault', ['missing_ft', 'missing_joints', 'nonfinite', 'overload',
                                   'joint_limit', 'clock_freeze', 'clock_reverse', 'supervisor_dead', 'planner_dead',
                                   'grasp_missing', 'grasp_detached', 'bad_encoder'])
def test_either_robot_fault_stops_both_and_latches(rig, fault):
    rig.start()
    if fault == 'clock_reverse':
        rig.sim -= 2.
    elif fault == 'supervisor_dead':
        rig.supervisor.timer.cancel()
    elif fault == 'planner_dead':
        rig.send_commands = False
    else:
        rig.inject = fault
    rig.pump(.6 if fault == 'grasp_missing' else .4)
    assert rig.output == {'leader': 0., 'follower': 0.}, (rig.output, rig.supervisor.fault)
    assert rig.supervisor.fault or all(g.fault for g in rig.gates.values())
    rig.inject = None
    rig.send_commands = True
    rig.pump(.15)
    assert rig.output == {'leader': 0., 'follower': 0.}


def test_manual_rearm_does_not_replay_previous_commands(rig):
    rig.start()
    rig.inject = 'overload'
    rig.pump(.15)
    rig.inject, rig.send_commands = None, False
    rig.pump(.2)
    assert rig.output == {'leader': 0., 'follower': 0.}
    for robot in rig.gates:
        assert rig.service('/'+robot+'/safety/reset').success
    rig.pump(.06)
    assert rig.service('/transport/safety/reset').success
    assert rig.service('/transport/safety/arm').success
    rig.pump(.1)
    assert rig.output == {'leader': 0., 'follower': 0.}
    rig.send_commands = True
    rig.pump(.1)
    assert all(v > 0 for v in rig.output.values())


def test_follower_safety_reads_only_local_state_and_stop_bus(rig):
    names = {s.topic_name for s in rig.gates['follower'].subscriptions}
    assert not any(t.startswith('/leader/') or 'ground_truth' in t or t.endswith('/tf') for t in names)


@pytest.mark.parametrize('fault',['missing_ft','overload','joint_limit','bad_encoder'])
def test_coherent_history_accepts_async_state_but_still_stops_both(tmp_path,fault):
    instance=Rig(tmp_path,coherent=True,reaction_delay=.03)
    try:
        instance.start()
        assert all(g.coherent_sample_stamps for g in instance.gates.values())
        instance.inject=fault
        instance.pump(.4)
        assert instance.output=={'leader':0.,'follower':0.}
        assert instance.supervisor.fault or all(g.fault for g in instance.gates.values())
    finally:instance.close()


@pytest.mark.parametrize('fault',['missing_ft','overload','joint_limit','clock_freeze'])
def test_simulation_sensor_clock_accepts_slow_frames_but_preserves_shared_stop(tmp_path,fault):
    instance=Rig(tmp_path,coherent=True,source_watchdog=True,slow=True)
    try:
        instance.start()  # FT frames every ~180 ms wall, only 12 ms simulation
        instance.inject=fault;instance.pump(1.5)
        assert instance.output=={'leader':0.,'follower':0.}
        assert instance.supervisor.fault or all(g.fault for g in instance.gates.values())
    finally:instance.close()


@pytest.mark.parametrize('duplicate', [False, True])
def test_arming_requires_exclusive_real_command_writers(rig, duplicate):
    rig.supervisor.set_parameters([Parameter('check_writers_on_arm', value=True)])
    if duplicate:
        rig.driver.create_publisher(Twist, '/leader/rover_twist', 10)
    rig.pump(.5)
    response = rig.service('/transport/safety/arm')
    assert not response.success and 'ownership not confirmed' in response.message
    assert rig.output == {'leader': 0., 'follower': 0.}
