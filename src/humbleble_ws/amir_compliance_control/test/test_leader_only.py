"""CP-4 ROS boundaries and lifecycle checks without Gazebo."""
import json
import time
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from amir_compliance_control.safety import LATCHED
from amir_compliance_control.transport import TransportEndpoint, straight_reference


@pytest.mark.parametrize('distance', [.001, -.001, 1e-5, 0.])
def test_signed_reference_integral_and_bounds(distance):
    _, _, duration = straight_reference(0., distance, .00025, .001)
    t = np.linspace(0., max(duration, .01), 10001)
    v = np.array([straight_reference(x, distance, .00025, .001)[1] for x in t])
    assert abs(np.sum((v[1:]+v[:-1])*.5*np.diff(t))-distance) < 1e-9
    assert np.max(np.abs(v)) <= .00025+1e-12
    assert np.max(np.abs(np.diff(v)/np.diff(t))) <= .001+1e-9
    assert straight_reference(duration+1., distance, .00025, .001)[:2] == (distance, 0.)


class Rig:
    def __init__(self, path):
        rclpy.init(args=[])
        root = ET.Element('robot'); parent = 'base_footprint'
        control = ET.SubElement(root, 'ros2_control')
        for i in range(1, 6):
            child = 'gripper_base_1' if i == 5 else 'link'+str(i)
            j = ET.SubElement(root, 'joint', name='Joint_'+str(i), type='revolute')
            ET.SubElement(j, 'parent', link=parent); ET.SubElement(j, 'child', link=child)
            ET.SubElement(j, 'axis', xyz='0 0 1')
            ET.SubElement(j, 'limit', lower='-2', upper='2', velocity='1', effort='4')
            s = ET.SubElement(ET.SubElement(control, 'joint', name='Joint_'+str(i)), 'state_interface', name='position')
            ET.SubElement(s, 'param', name='initial_value').text = '0'
            parent = child
        j = ET.SubElement(root, 'joint', name='tcp', type='fixed')
        ET.SubElement(j, 'parent', link=parent); ET.SubElement(j, 'child', link='tcp_link')
        self.file = path/'test.urdf'; self.file.write_text(ET.tostring(root, encoding='unicode'))
        self.executor = SingleThreadedExecutor()
        self.driver = rclpy.create_node('experiment_driver')
        self.nodes, self.ends, self.pubs, self.output, self.command_history = [self.driver], {}, {}, {}, []
        for robot in ('leader', 'follower'):
            end = TransportEndpoint(leader=robot == 'leader', namespace='/'+robot, parameter_overrides=[
                Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(self.file))])
            self.ends[robot] = end; self.nodes.append(end)
            for key, name, typ, topic in [
                    ('encoder', 'mecanum_drive_controller', Odometry, 'mecanum_drive_controller/odometry'),
                    ('joints', 'joint_state_broadcaster', JointState, 'joint_states')]:
                node = rclpy.create_node(name, namespace='/'+robot); self.nodes.append(node)
                self.pubs[robot, key] = node.create_publisher(typ, '/'+robot+'/'+topic, 10)
            self.pubs[robot, 'gate'] = self.driver.create_publisher(String, '/transport/safety/'+robot, 10)
            self.pubs[robot, 'grasp'] = self.driver.create_publisher(String, '/'+robot+'/grasp/state', 10)
            self.driver.create_subscription(TwistStamped, '/'+robot+'/base_cmd',
                lambda m, r=robot: self.record_command(r, m), 10)
        self.clock_pub = self.driver.create_publisher(Clock, '/clock', 10)
        self.permit_pub = self.driver.create_publisher(Bool, '/transport/safety_permit', 1)
        self.stop_pub = self.driver.create_publisher(Bool, '/transport/emergency_stop', LATCHED)
        self.goal_pub = self.driver.create_publisher(Vector3Stamped, '/leader/transport_goal', 1)
        self.stop_calls = 0
        self.driver.create_service(Trigger, '/transport/safety/stop', self.stop_service)
        for node in self.nodes: self.executor.add_node(node)
        self.sim, self.permit, self.seq, self.freeze_encoder = 10., False, 0, False
        self.sim_step = .02
        self.pose = {'leader': 0., 'follower': 0.}

    def stop_service(self, request, response):
        self.stop_calls += 1; self.permit = False; response.success = True
        return response

    def record_command(self, robot, msg):
        self.output[robot] = msg.twist.linear.x
        if robot == 'leader':
            self.command_history.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, msg.twist.linear.x))

    def pump(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.sim += self.sim_step; self.seq += 1
            self.clock_pub.publish(Clock(clock=Time(seconds=self.sim).to_msg()))
            self.permit_pub.publish(Bool(data=self.permit)); self.stop_pub.publish(Bool(data=not self.permit))
            for robot in self.ends:
                self.pose[robot] += self.output.get(robot, 0.)*.02
                o = Odometry(); o.header.stamp = Time(seconds=self.sim).to_msg()
                o.header.frame_id, o.child_frame_id = robot+'/odom', robot+'/base_footprint'
                o.pose.pose.position.x = self.pose[robot]; o.pose.pose.orientation.w = 1.
                if not self.freeze_encoder: self.pubs[robot, 'encoder'].publish(o)
                j = JointState(name=['Joint_'+str(i) for i in range(1, 6)], position=[0.]*5)
                j.header.stamp = Time(seconds=self.sim).to_msg(); self.pubs[robot, 'joints'].publish(j)
                self.pubs[robot, 'gate'].publish(String(data=json.dumps({'robot': robot, 'ready': True, 'fault': '', 'sequence': self.seq,
                    'armed': self.permit, 'armed_sim_s': 0.})))
                self.pubs[robot, 'grasp'].publish(String(data='attached'))
            until = time.monotonic()+.01
            while time.monotonic() < until: self.executor.spin_once(timeout_sec=.0001)

    def start(self):
        self.pump(.3)
        for end in self.ends.values():
            response = end.prepare(Trigger.Request(), Trigger.Response())
            assert response.success, response.message
        self.permit = True; self.pump(max(1.9, 3.8*.01/self.sim_step))
        assert all(e.state == 'READY' for e in self.ends.values()), [(e.state, e.reason) for e in self.ends.values()]

    def goal(self, distance=.0001):
        msg = Vector3Stamped()
        msg.header.stamp, msg.header.frame_id = Time(seconds=self.sim).to_msg(), 'leader/base_footprint'
        msg.vector.x = distance; self.goal_pub.publish(msg); self.pump(.1)

    def close(self):
        for node in self.nodes:
            self.executor.remove_node(node); node.destroy_node()
        self.executor.shutdown(); rclpy.shutdown()


@pytest.fixture
def rig(tmp_path):
    instance = Rig(tmp_path)
    yield instance
    instance.close()


def test_leader_goal_moves_only_leader_then_returns_ready(rig):
    rig.start(); rig.goal()
    assert rig.ends['leader'].state == 'TRANSPORT'
    rig.pump(1.)
    assert rig.ends['leader'].completed == 1 and rig.ends['leader'].state == 'READY'
    assert abs(rig.pose['leader']-.0001) <= .000002
    assert rig.pose['follower'] == 0. and rig.output['follower'] == 0.
    assert rig.ends['follower'].accepted == 0
    history = np.array(rig.command_history)
    dt, dv = np.diff(history[:, 0]), np.diff(history[:, 1])
    assert np.all(dt >= 0) and np.all(np.abs(dv[dt == 0]) <= 1e-12)
    assert np.max(np.abs(dv[dt > 0]/dt[dt > 0])) <= .001+1e-9


def test_missing_encoder_stops_latches_and_discards_goal(rig):
    rig.start(); rig.goal(); rig.freeze_encoder = True; rig.pump(.25)
    assert all(e.state == 'SAFE_STOP' and e.goal_data is None for e in rig.ends.values())
    assert rig.stop_calls > 0
    rig.freeze_encoder = False; rig.permit = True; rig.pump(.2)
    assert all(e.state == 'SAFE_STOP' for e in rig.ends.values())
    rig.goal(); assert rig.ends['leader'].accepted == 1


@pytest.mark.parametrize('bad', ['frame', 'stale', 'nonfinite', 'range', 'lateral'])
def test_invalid_goal_rejected_without_motion(rig, bad):
    rig.start()
    msg = Vector3Stamped(); msg.header.stamp = Time(seconds=rig.sim).to_msg()
    msg.header.frame_id, msg.vector.x = 'leader/base_footprint', .0001
    if bad == 'frame': msg.header.frame_id = 'world'
    if bad == 'stale': msg.header.stamp = Time(seconds=rig.sim-1.).to_msg()
    if bad == 'nonfinite': msg.vector.x = float('nan')
    if bad == 'range': msg.vector.x = .01
    if bad == 'lateral': msg.vector.y = .001
    rig.goal_pub.publish(msg); rig.pump(.1)
    assert rig.ends['leader'].accepted == 0 and rig.ends['leader'].rejected == 1
    assert rig.output == {'leader': 0., 'follower': 0.}


def test_follower_ignores_leader_goals_and_tf(rig):
    from tf2_msgs.msg import TFMessage
    rig.start()
    pubs = [rig.driver.create_publisher(TFMessage, t, 10) for t in ['/leader/tf', '/tf', '/follower/tf']]
    follower_goal = rig.driver.create_publisher(Vector3Stamped, '/follower/transport_goal', 10)
    for pub in pubs: pub.publish(TFMessage())
    follower_goal.publish(Vector3Stamped()); rig.goal(); rig.pump(.2)
    subs = {s.topic_name for s in rig.ends['follower'].subscriptions}
    assert not any(s.startswith('/leader/') or 'goal' in s or s.endswith('/tf') for s in subs)
    assert rig.output['follower'] == 0. and rig.ends['follower'].state == 'READY'


@pytest.mark.parametrize('remap', [('/follower/mecanum_drive_controller/odometry', '/leader/odom'),
    ('/follower/joint_states', '/world/ground_truth'), ('/follower/base_cmd', '/leader/base_cmd')])
def test_forbidden_remaps_fail_at_construction(rig, remap):
    with pytest.raises(ValueError, match='boundary'):
        TransportEndpoint(leader=False, namespace='/follower', cli_args=['--ros-args', '-r', ':='.join(remap)],
            parameter_overrides=[Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(rig.file))])


@pytest.mark.parametrize('replace', [False, True])
def test_wrong_publisher_on_same_local_topic_cannot_prepare(rig, replace):
    rig.pump(.3)
    if replace:
        node = next(n for n in rig.nodes if n.get_name() == 'mecanum_drive_controller' and n.get_namespace() == '/follower')
        node.destroy_publisher(rig.pubs['follower', 'encoder'])
        del rig.pubs['follower', 'encoder']
        # Keep the local stream valid while its source identity is wrong.
        rig.pubs['follower', 'encoder'] = rig.driver.create_publisher(Odometry, '/follower/mecanum_drive_controller/odometry', 10)
    else:
        rig.driver.create_publisher(Odometry, '/follower/mecanum_drive_controller/odometry', 10)
    rig.pump(.1)
    response = rig.ends['follower'].prepare(Trigger.Request(), Trigger.Response())
    assert not response.success and 'publisher' in response.message


def test_explicit_reset_prepare_arm_cannot_replay_goal(rig):
    rig.start(); rig.goal()
    rig.permit = False; rig.pump(.1)
    assert rig.ends['leader'].state == 'SAFE_STOP'
    for end in rig.ends.values():
        assert end.reset(Trigger.Request(), Trigger.Response()).success
        assert end.prepare(Trigger.Request(), Trigger.Response()).success
    rig.permit = True; rig.pump(1.9)
    assert rig.ends['leader'].state == 'READY' and rig.ends['leader'].goal_data is None
    assert rig.ends['leader'].completed == 0
    assert rig.output == {'leader': 0., 'follower': 0.}


def test_busy_goal_is_rejected_without_replacing_active_reference(rig):
    rig.start(); rig.goal(.0004)
    before = rig.ends['leader'].goal_data
    rig.goal(.0001)
    assert rig.ends['leader'].accepted == 1 and rig.ends['leader'].rejected == 1
    assert rig.ends['leader'].goal_data == before
