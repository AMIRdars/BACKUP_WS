import time
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.admittance import PlanarAdmittance, solve_xyz
from amir_compliance_control.arm_admittance import ArmAdmittance
from test_leader_only import Rig


def model_xml():
    root = ET.Element('robot'); parent = 'base_footprint'
    control = ET.SubElement(root, 'ros2_control')
    for i, axis in enumerate(['0 0 1', '0 1 0', '1 0 0', '0 1 0', '0 0 1'], 1):
        child = 'gripper_base_1' if i == 5 else 'link'+str(i)
        joint = ET.SubElement(root, 'joint', name='Joint_'+str(i), type='revolute')
        ET.SubElement(joint, 'parent', link=parent); ET.SubElement(joint, 'child', link=child)
        offsets = ['.1 .025 .04', '.04 .08 .02', '.07 -.03 .06', '.08 .02 -.01', '.02 .01 .05']
        ET.SubElement(joint, 'origin', xyz=offsets[i-1]); ET.SubElement(joint, 'axis', xyz=axis)
        ET.SubElement(joint, 'limit', lower='-2', upper='2', velocity='1', effort='4')
        entry = ET.SubElement(ET.SubElement(control, 'joint', name='Joint_'+str(i)), 'state_interface', name='position')
        ET.SubElement(entry, 'param', name='initial_value').text = '0'
        parent = child
    joint = ET.SubElement(root, 'joint', name='tcp', type='fixed')
    ET.SubElement(joint, 'parent', link=parent); ET.SubElement(joint, 'child', link='tcp_link')
    ET.SubElement(joint, 'origin', xyz='.08 .02 .03')
    return ET.tostring(root, encoding='unicode')


def core():
    return PlanarAdmittance([1., 1.], [40., 40.], [400., 400.], [.0005]*2, [.001]*2, [.02]*2)


def test_analytic_geometric_jacobian_against_finite_differences():
    model = LocalModel(model_xml()); rng = np.random.default_rng(5)
    for q in rng.uniform(-.4, .4, (20, 5)):
        j = model.jacobian(q); numeric = np.zeros((6, 5)); h = 1e-6
        _, sensor = model.frames(q)
        for i in range(5):
            delta = np.eye(5)[i]*h
            p1, s1 = model.frames(q+delta); p0, s0 = model.frames(q-delta)
            numeric[:3, i] = (p1-p0)/(2*h)
            w = ((s1[:3, :3]-s0[:3, :3])/(2*h))@sensor[:3, :3].T
            numeric[3:, i] = [w[2, 1], w[0, 2], w[1, 0]]
        np.testing.assert_allclose(j, numeric, atol=1e-9)


def test_independent_second_order_steady_response_and_zero_initial_step():
    a = core(); np.testing.assert_array_equal(a.x, [0., 0.])
    for _ in range(200): a.step([.1, -.08], .01)
    np.testing.assert_allclose(a.x, [.00025, -.0002], atol=1e-8)
    assert np.max(np.abs(a.v)) < 1e-8


def test_saturation_then_release_has_no_hidden_windup():
    a = core()
    for _ in range(500): a.step([.8, -.8], .01)
    np.testing.assert_allclose(a.x, [.0005, -.0005], atol=1e-10)
    assert a.saturated.all() and not a.v.any()
    for _ in range(300):
        a.step([0., 0.], .01)
        assert np.max(np.abs(a.x)) <= .0005
        assert np.max(np.abs(a.v)) <= .001
    assert np.max(np.abs(a.x)) < 1e-8


@pytest.mark.parametrize('dt', [0., -.01, .051, float('nan')])
def test_discontinuous_timestep_is_rejected(dt):
    with pytest.raises(ValueError): core().step([0., 0.], dt)


def test_dls_tracks_small_xy_without_height_drift_and_respects_margin():
    m = LocalModel(model_xml()); q = np.zeros(5); p, _ = m.frames(q)
    target = p+np.array([.0004, -.0003, 0.])
    result = solve_xyz(m, q, target)
    np.testing.assert_allclose(m.frames(result)[0], target, atol=1e-7)
    assert np.all(result > m.lower+.01) and np.all(result < m.upper-.01)
    with pytest.raises(ValueError): solve_xyz(m, m.upper-.001, target)
    with pytest.raises(ValueError): solve_xyz(m, q, p+[10., 0., 0.])


def test_rank_deficient_pose_is_rejected_instead_of_assuming_full_task_rank():
    root = ET.fromstring(model_xml())
    for j in root.findall('joint'):
        j.find('origin').set('xyz', '.1 0 0')
    model = LocalModel(ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError, match='rank'):
        solve_xyz(model, np.zeros(5), model.frames(np.zeros(5))[0])


class ArmRig(Rig):
    def __init__(self, path):
        super().__init__(path)
        self.sim_step = .01  # Real-time clock; do not double watchdog time under CPU load.
        old = self.ends['follower']; self.executor.remove_node(old); self.nodes.remove(old); old.destroy_node()
        self.file.write_text(model_xml())
        end = ArmAdmittance(namespace='/follower', parameter_overrides=[
            Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(self.file))])
        self.ends['follower'] = end; self.nodes.append(end); self.executor.add_node(end)
        pre = rclpy.create_node('wrench_preprocessor', namespace='/follower')
        self.nodes.append(pre); self.executor.add_node(pre)
        self.filtered_pub = pre.create_publisher(WrenchStamped, '/follower/wrench_filtered', 10)
        self.status_pub = pre.create_publisher(String, '/follower/wrench_status', 10)
        self.force, self.dropout, self.filter_value, self.last_sent = [.1, 0.], False, 'READY', None
        self.driver.create_timer(.005, self.send_wrench)

    def send_wrench(self):
        self.status_pub.publish(String(data=self.filter_value))
        if not self.dropout and self.sim != self.last_sent:
            msg = WrenchStamped(); msg.header.stamp = Time(seconds=self.sim).to_msg()
            msg.header.frame_id = 'follower/grasp_aligned'
            msg.wrench.force.x, msg.wrench.force.y = self.force
            self.filtered_pub.publish(msg); self.last_sent = self.sim


@pytest.fixture
def rig(tmp_path):
    instance = ArmRig(tmp_path)
    yield instance
    instance.close()


def test_arm_only_uses_measured_origin_and_base_stays_zero(rig):
    rig.start(); end = rig.ends['follower']
    assert end.enable(Trigger.Request(), Trigger.Response()).success
    np.testing.assert_allclose(end.reference, end.model.frames(np.zeros(5))[0])
    assert not end.core.x.any()
    rig.pump(.4)
    assert end.core is not None and end.core.x[0] > 0 and end.core.x[1] == 0.
    assert rig.output['follower'] == 0. and rig.output['leader'] == 0.
    assert not any('leader' in s.topic_name or 'goal' in s.topic_name or s.topic_name.endswith('/tf') for s in end.subscriptions)


@pytest.mark.parametrize('fault', ['dropout', 'invalid_bias', 'nonfinite'])
def test_filtered_sensor_fault_stops_both_and_requires_new_enable(rig, fault):
    rig.start(); end = rig.ends['follower']
    assert end.enable(Trigger.Request(), Trigger.Response()).success
    rig.pump(.1)
    if fault == 'dropout': rig.dropout = True
    if fault == 'invalid_bias': rig.filter_value = 'UNTARED'
    if fault == 'nonfinite': rig.force = [float('nan'), 0.]
    rig.pump(.3)
    assert end.core is None and end.state == 'SAFE_STOP' and rig.stop_calls > 0
    rig.dropout, rig.filter_value, rig.force = False, 'READY', [0., 0.]
    rig.pump(.15)
    assert end.state == 'SAFE_STOP' and end.core is None


def test_filtered_wrench_cannot_be_remapped_to_leader_or_truth(rig):
    with pytest.raises(ValueError, match='boundary'):
        ArmAdmittance(namespace='/follower',
            cli_args=['--ros-args', '-r', '/follower/wrench_filtered:=/leader/ground_truth'],
            parameter_overrides=[Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(rig.file))])
