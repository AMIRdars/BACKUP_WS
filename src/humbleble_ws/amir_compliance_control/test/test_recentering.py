import numpy as np
import pytest
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from amir_compliance_control.recentering import BaseRecentering
from amir_compliance_control.admittance import solve_pose, solve_xyz, rotation_error
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.arm_base import ArmBase
from test_admittance import ArmRig, model_xml
import xml.etree.ElementTree as ET


def base():
    return BaseRecentering(.2, [1.5, 1.5, 1.], [.0005, .0005, .001], [.001, .001, .002])


@pytest.mark.parametrize('axis', [0, 1, 2])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_task_velocity_signs_and_yaw_lever_arm(axis, sign):
    b = base(); e = np.zeros(3); e[axis] = sign*.0002
    grasp = np.array([.475, .01, .65])
    for _ in range(500): command = b.step(e, grasp, .01)
    task = np.array([[1, 0, -grasp[1]], [0, 1, grasp[0]], [0, 0, 1]])@command
    assert sign*task[axis] > 0
    np.testing.assert_allclose(task, b.gains*b.low, atol=1e-12)


def test_slow_branch_rejects_fast_error_and_follows_persistent_error():
    ratios = []
    for freq in [.05, 2.]:
        b = base(); samples = []
        for i in range(10000):
            e = .0001*np.sin(2*np.pi*freq*i*.01)
            b.step([e, 0., 0.], [.475, 0., .65], .01)
            if i >= 2000: samples.append(b.low[0])
        ratios.append(float(np.sqrt(np.mean(np.square(samples)))/(.0001/np.sqrt(2))))
    assert ratios[0] > .9 and ratios[1] < .11 and ratios[1]/ratios[0] < .15


def test_base_speed_and_acceleration_remain_bounded_through_reversal():
    b = base(); previous = np.zeros(3)
    for i in range(1000):
        cmd = b.step([1., 1., 1.] if i < 500 else [-1., -1., -1.], [.475, 0., .65], .01)
        assert np.all(np.abs(cmd) <= b.vmax+1e-12)
        assert np.all(np.abs(cmd-previous) <= b.amax*.01+1e-12)
        previous = cmd


def test_pose_lock_supports_straight_motion_and_refuses_unavailable_lateral_task():
    root = ET.fromstring(model_xml())
    for joint, axis in zip(root.findall("joint[@type='revolute']"), ['0 0 1','0 1 0','0 1 0','0 1 0','1 0 0']):
        joint.find('axis').set('xyz', axis); joint.find('origin').set('xyz', '.1 0 0')
    m = LocalModel(ET.tostring(root, encoding='unicode')); q = np.array([0., .4, -.8, .4, 0.])
    p, s = m.frames(q)
    for dx in [-.0005, .0005]:
        result = solve_pose(m, q, p+[dx,0.,0.], s[:3,:3])
        pp, ss = m.frames(result)
        assert np.linalg.norm(pp-p-[dx,0.,0.]) < 1e-7
        assert np.linalg.norm(rotation_error(s[:3,:3], ss[:3,:3])) < 1e-6
    with pytest.raises(ValueError, match='not reachable'):
        solve_pose(m, q, p+[0.,.0005,0.], s[:3,:3])


class JointOverride:
    def __init__(self, publisher):
        self.publisher, self.positions = publisher, None

    def publish(self, message):
        if self.positions is not None:
            message.position = list(self.positions)
        self.publisher.publish(message)


def make_rig(tmp_path, gains):
    rig = ArmRig(tmp_path)
    rig.sim_step = .01
    old = rig.ends['follower']; rig.executor.remove_node(old); rig.nodes.remove(old); old.destroy_node()
    node = ArmBase(namespace='/follower', parameter_overrides=[Parameter('use_sim_time', value=True),
        Parameter('urdf_file', value=str(rig.file)), Parameter('allow_ungrasped_test', value=True),
        Parameter('preserve_orientation', value=False), Parameter('straight_only', value=False),
        Parameter('base_gains', value=gains)])
    rig.ends['follower'] = node; rig.nodes.append(node); rig.executor.add_node(node)
    rig.override = JointOverride(rig.pubs['follower','joints']); rig.pubs['follower','joints'] = rig.override
    rig.twist = np.zeros(3)
    rig.driver.create_subscription(TwistStamped, '/follower/base_cmd',
        lambda m: setattr(rig, 'twist', np.array([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z])), 10)
    return rig


@pytest.mark.parametrize('axis', [0,1,2])
def test_actual_local_joint_deflection_drives_correct_base_axis_and_fault_latches(tmp_path, axis):
    gains = [0.,0.,0.]; gains[axis] = 1.
    rig = make_rig(tmp_path, gains)
    try:
        rig.start(); node = rig.ends['follower']
        assert node.enable(Trigger.Request(), Trigger.Response()).success
        rig.pump(.2)
        # Force/virtual offset alone cannot command the base if actual arm stayed still.
        assert not rig.twist.any()
        if axis == 2:
            q = np.zeros(5); q[0] = .0002
        else:
            target = node.reference.copy(); target[axis] += .0002
            q = solve_xyz(node.model, np.zeros(5), target)
        rig.override.positions = q
        rig.pump(.4)
        assert rig.twist[axis] > 0, rig.twist
        assert not any(s.topic_name.startswith('/leader/') or 'goal' in s.topic_name or s.topic_name.endswith('/tf') for s in node.subscriptions)
        rig.dropout = True; rig.pump(.3)
        assert node.state == 'SAFE_STOP' and node.base is None and not rig.twist.any()
        rig.dropout, rig.permit = False, True; rig.pump(.15)
        assert node.state == 'SAFE_STOP' and not rig.twist.any()
    finally:
        rig.close()
