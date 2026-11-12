"""Actual five-joint feasibility, local yaw compensation, bounds and ROS stop."""
from pathlib import Path

import numpy as np
import pytest
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

from amir_compliance_control.local_yaw import ArmSoftTask, YawRecentering
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.yaw_arm_base import YawArmBase
from test_admittance import ArmRig


ROOT = Path(__file__).resolve().parents[4]
URDF = ROOT/'audits/object_center_rotation_20261103/cp8r1/generated/follower.urdf'
HOME = np.array([0., 1.4, -1.6, .2, 1.570796])


class LocalYawRig(ArmRig):
    def __init__(self, path):
        super().__init__(path)
        pre = next(n for n in self.nodes if n.get_name() == 'wrench_preprocessor')
        self.raw_pub = pre.create_publisher(WrenchStamped, '/follower/wrench_at_grasp_raw', 10)
        self.raw_dropout = False
        old = self.ends['follower']
        self.executor.remove_node(old)
        self.nodes.remove(old)
        old.destroy_node()
        node = YawArmBase(namespace='/follower', parameter_overrides=[Parameter('use_sim_time', value=True),
            Parameter('urdf_file', value=str(self.file)), Parameter('allow_ungrasped_test', value=True),
            Parameter('wrench_force_limit_N', value=.5), Parameter('wrench_torque_limit_Nm', value=.1)])
        self.ends['follower'] = node
        self.nodes.append(node)
        self.executor.add_node(node)

    def send_wrench(self):
        if not self.dropout and not self.raw_dropout and self.sim != self.last_sent:
            msg = WrenchStamped()
            msg.header.stamp = Time(seconds=self.sim).to_msg()
            msg.header.frame_id = 'follower/grasp_aligned'
            msg.wrench.force.x, msg.wrench.force.y = self.force
            self.raw_pub.publish(msg)
        super().send_wrench()


@pytest.mark.parametrize('axis', [1, 5])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_five_joint_arm_moves_in_local_force_yaw_direction_and_exposes_unavailable_task(axis, sign):
    m = LocalModel(URDF.read_text())
    arm = ArmSoftTask(m, metric=[1., 1., 1., .1, .1, .1], speed=.03, acceleration=.15, margin=.01)
    target = np.zeros(6)
    target[axis] = sign*.0001
    u = arm.step(HOME, HOME, target, .01)
    assert sign*(m.jacobian(HOME)@u)[axis] > 0
    assert np.linalg.matrix_rank(m.jacobian(HOME)) == 5
    assert np.linalg.norm(arm.last['residual_twist']) > 1e-6
    assert np.all(abs(u) <= .15*.01+1e-14)


def test_near_joint_limit_and_bad_timestep_fail_instead_of_clipping_pose():
    m = LocalModel(URDF.read_text())
    arm = ArmSoftTask(m, metric=[1.]*6, speed=.03, acceleration=.15, margin=.01)
    q = HOME.copy()
    q[0] = m.upper[0]-.005
    with pytest.raises(ValueError, match='margin'):
        arm.step(q, q, np.zeros(6), .01)
    for dt in [0., -.01, .06, np.nan]:
        with pytest.raises(ValueError):
            arm.step(HOME, HOME, np.zeros(6), dt)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_compensation_uses_limited_yaw_and_cancels_grasp_point_velocity(sign):
    base = YawRecentering(cutoff=10000., gains=[0., 0., 10.], velocity=[1., 1., .1], acceleration=[100., 100., 100.])
    p = np.array([.5, .1, .6])
    command = base.step([0., 0., sign*.1], p, [0., 0., 0.], .01)
    assert command[2] == sign*.1
    np.testing.assert_allclose(np.r_[command[:2], 0.]+np.cross([0., 0., command[2]], p), 0., atol=1e-15)


def test_default_has_no_force_to_base_path_and_optional_memory_is_slow_bounded():
    base = YawRecentering(cutoff=.2, gains=[.5, .5, 1.], velocity=[.002]*3, acceleration=[.001]*3)
    np.testing.assert_array_equal(base.step([0.]*3, [.5, 0., .6], [1.]*3, .01), np.zeros(3))
    base = YawRecentering(cutoff=.2, gains=[.5, .5, 1.], velocity=[.002]*3,
                         acceleration=[.001]*3, force_gains=[.005, .005, 0.], force_cutoff=.1)
    first = base.step([0.]*3, [.5, 0., .6], [1., -1., 0.], .01)
    assert np.max(abs(first)) < 1e-6
    previous = first
    for force in ([1., -1., 0.], [-1., 1., 0.]):
        for _ in range(1000):
            command = base.step([0.]*3, [.5, 0., .6], force, .01)
            assert np.all(abs(command-previous) <= .001*.01+1e-14)
            assert np.all(abs(command) <= .002+1e-14)
            assert np.all(abs(base.memory) <= .002+1e-14)
            previous = command


def test_local_ros_wrench_and_yaw_only_boundary_then_watchdog_discards_all_state(tmp_path):
    rig = LocalYawRig(tmp_path)
    try:
        node = rig.ends['follower']
        rig.start()
        assert node.enable(Trigger.Request(), Trigger.Response()).success
        rig.pump(.2)
        assert node.core is not None and node.arm_task is not None
        inputs = {s.topic_name for s in node.subscriptions}
        assert not any(t.startswith(('/leader/', '/experiment/', '/world/')) or '/tf' in t or 'goal' in t or 'rotation_delta' in t for t in inputs)
        assert sum(p.topic_name == '/follower/base_cmd' for p in node.publishers) == 1
        rig.dropout = True
        rig.pump(.3)
        assert node.latched and node.core is None and node.base is None and node.yaw_core is None
        assert not node.base_request(0.).any()
        rig.dropout = False
        rig.pump(.1)
        assert node.latched and node.core is None
    finally:
        rig.close()


@pytest.mark.parametrize('fault', ['raw_force', 'raw_torque', 'raw_frame', 'raw_dropout', 'filtered_torque'])
def test_independent_tcp_wrench_faults_stop_both_without_replaying(tmp_path, fault):
    rig = LocalYawRig(tmp_path)
    try:
        rig.start()
        node = rig.ends['follower']
        assert node.enable(Trigger.Request(), Trigger.Response()).success
        rig.pump(.1)
        msg = WrenchStamped()
        msg.header.stamp = Time(seconds=rig.sim+.001).to_msg()
        msg.header.frame_id = 'follower/grasp_aligned'
        if fault == 'raw_dropout':
            rig.raw_dropout = True
        elif fault == 'filtered_torque':
            msg.wrench.torque.z = .101
            node.filtered(msg)
        else:
            if fault == 'raw_force': msg.wrench.force.z = .501
            if fault == 'raw_torque': msg.wrench.torque.x = .101
            if fault == 'raw_frame': msg.header.frame_id = 'leader/grasp_aligned'
            node.raw(msg)
        rig.pump(.3)
        assert node.latched and node.core is None and node.raw_bias is None
        assert rig.stop_calls > 0 and rig.ends['leader'].latched
        assert rig.output['leader'] == rig.output['follower'] == 0.
        rig.raw_dropout = False
        rig.pump(.1)
        assert node.latched and node.core is None
    finally:
        rig.close()


def test_slow_force_pi_settles_an_independent_unavailable_direction_spring():
    # Independent physical oracle: an otherwise unobserved spring, target
    # ramp then hold. No FK deflection is available in this direction.
    base = YawRecentering(cutoff=.2, gains=[0.]*3, velocity=[.002]*3, acceleration=[.001]*3,
        force_gains=[0., .0005, 0.], force_velocity_gains=[0., .005, 0.], force_cutoff=.1)
    position = 0.
    for i in range(4000):
        target = min(.001, i*.01*.0001)
        force = 1000.*(target-position)
        command = base.step([0.]*3, [.5, 0., .6], [0., force, 0.], .01)
        position += command[1]*.01
    assert abs(target-position) < 1e-6
    assert abs(command[1]) < 1e-6
