"""Supervisor settling must use continuous fresh measurements, not elapsed wait."""
import pytest
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import JointState

from amir_follower_control.experiment_support import StationaryWindow,ExperimentGrasps


def samples(at, qdot=0.):
    joints = JointState(name=[f'Joint_{i}' for i in range(1, 6)], velocity=[qdot]*5)
    odom = Odometry()
    for message in (joints, odom):
        message.header.stamp = Time(seconds=at).to_msg()
    return joints, odom


def test_stale_gap_requires_a_new_continuous_stationary_window():
    window = StationaryWindow()
    assert not window.ready(1., *samples(1.))
    assert not window.ready(1.2, *samples(1.))
    assert not window.ready(1.4, *samples(1.4))
    assert window.ready(1.71, *samples(1.71))
    assert not window.ready(1., *samples(1.))  # clock reset


@pytest.mark.parametrize('velocity', [.012, float('nan')])
def test_motion_or_invalid_velocity_cannot_be_bias_readiness(velocity):
    window = StationaryWindow()
    assert not window.ready(1., *samples(1.))
    assert not window.ready(1.4, *samples(1.4, velocity))
    assert not window.ready(1.5, *samples(1.5))
    assert window.ready(1.81, *samples(1.81))


def test_experiment_cannot_start_with_only_follower_grasp_or_stale_leader():
    state=ExperimentGrasps()
    state.update('amir2','attached',1.)
    assert not state.ready(1.)
    state.update('amir1','attached',.6)
    assert not state.ready(1.)
    state.update('amir1','attached',1.)
    assert state.ready(1.1)
    state.update('amir1','detached',1.1)
    assert not state.ready(1.1)


def test_experiment_clock_reset_cannot_reuse_future_grasp_heartbeat():
    state=ExperimentGrasps()
    for robot in ('amir1','amir2'):state.update(robot,'attached',10.)
    assert not state.ready(1.)
