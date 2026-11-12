"""Exercise trajectory freshness, endpoint holding and simulation-time ramps."""

import math

import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.time import Time

from cooperative_transport_control.coordinator import CooperativeCoordinator
from cooperative_transport_control.kinematics import Pose2


@pytest.fixture
def coordinator():
    rclpy.init(args=[])
    node = CooperativeCoordinator()
    node.set_parameters([
        Parameter('use_sim_time', value=True),
        Parameter('enable_translation_feedforward', value=True)])
    node.get_clock().set_ros_time_override(Time(seconds=10.0))
    yield node
    node.destroy_node()
    rclpy.shutdown()


def velocity(stamp=10.0, frame='world', x=0.10):
    message = TwistStamped()
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.header.frame_id = frame
    message.twist.linear.x = x
    return message


def test_reference_expires_in_simulation_time(coordinator):
    coordinator._on_goal_velocity(velocity())
    assert coordinator._translation_feedforward() == (0.10, 0.0)
    coordinator.get_clock().set_ros_time_override(Time(seconds=10.16))
    assert coordinator._translation_feedforward() == (0.0, 0.0)


@pytest.mark.parametrize('message', [
    velocity(frame='base_footprint'), velocity(x=math.nan),
    velocity(x=math.inf), velocity(stamp=11.0)])
def test_invalid_reference_cannot_keep_a_previous_velocity(coordinator, message):
    coordinator._on_goal_velocity(velocity())
    coordinator._on_goal_velocity(message)
    assert coordinator._translation_feedforward() == (0.0, 0.0)


def test_moving_goal_does_not_trigger_endpoint_stop_and_ramp_uses_sim_time(
        coordinator, monkeypatch):
    monkeypatch.setattr(
        'cooperative_transport_control.coordinator.time.monotonic', lambda: 1000.0)
    coordinator._poses = {
        'amir1': Pose2(-1.0, 0.0, 0.0),
        'amir2': Pose2(1.0, 0.0, math.pi)}
    coordinator._odom_times = {'amir1': 1000.0, 'amir2': 1000.0}
    coordinator._attached = {'amir1': True, 'amir2': True}
    coordinator._enabled = True
    coordinator._goal = Pose2(0.0, 0.0, 0.0)
    coordinator._last_tick = 10.0
    coordinator._on_goal_velocity(velocity())
    coordinator.get_clock().set_ros_time_override(Time(seconds=10.02))
    coordinator._control_tick()
    # 0.1 m/s2 * 0.02 simulated seconds, independent of the wall clock.
    assert coordinator._last_commands['amir1'].x == pytest.approx(0.002)
    assert coordinator._last_commands['amir2'].x == pytest.approx(-0.002)
    coordinator._on_goal_velocity(velocity(stamp=10.02, x=0.0))
    coordinator._control_tick()
    assert coordinator._last_commands['amir1'].x == 0.0


def test_rotating_reference_prevents_false_holding_and_expires(coordinator, monkeypatch):
    monkeypatch.setattr(
        'cooperative_transport_control.coordinator.time.monotonic', lambda: 1000.0)
    coordinator.set_parameters([Parameter('enable_rotation_feedforward', value=True)])
    coordinator._poses = {
        'amir1': Pose2(-1.0, 0.0, 0.0),
        'amir2': Pose2(1.0, 0.0, math.pi)}
    coordinator._odom_times = {'amir1': 1000.0, 'amir2': 1000.0}
    coordinator._attached = {'amir1': True, 'amir2': True}
    coordinator._enabled = True
    coordinator._goal = Pose2(0.0, 0.0, 0.0)
    coordinator._last_tick = 10.0
    message = velocity(x=0.0)
    message.twist.angular.z = math.radians(1.0)
    coordinator._on_goal_velocity(message)
    coordinator.get_clock().set_ros_time_override(Time(seconds=10.02))
    coordinator._control_tick()
    assert coordinator._last_commands['amir1'].yaw > 0.0
    assert coordinator._last_commands['amir1'].y < 0.0
    coordinator.get_clock().set_ros_time_override(Time(seconds=10.16))
    assert coordinator._rotation_feedforward() == 0.0


def test_invalid_angular_reference_clears_linear_and_angular_motion(coordinator):
    coordinator.set_parameters([Parameter('enable_rotation_feedforward', value=True)])
    message = velocity()
    message.twist.angular.z = math.nan
    coordinator._on_goal_velocity(message)
    assert coordinator._translation_feedforward() == (0.0, 0.0)
    assert coordinator._rotation_feedforward() == 0.0
