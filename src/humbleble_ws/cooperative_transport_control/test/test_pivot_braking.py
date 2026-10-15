"""A pivot must brake before completion, including under irregular ROS timing."""

import math

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time

from cooperative_transport_control.pivot_rotation_controller import PivotRotationController


class CapturePublisher:
    def publish(self, message):
        self.message = message


@pytest.mark.parametrize('angle_deg', [90.0, -90.0, 10.0, -10.0])
def test_pivot_stops_without_an_angular_velocity_step(angle_deg):
    rclpy.init(args=[])
    node = PivotRotationController()
    try:
        node.set_parameters([Parameter('use_sim_time', value=True)])
        node._target = math.radians(angle_deg)
        node._started = True
        node._grasped = {'amir1': True, 'amir2': True}
        node._initial_pivot = (-1.0, 0.0, 0.0)
        node._initial_vector_angle = 0.0
        node._last_tick_sim = 10.0
        node._command_publishers = {
            'amir1': CapturePublisher(), 'amir2': CapturePublisher()}
        rotation, previous, now = 0.0, 0.0, 10.0
        for tick in range(10000):
            dt = (0.012, 0.028, 0.020)[tick % 3]
            now += dt
            node.get_clock().set_ros_time_override(Time(seconds=now))
            node._poses = {
                'amir1': (-1.0, 0.0, rotation),
                'amir2': (-1.0 + 2.0 * math.cos(rotation),
                          2.0 * math.sin(rotation), math.pi + rotation)}
            node._tick()
            omega = node._command_publishers['amir1'].message.angular.z
            assert abs(omega - previous) <= node._alpha * dt + 1.0e-9
            assert abs(omega) <= node._max_omega + 1.0e-9
            rotation += omega * dt
            previous = omega
            if node._finished:
                break
        assert node._finished
        assert abs(node._target - rotation) <= node._angle_tolerance
        assert previous == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
