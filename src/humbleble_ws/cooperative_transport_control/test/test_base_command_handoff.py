"""The approach controller must stop publishing once transport takes over."""

import pytest
import rclpy
from std_msgs.msg import Bool

from cooperative_transport_control.dual_base_approach import DualBaseApproach


@pytest.fixture
def approach():
    rclpy.init(args=[])
    node = DualBaseApproach()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_approach_holds_during_grasp_then_relinquishes_commands(approach, monkeypatch):
    zeros = []
    monkeypatch.setattr(approach, '_publish_zero', lambda: zeros.append(True))
    approach._phase = 'HOLDING_ZERO'
    approach._tick()
    assert zeros == [True]
    approach._on_transport_active(Bool(data=False))
    assert not approach._timer.is_canceled()
    approach._on_transport_active(Bool(data=True))
    assert approach._phase == 'HANDED_OFF'
    assert approach._timer.is_canceled()
    approach._tick()  # A previously queued timer callback must also stay silent.
    assert zeros == [True]


def test_transport_status_cannot_interrupt_the_initial_approach(approach):
    approach._phase = 'MOVING'
    approach._on_transport_active(Bool(data=True))
    assert approach._phase == 'MOVING'
    assert not approach._timer.is_canceled()
