from geometry_msgs.msg import Vector3
from ros_gz_interfaces.msg import Contact, JointWrench

from cooperative_transport_control.friction_grasp_manager import (
    contact_normal_force,
)


def _contact(first: str, second: str, force_x: float) -> Contact:
    message = Contact()
    message.collision1.name = first
    message.collision2.name = second
    message.normals = [Vector3(x=1.0, y=0.0, z=0.0)]
    wrench = JointWrench()
    wrench.body_1_wrench.force.x = force_x
    wrench.body_2_wrench.force.x = -force_x
    message.wrenches = [wrench]
    return message


def test_contact_normal_force_accepts_matching_finger_and_payload():
    message = _contact(
        'amir1::finger_left_1::finger_left_1_collision',
        'cooperative_payload::payload_link::payload_collision', 3.5)
    assert contact_normal_force(message, 'amir1', 'left') == 3.5


def test_contact_normal_force_handles_reversed_collision_order():
    message = _contact(
        'cooperative_payload::payload_link::payload_collision',
        'amir2::finger_right_1::finger_right_1_collision', 2.25)
    assert contact_normal_force(message, 'amir2', 'right') == 2.25


def test_contact_normal_force_rejects_non_payload_contact():
    message = _contact(
        'amir1::finger_left_1::finger_left_1_collision',
        'ground_plane::link::collision', 8.0)
    assert contact_normal_force(message, 'amir1', 'left') == 0.0


def test_contact_normal_force_rejects_wrong_finger():
    message = _contact(
        'amir1::finger_right_1::finger_right_1_collision',
        'cooperative_payload::payload_link::payload_collision', 8.0)
    assert contact_normal_force(message, 'amir1', 'left') == 0.0
