"""Coordinate the two Ignition DetachableJoint instances."""

from functools import partial
from typing import Dict, Optional, Set

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool, Trigger


class AttachManager(Node):
    """Issue attach/detach commands in a deterministic, safe order."""

    def __init__(self) -> None:
        super().__init__('attach_manager')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('second_command_delay', 0.5)
        self.declare_parameter('auto_attach', True)
        self.declare_parameter('auto_attach_delay', 1.0)
        self.declare_parameter('close_grippers_before_attach', False)
        self.declare_parameter('gripper_close_position', 0.20)
        self.declare_parameter('gripper_maximum_effort', 0.8)
        self.declare_parameter(
            'require_gripper_contacts_before_attach', False)
        self.declare_parameter(
            'payload_collision_name',
            'cooperative_payload::payload_link::payload_collision')

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._states: Dict[str, Optional[bool]] = {name: None for name in self._names}
        self._attach_publishers = {}
        self._detach_publishers = {}
        self._state_publishers = {}
        self._gripper_clients = {}
        for name in self._names:
            self._attach_publishers[name] = self.create_publisher(
                Empty, f'/{name}/grasp/attach', 10)
            self._detach_publishers[name] = self.create_publisher(
                Empty, f'/{name}/grasp/detach', 10)
            self._state_publishers[name] = self.create_publisher(
                Bool, f'/cooperative_transport/{name}/grasp_state', 10)
            self._gripper_clients[name] = ActionClient(
                self, GripperCommand,
                f'/{name}/gripper_controller/gripper_cmd')
            self.create_subscription(
                String, f'/{name}/grasp/state',
                lambda message, robot=name: self._on_state(robot, message), 10)
            for side in ('left', 'right'):
                self.create_subscription(
                    Contacts, f'/{name}/finger_{side}_contact',
                    lambda message, robot=name, finger=side:
                    self._on_finger_contact(robot, finger, message), 10)

        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/grasp_status', 10)
        self.create_service(
            SetBool, '/cooperative_transport/attach_all', self._on_attach_all)
        self.create_service(
            Trigger, '/cooperative_transport/grasp_status', self._on_status)

        self._one_shot_timers = []
        self._gripper_close_pending = False
        self._gripper_close_complete = False
        self._closed_grippers: Set[str] = set()
        self._payload_contacts = set()
        self._gripper_retry_scheduled = False
        # Republish logical state so coordinators started in the same launch do
        # not miss the one-shot auto-attach command during DDS discovery.
        self.create_timer(0.2, self._publish_all_states)
        if bool(self.get_parameter('auto_attach').value):
            self._schedule_once(
                float(self.get_parameter('auto_attach_delay').value),
                partial(self._command_pair, True))
        self.get_logger().info(
            'Grasp manager ready; service /cooperative_transport/attach_all')

    def _on_state(self, robot: str, message: String) -> None:
        state = message.data.strip().lower()
        if state not in ('attached', 'detached'):
            self.get_logger().warning(
                f'Ignoring unknown {robot} grasp state: {message.data!r}')
            return
        self._states[robot] = state == 'attached'
        self._publish_state(robot)
        self._publish_status()

    def _on_finger_contact(
            self, robot: str, side: str, message: Contacts) -> None:
        payload_collision = str(
            self.get_parameter('payload_collision_name').value)
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if payload_collision in names:
                self._payload_contacts.add((robot, side))
                self._try_finish_gripper_close()
                return

    def _on_attach_all(self, request: SetBool.Request,
                       response: SetBool.Response) -> SetBool.Response:
        self._command_pair(request.data)
        response.success = True
        operation = 'attach' if request.data else 'detach'
        response.message = f'{operation} sequence sent; inspect grasp_status for confirmation.'
        return response

    def _command_pair(self, attach: bool) -> None:
        if (attach
                and bool(self.get_parameter(
                    'close_grippers_before_attach').value)
                and not self._gripper_close_complete):
            self._close_grippers_then_attach()
            return
        self._publish_command_pair(attach)

    def _publish_command_pair(self, attach: bool) -> None:
        # Attach leader first, follower second. Detach in reverse order.
        order = self._names if attach else tuple(reversed(self._names))
        self._publish_command(order[0], attach)
        self._schedule_once(
            float(self.get_parameter('second_command_delay').value),
            partial(self._publish_command, order[1], attach))

    def _close_grippers_then_attach(self) -> None:
        if self._gripper_close_pending or self._gripper_close_complete:
            return
        if not all(
                client.server_is_ready()
                for client in self._gripper_clients.values()):
            if not self._gripper_retry_scheduled:
                self._gripper_retry_scheduled = True

                def retry() -> None:
                    self._gripper_retry_scheduled = False
                    self._close_grippers_then_attach()

                self._schedule_once(0.5, retry)
            return

        self._gripper_close_pending = True
        self._closed_grippers.clear()
        position = float(self.get_parameter('gripper_close_position').value)
        effort = float(self.get_parameter('gripper_maximum_effort').value)
        for robot, client in self._gripper_clients.items():
            goal = GripperCommand.Goal()
            goal.command.position = position
            goal.command.max_effort = effort
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda response, name=robot:
                self._on_gripper_goal_response(name, response))
        self.get_logger().info(
            f'Closing both grippers to {position:.3f} rad before attachment')

    def _on_gripper_goal_response(self, robot: str, future) -> None:
        try:
            handle = future.result()
        except Exception as exception:
            self._gripper_close_pending = False
            self.get_logger().error(
                f'{robot} gripper close request failed: {exception}')
            return
        if handle is None or not handle.accepted:
            self._gripper_close_pending = False
            self.get_logger().error(
                f'{robot} gripper controller rejected the close command')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result, name=robot:
            self._on_gripper_close_result(name, result))

    def _on_gripper_close_result(self, robot: str, future) -> None:
        try:
            result = future.result().result
        except Exception as exception:
            self._gripper_close_pending = False
            self.get_logger().error(
                f'{robot} gripper close action failed: {exception}')
            return
        if not (result.reached_goal or result.stalled):
            self._gripper_close_pending = False
            self.get_logger().error(
                f'{robot} gripper did not reach or stall on the payload')
            return
        self._closed_grippers.add(robot)
        self._try_finish_gripper_close()

    def _try_finish_gripper_close(self) -> None:
        if not self._gripper_close_pending:
            return
        require_contacts = bool(self.get_parameter(
            'require_gripper_contacts_before_attach').value)
        if require_contacts:
            expected_contacts = {
                (name, side)
                for name in self._names
                for side in ('left', 'right')
            }
            if not expected_contacts.issubset(self._payload_contacts):
                return
            completion = 'confirmed payload contact on all four fingers'
        else:
            if len(self._closed_grippers) != len(self._names):
                return
            completion = 'completed or stalled'
        self._gripper_close_pending = False
        self._gripper_close_complete = True
        self.get_logger().info(
            f'Both grippers {completion}; attaching both grasp joints')
        self._publish_command_pair(True)

    def _publish_command(self, robot: str, attach: bool) -> None:
        publisher = (
            self._attach_publishers[robot] if attach
            else self._detach_publishers[robot])
        publisher.publish(Empty())
        # Gazebo Fortress creates DetachableJoint in the attached state but does
        # not emit its output_topic until a state transition. Track the command
        # as the logical state; physical feedback overrides it in _on_state.
        self._states[robot] = attach
        self._publish_state(robot)
        self._publish_status()
        self.get_logger().info(f'{"attach" if attach else "detach"}: {robot}')

    def _publish_state(self, robot: str) -> None:
        state = self._states[robot]
        if state is None:
            return
        message = Bool()
        message.data = state
        self._state_publishers[robot].publish(message)

    def _publish_all_states(self) -> None:
        for robot in self._names:
            self._publish_state(robot)

    def _schedule_once(self, delay: float, callback) -> None:
        holder = {}

        def wrapped():
            callback()
            timer = holder['timer']
            timer.cancel()
            self._one_shot_timers.remove(timer)

        holder['timer'] = self.create_timer(max(0.001, delay), wrapped)
        self._one_shot_timers.append(holder['timer'])

    def _status_text(self) -> str:
        values = []
        for name in self._names:
            state = self._states[name]
            values.append(f'{name}={"unknown" if state is None else str(state).lower()}')
        return ', '.join(values)

    def _publish_status(self) -> None:
        message = String()
        message.data = self._status_text()
        self._status_publisher.publish(message)

    def _on_status(self, _request: Trigger.Request,
                   response: Trigger.Response) -> Trigger.Response:
        response.success = all(self._states[name] is not None for name in self._names)
        response.message = self._status_text()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AttachManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
