"""Coordinate the two Ignition DetachableJoint instances."""

from functools import partial
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
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

        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        self._states: Dict[str, Optional[bool]] = {name: None for name in self._names}
        self._attach_publishers = {}
        self._detach_publishers = {}
        self._state_publishers = {}
        for name in self._names:
            self._attach_publishers[name] = self.create_publisher(
                Empty, f'/{name}/grasp/attach', 10)
            self._detach_publishers[name] = self.create_publisher(
                Empty, f'/{name}/grasp/detach', 10)
            self._state_publishers[name] = self.create_publisher(
                Bool, f'/cooperative_transport/{name}/grasp_state', 10)
            self.create_subscription(
                Bool, f'/{name}/grasp/state',
                lambda message, robot=name: self._on_state(robot, message), 10)

        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/grasp_status', 10)
        self.create_service(
            SetBool, '/cooperative_transport/attach_all', self._on_attach_all)
        self.create_service(
            Trigger, '/cooperative_transport/grasp_status', self._on_status)

        self._one_shot_timers = []
        # Republish logical state so coordinators started in the same launch do
        # not miss the one-shot auto-attach command during DDS discovery.
        self.create_timer(0.2, self._publish_all_states)
        if bool(self.get_parameter('auto_attach').value):
            self._schedule_once(
                float(self.get_parameter('auto_attach_delay').value),
                partial(self._command_pair, True))
        self.get_logger().info(
            'Grasp manager ready; service /cooperative_transport/attach_all')

    def _on_state(self, robot: str, message: Bool) -> None:
        self._states[robot] = message.data
        self._publish_state(robot)
        self._publish_status()

    def _on_attach_all(self, request: SetBool.Request,
                       response: SetBool.Response) -> SetBool.Response:
        self._command_pair(request.data)
        response.success = True
        operation = 'attach' if request.data else 'detach'
        response.message = f'{operation} sequence sent; inspect grasp_status for confirmation.'
        return response

    def _command_pair(self, attach: bool) -> None:
        # Attach leader first, follower second. Detach in reverse order.
        order = self._names if attach else tuple(reversed(self._names))
        self._publish_command(order[0], attach)
        self._schedule_once(
            float(self.get_parameter('second_command_delay').value),
            partial(self._publish_command, order[1], attach))

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
