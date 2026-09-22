"""Run repeatable cooperative-transport trajectories and record metrics."""

import csv
import math
import os
from datetime import datetime
from typing import Dict, Optional, TextIO, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage

from .evaluation_trajectories import Waypoint, scenario_waypoints
from .evaluation_trajectories import transform_waypoints
from .kinematics import normalize_angle, yaw_from_quaternion
from .kinematics import Pose2
from .obstacle_clearance import CircularObstacle, minimum_system_clearance
from .wrench_metrics import RunningWrenchMetrics, WrenchVector


class TrajectoryEvaluator(Node):
    """Execute waypoint scenarios and save synchronized samples to CSV."""

    def __init__(self) -> None:
        super().__init__('trajectory_evaluator')
        self.declare_parameter('scenario', 'turn_30')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 2.0)
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('goal_settle_time', 0.5)
        self.declare_parameter('waypoint_timeout', 40.0)
        self.declare_parameter('position_tolerance', 0.025)
        self.declare_parameter('yaw_tolerance', math.radians(3.0))
        self.declare_parameter('pass_through_waypoints', False)
        self.declare_parameter('pass_through_final_waypoint', False)
        self.declare_parameter('pass_through_position_tolerance', 0.075)
        self.declare_parameter(
            'pass_through_yaw_tolerance', math.radians(6.0))
        self.declare_parameter('pass_through_slip_tolerance', 0.015)
        self.declare_parameter(
            'results_directory', '/tmp/cooperative_transport_results')
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter(
            'obstacle_data', [2.5, 1.0, 0.18, 4.0, -1.0, 0.18])
        self.declare_parameter('payload_length', 1.20)
        self.declare_parameter('payload_width', 0.03)
        self.declare_parameter('robot_footprint_radius', 0.24)
        self.declare_parameter('use_world_model_poses', False)
        self.declare_parameter('world_name', 'cooperative_transport_friction')
        self.declare_parameter('require_support_removed', False)
        self.declare_parameter('enable_obstacle_evaluation', True)

        self._scenario = str(self.get_parameter('scenario').value)
        # Validate early so a typo cannot command an unintended trajectory.
        scenario_waypoints(self._scenario)
        self._names = (
            self.get_parameter('robot1_namespace').value.strip('/'),
            self.get_parameter('robot2_namespace').value.strip('/'),
        )
        obstacle_data = list(self.get_parameter('obstacle_data').value)
        if not obstacle_data or len(obstacle_data) % 3 != 0:
            raise ValueError('obstacle_data must contain x, y, radius triples.')
        if bool(self.get_parameter('enable_obstacle_evaluation').value):
            self._obstacles = [
                CircularObstacle(
                    f'cylinder_{index + 1}',
                    float(obstacle_data[3 * index]),
                    float(obstacle_data[3 * index + 1]),
                    float(obstacle_data[3 * index + 2]))
                for index in range(len(obstacle_data) // 3)
            ]
        else:
            self._obstacles = []
        self._grasped = {name: False for name in self._names}
        self._base_poses: Dict[str, Waypoint] = {}
        self._base_commands: Dict[str, Tuple[float, float, float]] = {}
        self._measured: Optional[Waypoint] = None
        self._estimated: Optional[Waypoint] = None
        self._slip_error = (0.0, 0.0, 0.0, 0.0)
        self._safety_status = 'UNKNOWN'
        self._slip_status = 'UNKNOWN'
        self._coordinator_state = 'UNKNOWN'
        self._admittance_status = 'UNKNOWN'
        self._admittance_data = (0.0, 0.0, 0.0, 0.0)
        self._support_removed = False
        self._wrenches: Dict[str, WrenchVector] = {}
        self._wrench_metrics = RunningWrenchMetrics()

        self._mode = 'WAITING_FOR_GRASP'
        self._ready_since: Optional[float] = None
        self._run_started = 0.0
        self._goal_started = 0.0
        self._settled_since: Optional[float] = None
        self._waypoints = []
        self._waypoint_index = 0
        self._result_path = ''
        self._result_file: Optional[TextIO] = None
        self._csv_writer = None
        self._max_position_error = 0.0
        self._max_yaw_error = 0.0
        self._max_horizontal_slip = 0.0
        self._max_vertical_slip = 0.0
        self._max_yaw_slip = 0.0
        self._initial_separation: Optional[float] = None
        self._max_separation_change = 0.0
        self._minimum_obstacle_clearance = float('inf')
        self._closest_obstacle_pair = (
            'unavailable' if self._obstacles else 'none (open field)')
        self._obstacle_collision = False
        self._obstacle_collision_count = 0
        self._obstacle_collision_pair = ''
        self._max_admittance_force_error = 0.0
        self._max_admittance_displacement = 0.0
        self._max_admittance_correction = 0.0

        for name in self._names:
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name: self._on_grasp(robot, message), 10)
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name: self._on_odometry(robot, message), 10)
            self.create_subscription(
                Twist, f'/{name}/rover_twist',
                lambda message, robot=name: self._on_command(robot, message), 10)
            self.create_subscription(
                WrenchStamped, f'/{name}/ft_sensor',
                lambda message, robot=name: self._on_wrench(robot, message), 10)
        self.create_subscription(
            PoseStamped, '/cooperative_transport/measured_payload_pose',
            self._on_measured_pose, 10)
        self.create_subscription(
            PoseStamped, '/cooperative_transport/payload_pose',
            self._on_estimated_pose, 10)
        world = self.get_parameter('world_name').value
        self.create_subscription(
            TFMessage, f'/world/{world}/pose/info',
            self._on_world_poses, 10)
        self.create_subscription(
            Float64MultiArray, '/cooperative_transport/slip_error',
            self._on_slip_error, 10)
        self.create_subscription(
            String, '/cooperative_transport/safety_status',
            lambda message: setattr(self, '_safety_status', message.data), 10)
        self.create_subscription(
            String, '/cooperative_transport/slip_status',
            lambda message: setattr(self, '_slip_status', message.data), 10)
        self.create_subscription(
            String, '/cooperative_transport/state',
            lambda message: setattr(self, '_coordinator_state', message.data), 10)
        self.create_subscription(
            Float64MultiArray,
            '/cooperative_transport/admittance_correction',
            self._on_admittance, 10)
        self.create_subscription(
            String, '/cooperative_transport/admittance_status',
            lambda message: setattr(
                self, '_admittance_status', message.data), 10)
        support_qos = QoSProfile(depth=1)
        support_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_transport/support_removed',
            lambda message: setattr(self, '_support_removed', message.data),
            support_qos)
        for obstacle in self._obstacles:
            self.create_subscription(
                Contacts,
                f'/cooperative_transport/{obstacle.name}/contacts',
                lambda message, name=obstacle.name:
                self._on_obstacle_contacts(name, message), 10)

        self._goal_publisher = self.create_publisher(
            PoseStamped, '/cooperative_transport/goal', 10)
        self._status_publisher = self.create_publisher(
            String, '/cooperative_transport/evaluation_status', 10)
        result_qos = QoSProfile(depth=1)
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._result_publisher = self.create_publisher(
            String, '/cooperative_transport/evaluation_result', result_qos)
        self._obstacle_status_publisher = self.create_publisher(
            String, '/cooperative_transport/obstacle_status', 10)
        self._clearance_publisher = self.create_publisher(
            Float64, '/cooperative_transport/minimum_clearance', 10)
        self._enable_client = self.create_client(
            SetBool, '/cooperative_transport/set_enabled')
        self.create_service(
            Trigger, '/cooperative_transport/start_evaluation', self._on_start)
        self.create_service(
            Trigger, '/cooperative_transport/cancel_evaluation', self._on_cancel)

        rate = float(self.get_parameter('sample_rate').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Trajectory evaluator ready; scenario={self._scenario}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_grasp(self, robot: str, message: Bool) -> None:
        self._grasped[robot] = message.data

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        if bool(self.get_parameter('use_world_model_poses').value):
            return
        pose = message.pose.pose
        orientation = pose.orientation
        self._base_poses[robot] = Waypoint(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                orientation.x, orientation.y,
                orientation.z, orientation.w),
        )

    def _on_world_poses(self, message: TFMessage) -> None:
        if not bool(self.get_parameter('use_world_model_poses').value):
            return
        for transform in message.transforms:
            name = transform.child_frame_id
            if name not in self._names:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._base_poses[name] = Waypoint(
                translation.x,
                translation.y,
                yaw_from_quaternion(
                    rotation.x, rotation.y, rotation.z, rotation.w),
            )

    def _on_command(self, robot: str, message: Twist) -> None:
        self._base_commands[robot] = (
            message.linear.x, message.linear.y, message.angular.z)

    def _on_wrench(self, robot: str, message: WrenchStamped) -> None:
        wrench = message.wrench
        self._wrenches[robot] = WrenchVector(
            wrench.force.x, wrench.force.y, wrench.force.z,
            wrench.torque.x, wrench.torque.y, wrench.torque.z)

    def _pose_to_waypoint(self, message: PoseStamped) -> Waypoint:
        pose = message.pose
        orientation = pose.orientation
        return Waypoint(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w),
        )

    def _on_measured_pose(self, message: PoseStamped) -> None:
        self._measured = self._pose_to_waypoint(message)

    def _on_estimated_pose(self, message: PoseStamped) -> None:
        self._estimated = self._pose_to_waypoint(message)

    def _on_slip_error(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 4:
            self._slip_error = tuple(message.data[:4])

    def _on_admittance(self, message: Float64MultiArray) -> None:
        if len(message.data) >= 4:
            self._admittance_data = tuple(message.data[:4])

    def _on_obstacle_contacts(
            self, obstacle: str, message: Contacts) -> None:
        targets = ('amir1', 'amir2', 'cooperative_payload')
        for contact in message.contacts:
            first = contact.collision1.name
            second = contact.collision2.name
            combined = f'{first} {second}'
            if not any(target in combined for target in targets):
                continue
            if not self._obstacle_collision:
                self._obstacle_collision_count += 1
                self._obstacle_collision_pair = (
                    f'{obstacle}: {first} <-> {second}')
                self.get_logger().error(
                    f'Obstacle contact detected: {self._obstacle_collision_pair}')
            self._obstacle_collision = True
            return

    def _on_start(self, _request: Trigger.Request,
                  response: Trigger.Response) -> Trigger.Response:
        success, text = self._start_run()
        response.success = success
        response.message = text
        return response

    def _on_cancel(self, _request: Trigger.Request,
                   response: Trigger.Response) -> Trigger.Response:
        if self._mode not in ('ENABLING', 'ENABLE_REQUESTED', 'RUNNING'):
            response.success = False
            response.message = f'No active evaluation; state={self._mode}.'
            return response
        self._finish(False, 'cancelled by operator')
        response.success = True
        response.message = 'Evaluation cancelled and transport disabled.'
        return response

    def _start_run(self):
        if self._mode in ('ENABLING', 'RUNNING'):
            return False, 'Evaluation is already running.'
        if not all(self._grasped.values()):
            return False, 'Both friction grasps must be HOLDING.'
        if (bool(self.get_parameter('require_support_removed').value)
                and not self._support_removed):
            return False, 'Temporary grasp support has not been removed.'
        if self._measured is None:
            return False, 'Measured payload pose is unavailable.'
        relative = scenario_waypoints(self._scenario)
        self._waypoints = transform_waypoints(relative, self._measured)
        self._waypoint_index = 0
        self._max_position_error = 0.0
        self._max_yaw_error = 0.0
        self._max_horizontal_slip = 0.0
        self._max_vertical_slip = 0.0
        self._max_yaw_slip = 0.0
        self._max_separation_change = 0.0
        self._minimum_obstacle_clearance = float('inf')
        self._closest_obstacle_pair = (
            'unavailable' if self._obstacles else 'none (open field)')
        self._obstacle_collision = False
        self._obstacle_collision_count = 0
        self._obstacle_collision_pair = ''
        self._max_admittance_force_error = 0.0
        self._max_admittance_displacement = 0.0
        self._max_admittance_correction = 0.0
        self._wrench_metrics.reset()
        self._run_started = self._now()
        self._initial_separation = self._separation()
        self._open_csv()
        self._publish_goal()
        self._mode = 'ENABLING'
        self._goal_started = self._now()
        self._settled_since = None
        return True, f'Started scenario {self._scenario}; output={self._result_path}'

    def _open_csv(self) -> None:
        directory = os.path.expanduser(
            str(self.get_parameter('results_directory').value))
        os.makedirs(directory, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._result_path = os.path.join(
            directory, f'{stamp}_{self._scenario}.csv')
        self._result_file = open(  # noqa: SIM115 - kept open during the run
            self._result_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._result_file)
        self._csv_writer.writerow([
            'sim_time_s', 'elapsed_s', 'scenario', 'waypoint_index',
            'goal_x_m', 'goal_y_m', 'goal_yaw_rad',
            'measured_x_m', 'measured_y_m', 'measured_yaw_rad',
            'estimated_x_m', 'estimated_y_m', 'estimated_yaw_rad',
            'estimate_position_error_m', 'estimate_yaw_error_rad',
            'slip_x_m', 'slip_y_m', 'slip_z_m', 'slip_yaw_rad',
            'robot_separation_m',
            'robot1_x_m', 'robot1_y_m', 'robot1_yaw_rad',
            'robot2_x_m', 'robot2_y_m', 'robot2_yaw_rad',
            'robot1_cmd_x_mps', 'robot1_cmd_y_mps', 'robot1_cmd_yaw_radps',
            'robot2_cmd_x_mps', 'robot2_cmd_y_mps', 'robot2_cmd_yaw_radps',
            'robot1_fx_N', 'robot1_fy_N', 'robot1_fz_N',
            'robot1_mx_Nm', 'robot1_my_Nm', 'robot1_mz_Nm',
            'robot2_fx_N', 'robot2_fy_N', 'robot2_fz_N',
            'robot2_mx_Nm', 'robot2_my_Nm', 'robot2_mz_Nm',
            'wrench_force_rms_N', 'wrench_peak_force_N',
            'wrench_torque_rms_Nm', 'wrench_peak_torque_Nm',
            'admittance_force_error_N', 'admittance_filtered_force_N',
            'admittance_displacement_m', 'admittance_correction_mps',
            'obstacle_clearance_m', 'closest_obstacle_pair',
            'obstacle_collision_count', 'obstacle_status',
            'coordinator_state',
            'slip_status', 'safety_status',
        ])

    def _publish_goal(self) -> None:
        goal = self._waypoints[self._waypoint_index]
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        message.pose.position.x = goal.x
        message.pose.position.y = goal.y
        message.pose.orientation.z = math.sin(0.5 * goal.yaw)
        message.pose.orientation.w = math.cos(0.5 * goal.yaw)
        self._goal_publisher.publish(message)
        self._goal_started = self._now()
        self._settled_since = None
        self.get_logger().info(
            f'Waypoint {self._waypoint_index + 1}/{len(self._waypoints)}: '
            f'x={goal.x:.3f}, y={goal.y:.3f}, '
            f'yaw={math.degrees(goal.yaw):.1f} deg')

    def _request_enabled(self, enabled: bool) -> None:
        if not self._enable_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = enabled
        future = self._enable_client.call_async(request)
        if enabled:
            future.add_done_callback(self._on_enabled)

    def _on_enabled(self, future) -> None:
        try:
            response = future.result()
        except Exception as exception:  # noqa: B902 - ROS future
            self._finish(False, f'enable service failed: {exception}')
            return
        if response.success:
            self._mode = 'RUNNING'
            self.get_logger().info('Cooperative transport enabled for evaluation.')
        else:
            self._finish(False, f'enable rejected: {response.message}')

    def _tick(self) -> None:
        now = self._now()
        support_ready = (
            not bool(self.get_parameter('require_support_removed').value)
            or self._support_removed)
        ready = (
            all(self._grasped.values())
            and self._measured is not None
            and support_ready)
        if self._mode == 'WAITING_FOR_GRASP' and ready:
            if self._ready_since is None:
                self._ready_since = now
            delay = float(self.get_parameter('auto_start_delay').value)
            if (bool(self.get_parameter('auto_start').value)
                    and now - self._ready_since >= delay):
                success, message = self._start_run()
                if not success:
                    self.get_logger().error(message)
        elif self._mode == 'ENABLING':
            if now - self._goal_started >= 0.5:
                self._request_enabled(True)
                # Avoid multiple async requests while the response is pending.
                self._mode = 'ENABLE_REQUESTED'
        elif self._mode == 'ENABLE_REQUESTED':
            pass
        elif self._mode == 'RUNNING':
            self._sample()
            if self._obstacle_collision:
                self._finish(
                    False, f'obstacle collision: {self._obstacle_collision_pair}')
            elif self._safety_status.startswith('STOPPED'):
                self._finish(False, self._safety_status)
            elif self._slip_status.startswith('SLIP'):
                self._finish(False, self._slip_status)
            elif now - self._goal_started > float(
                    self.get_parameter('waypoint_timeout').value):
                self._finish(False, 'waypoint timeout')
            else:
                self._advance_if_settled(now)
        self._publish_status()

    def _advance_if_settled(self, now: float) -> None:
        if self._measured is None:
            return
        goal = self._waypoints[self._waypoint_index]
        position_error = math.hypot(
            goal.x - self._measured.x, goal.y - self._measured.y)
        yaw_error = abs(normalize_angle(goal.yaw - self._measured.yaw))
        final_waypoint = self._waypoint_index + 1 == len(self._waypoints)
        pass_through = (
            bool(self.get_parameter('pass_through_waypoints').value)
            and (not final_waypoint or bool(self.get_parameter(
                'pass_through_final_waypoint').value)))
        position_tolerance = float(self.get_parameter(
            'pass_through_position_tolerance' if pass_through
            else 'position_tolerance').value)
        yaw_tolerance = float(self.get_parameter(
            'pass_through_yaw_tolerance' if pass_through
            else 'yaw_tolerance').value)
        inside = (
            position_error <= position_tolerance
            and yaw_error <= yaw_tolerance
            and (not pass_through or math.hypot(
                self._slip_error[0], self._slip_error[1]) <= float(
                    self.get_parameter(
                        'pass_through_slip_tolerance').value))
            and (pass_through or self._coordinator_state == 'HOLDING'))
        if not inside:
            self._settled_since = None
            return
        if pass_through:
            if final_waypoint:
                self._finish(True, 'all waypoints completed')
                return
            self._waypoint_index += 1
            self._publish_goal()
            return
        if self._settled_since is None:
            self._settled_since = now
            return
        if now - self._settled_since < float(
                self.get_parameter('goal_settle_time').value):
            return
        if final_waypoint:
            self._finish(True, 'all waypoints completed')
            return
        self._waypoint_index += 1
        self._publish_goal()

    def _separation(self) -> float:
        if len(self._base_poses) != 2:
            return float('nan')
        first = self._base_poses[self._names[0]]
        second = self._base_poses[self._names[1]]
        return math.hypot(second.x - first.x, second.y - first.y)

    def _obstacle_clearance(self) -> Tuple[float, str]:
        if not self._obstacles:
            return float('inf'), 'none (open field)'
        if self._measured is None or len(self._base_poses) != 2:
            return float('nan'), 'unavailable'
        robots = [
            Pose2(*self._base_poses[name]) for name in self._names
        ]
        return minimum_system_clearance(
            Pose2(*self._measured), robots, self._obstacles,
            float(self.get_parameter('payload_length').value),
            float(self.get_parameter('payload_width').value),
            float(self.get_parameter('robot_footprint_radius').value),
        )

    def _sample(self) -> None:
        if self._measured is None or self._csv_writer is None:
            return
        nan = float('nan')
        estimated = self._estimated or Waypoint(nan, nan, nan)
        estimate_position_error = math.hypot(
            estimated.x - self._measured.x, estimated.y - self._measured.y)
        estimate_yaw_error = abs(normalize_angle(
            estimated.yaw - self._measured.yaw))
        separation = self._separation()
        self._max_position_error = max(
            self._max_position_error, estimate_position_error)
        self._max_yaw_error = max(self._max_yaw_error, estimate_yaw_error)
        horizontal_slip = math.hypot(self._slip_error[0], self._slip_error[1])
        self._max_horizontal_slip = max(
            self._max_horizontal_slip, horizontal_slip)
        self._max_vertical_slip = max(
            self._max_vertical_slip, abs(self._slip_error[2]))
        self._max_yaw_slip = max(
            self._max_yaw_slip, abs(self._slip_error[3]))
        if self._initial_separation is not None and math.isfinite(separation):
            self._max_separation_change = max(
                self._max_separation_change,
                abs(separation - self._initial_separation))
        obstacle_clearance, obstacle_pair = self._obstacle_clearance()
        if (math.isfinite(obstacle_clearance)
                and obstacle_clearance < self._minimum_obstacle_clearance):
            self._minimum_obstacle_clearance = obstacle_clearance
            self._closest_obstacle_pair = obstacle_pair
        goal = self._waypoints[self._waypoint_index]
        now = self._now()
        empty_pose = Waypoint(nan, nan, nan)
        empty_command = (nan, nan, nan)
        robot1 = self._base_poses.get(self._names[0], empty_pose)
        robot2 = self._base_poses.get(self._names[1], empty_pose)
        command1 = self._base_commands.get(self._names[0], empty_command)
        command2 = self._base_commands.get(self._names[1], empty_command)
        empty_wrench = WrenchVector(nan, nan, nan, nan, nan, nan)
        wrench1 = self._wrenches.get(self._names[0], empty_wrench)
        wrench2 = self._wrenches.get(self._names[1], empty_wrench)
        for wrench in (wrench1, wrench2):
            if math.isfinite(wrench.force_norm):
                self._wrench_metrics.update(wrench)
        self._max_admittance_force_error = max(
            self._max_admittance_force_error,
            abs(self._admittance_data[0]))
        self._max_admittance_displacement = max(
            self._max_admittance_displacement,
            abs(self._admittance_data[2]))
        self._max_admittance_correction = max(
            self._max_admittance_correction,
            abs(self._admittance_data[3]))
        self._csv_writer.writerow([
            f'{now:.6f}', f'{now - self._run_started:.6f}', self._scenario,
            self._waypoint_index, f'{goal.x:.6f}', f'{goal.y:.6f}',
            f'{goal.yaw:.6f}', f'{self._measured.x:.6f}',
            f'{self._measured.y:.6f}', f'{self._measured.yaw:.6f}',
            f'{estimated.x:.6f}', f'{estimated.y:.6f}',
            f'{estimated.yaw:.6f}', f'{estimate_position_error:.6f}',
            f'{estimate_yaw_error:.6f}',
            *[f'{value:.6f}' for value in self._slip_error],
            f'{separation:.6f}',
            f'{robot1.x:.6f}', f'{robot1.y:.6f}', f'{robot1.yaw:.6f}',
            f'{robot2.x:.6f}', f'{robot2.y:.6f}', f'{robot2.yaw:.6f}',
            *[f'{value:.6f}' for value in command1],
            *[f'{value:.6f}' for value in command2],
            *[f'{value:.6f}' for value in (
                wrench1.fx, wrench1.fy, wrench1.fz,
                wrench1.tx, wrench1.ty, wrench1.tz,
                wrench2.fx, wrench2.fy, wrench2.fz,
                wrench2.tx, wrench2.ty, wrench2.tz)],
            f'{self._wrench_metrics.force_rms:.6f}',
            f'{self._wrench_metrics.peak_force:.6f}',
            f'{self._wrench_metrics.torque_rms:.6f}',
            f'{self._wrench_metrics.peak_torque:.6f}',
            *[f'{value:.6f}' for value in self._admittance_data],
            f'{obstacle_clearance:.6f}', obstacle_pair,
            self._obstacle_collision_count,
            'COLLISION' if self._obstacle_collision else 'CLEAR',
            self._coordinator_state,
            self._slip_status, self._safety_status,
        ])

    def _finish(self, success: bool, reason: str) -> None:
        self._request_enabled(False)
        self._mode = 'COMPLETE' if success else 'FAILED'
        if self._result_file is not None:
            self._result_file.flush()
            self._result_file.close()
            self._result_file = None
        duration = max(0.0, self._now() - self._run_started)
        clearance = self._minimum_obstacle_clearance
        if not self._obstacles:
            clearance_text = 'not_applicable'
        else:
            clearance_text = (
                f'{clearance:.4f}m' if math.isfinite(clearance) else 'nan')
        result = (
            f'{"PASS" if success else "FAIL"}: scenario={self._scenario}; '
            f'reason={reason}; duration={duration:.2f}s; '
            f'max_estimate_error={self._max_position_error:.4f}m; '
            f'max_yaw_error={math.degrees(self._max_yaw_error):.2f}deg; '
            f'max_horizontal_slip={self._max_horizontal_slip:.4f}m; '
            f'max_vertical_slip={self._max_vertical_slip:.4f}m; '
            f'max_yaw_slip={math.degrees(self._max_yaw_slip):.2f}deg; '
            f'max_separation_change={self._max_separation_change:.4f}m; '
            f'wrench_force_rms={self._wrench_metrics.force_rms:.2f}N; '
            f'wrench_peak_force={self._wrench_metrics.peak_force:.2f}N; '
            f'wrench_torque_rms={self._wrench_metrics.torque_rms:.2f}Nm; '
            f'wrench_peak_torque={self._wrench_metrics.peak_torque:.2f}Nm; '
            f'max_admittance_force_error='
            f'{self._max_admittance_force_error:.2f}N; '
            f'max_admittance_displacement='
            f'{self._max_admittance_displacement:.4f}m; '
            f'max_admittance_correction='
            f'{self._max_admittance_correction:.4f}m/s; '
            f'min_obstacle_clearance={clearance_text}; '
            f'closest_pair={self._closest_obstacle_pair}; '
            f'obstacle_collisions={self._obstacle_collision_count}; '
            f'csv={self._result_path}')
        message = String()
        message.data = result
        self._result_publisher.publish(message)
        if success:
            self.get_logger().info(result)
        else:
            self.get_logger().error(result)

    def _publish_status(self) -> None:
        message = String()
        message.data = (
            f'{self._mode}; scenario={self._scenario}; '
            f'waypoint={self._waypoint_index + 1}/{len(self._waypoints)}; '
            f'support_removed={self._support_removed}')
        self._status_publisher.publish(message)
        obstacle = String()
        obstacle.data = (
            f'COLLISION: {self._obstacle_collision_pair}'
            if self._obstacle_collision else
            f'CLEAR; minimum={self._minimum_obstacle_clearance:.3f}m; '
            f'pair={self._closest_obstacle_pair}')
        self._obstacle_status_publisher.publish(obstacle)
        clearance = Float64()
        clearance.data = self._minimum_obstacle_clearance
        self._clearance_publisher.publish(clearance)

    def destroy_node(self):
        """Close an unfinished CSV before destroying the ROS node."""
        if self._result_file is not None:
            self._result_file.close()
            self._result_file = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryEvaluator()
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
