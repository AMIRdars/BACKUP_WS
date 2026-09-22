"""Stateful specified-axis rotation planner for two cooperating AMIRs."""

from collections import deque
import math
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from .trajectory import (
    AccelerationLimitedTrajectory,
    settling_transport_angle,
    TrackingErrorGovernor,
)
from .transform_utils import (
    invert_transform,
    make_transform,
    normalized,
    rotate_transform_about_axis,
    rotation_error,
    signed_axis_rotation,
    transform_components,
)


class CooperativeRotationController(Node):
    """Generate one payload trajectory and synchronized grasp targets."""

    STATES = (
        'WAITING', 'GRASP_CHECK', 'INITIALIZE', 'TRANSLATING',
        'TRANSLATION_SETTLING', 'ROTATING', 'SETTLING',
        'POST_ROTATION_TRANSLATING', 'POST_ROTATION_SETTLING', 'FINISHED',
        'ERROR')

    def __init__(self) -> None:
        super().__init__('cooperative_rotation_controller')
        self._declare_parameters()

        self._names = (
            str(self.get_parameter('robot1_namespace').value).strip('/'),
            str(self.get_parameter('robot2_namespace').value).strip('/'),
        )
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._gripper_frame = str(self.get_parameter('gripper_frame').value)
        self._axis = normalized(self.get_parameter('axis_direction').value)
        self._target_angle = math.radians(
            float(self.get_parameter('target_angle_deg').value))
        self._trajectory = AccelerationLimitedTrajectory(
            self._target_angle,
            math.radians(float(
                self.get_parameter('angular_velocity_deg_s').value)),
            math.radians(float(
                self.get_parameter('angular_acceleration_deg_s2').value)))
        self._translation_trajectory = AccelerationLimitedTrajectory(
            float(self.get_parameter('pre_rotation_translation_distance_m').value),
            float(self.get_parameter('translation_velocity_m_s').value),
            float(self.get_parameter('translation_acceleration_m_s2').value))
        self._post_rotation_translation_trajectory = (
            AccelerationLimitedTrajectory(
                float(self.get_parameter(
                    'post_rotation_translation_y_distance_m').value),
                float(self.get_parameter('translation_velocity_m_s').value),
                float(self.get_parameter(
                    'translation_acceleration_m_s2').value)))
        self._tracking_governor = TrackingErrorGovernor(
            math.radians(float(self.get_parameter(
                'tracking_pause_orientation_error_deg').value)),
            math.radians(float(self.get_parameter(
                'tracking_resume_orientation_error_deg').value)),
        )
        self._translation_tracking_governor = TrackingErrorGovernor(
            float(self.get_parameter(
                'translation_tracking_pause_position_error').value),
            float(self.get_parameter(
                'translation_tracking_resume_position_error').value),
        )

        self._state = 'WAITING'
        self._status_detail = 'waiting for robot poses'
        self._start_requested = False
        self._ready_since: Optional[float] = None
        self._motion_started = 0.0
        self._trajectory_elapsed = 0.0
        self._trajectory_clock_time: Optional[float] = None
        self._translation_elapsed = 0.0
        self._translation_clock_time: Optional[float] = None
        self._translation_target: Optional[np.ndarray] = None
        self._translation_settling_started = 0.0
        self._translation_within_tolerance_since: Optional[float] = None
        self._post_rotation_translation_elapsed = 0.0
        self._post_rotation_translation_clock_time: Optional[float] = None
        self._post_rotation_translation_target: Optional[np.ndarray] = None
        self._post_rotation_settling_started = 0.0
        self._post_rotation_within_tolerance_since: Optional[float] = None
        self._settling_started = 0.0
        self._within_tolerance_since: Optional[float] = None
        self._completion_published = False
        self._emergency_stop = False
        self._support_removed: Optional[bool] = None
        self._finger_force_history = {
            (robot, finger): deque()
            for robot in self._names for finger in ('left', 'right')}

        self._base_transforms: Dict[str, np.ndarray] = {}
        self._base_times: Dict[str, float] = {}
        self._grasped: Dict[str, Optional[bool]] = {
            name: None for name in self._names}
        self._tf_edges: Dict[str, Dict[Tuple[str, str], np.ndarray]] = {
            name: {} for name in self._names}
        self._measured_payload: Optional[np.ndarray] = None
        self._measured_payload_time = 0.0
        self._controller_clients = {
            name: self.create_client(
                ListControllers,
                f'/{name}/controller_manager/list_controllers')
            for name in self._names
        }
        self._controller_futures = {name: None for name in self._names}
        self._controller_request_times = {name: 0.0 for name in self._names}
        self._controllers_active = {name: False for name in self._names}

        self._object_initial: Optional[np.ndarray] = None
        self._object_target: Optional[np.ndarray] = None
        self._object_to_gripper: Dict[str, np.ndarray] = {}
        self._axis_point: Optional[np.ndarray] = None
        self._theta_desired = 0.0
        self._theta_actual = 0.0

        for name in self._names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda message, robot=name:
                self._on_odometry(robot, message), 20)
            self.create_subscription(
                Bool, f'/cooperative_transport/{name}/grasp_state',
                lambda message, robot=name:
                self._on_grasp(robot, message), 20)
            self.create_subscription(
                TFMessage, f'/{name}/tf',
                lambda message, robot=name:
                self._on_tf(robot, message), 100)
            for finger in ('left', 'right'):
                self.create_subscription(
                    WrenchStamped, f'/{name}/finger_{finger}_wrench',
                    lambda message, robot=name, side=finger:
                    self._on_finger_wrench(robot, side, message), 100)

            static_qos = QoSProfile(depth=100)
            static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                TFMessage, f'/{name}/tf_static',
                lambda message, robot=name:
                self._on_tf(robot, message), static_qos)

        self.create_subscription(
            PoseStamped, '/cooperative_transport/measured_payload_pose',
            self._on_measured_payload, 20)
        self.create_subscription(
            Bool, '/cooperative_transport/emergency_stop',
            self._on_emergency_stop, 20)
        support_qos = QoSProfile(depth=1)
        support_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_transport/support_removed',
            self._on_support_removed, support_qos)

        self._goal_publisher = self.create_publisher(
            PoseStamped, '/cooperative_transport/goal', 20)
        self._enable_publisher = self.create_publisher(
            Bool, '/cooperative_transport/enable', 20)
        self._object_target_publisher = self.create_publisher(
            PoseStamped, '/cooperative_rotation/object_target', 20)
        self._gripper_target_publishers = {
            name: self.create_publisher(
                PoseStamped,
                f'/cooperative_rotation/{name}/gripper_target', 20)
            for name in self._names
        }
        self._state_publisher = self.create_publisher(
            String, '/cooperative_rotation/state', 20)
        self._status_publisher = self.create_publisher(
            String, '/cooperative_rotation/status', 20)
        self._angle_publisher = self.create_publisher(
            Float64, '/cooperative_rotation/angle_deg', 20)
        self._desired_angle_publisher = self.create_publisher(
            Float64, '/cooperative_rotation/desired_angle_deg', 20)
        result_qos = QoSProfile(depth=1)
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._complete_publisher = self.create_publisher(
            Bool, '/cooperative_rotation/complete', result_qos)
        self._marker_publisher = self.create_publisher(
            MarkerArray, '/cooperative_rotation/markers', 10)

        self.create_service(
            Trigger, '/cooperative_rotation/start', self._on_start)
        self.create_service(
            Trigger, '/cooperative_rotation/cancel', self._on_cancel)

        period = float(self.get_parameter('control_period').value)
        if period <= 0.0:
            raise ValueError('control_period must be positive.')
        self.create_timer(period, self._control_loop)
        self.get_logger().info(
            'Rotation controller ready: target='
            f'{math.degrees(self._target_angle):.1f} deg, '
            f'axis={self._axis.tolist()}')

    def _declare_parameters(self) -> None:
        self.declare_parameter('robot1_namespace', 'amir1')
        self.declare_parameter('robot2_namespace', 'amir2')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('gripper_frame', 'tcp_link')
        self.declare_parameter(
            'required_controllers',
            ['arm_controller', 'mecanum_drive_controller'])
        self.declare_parameter('axis_point_at_object_center', True)
        self.declare_parameter('axis_point', [0.0, 0.0, 0.443])
        self.declare_parameter('axis_direction', [0.0, 0.0, 1.0])
        self.declare_parameter('target_angle_deg', 10.0)
        self.declare_parameter('angular_velocity_deg_s', 2.0)
        self.declare_parameter('angular_acceleration_deg_s2', 0.25)
        self.declare_parameter('pre_rotation_translation_distance_m', 1.0)
        self.declare_parameter('post_rotation_translation_y_distance_m', 1.0)
        self.declare_parameter('translation_velocity_m_s', 0.05)
        self.declare_parameter('translation_acceleration_m_s2', 0.025)
        self.declare_parameter('translation_settling_time', 1.0)
        self.declare_parameter('translation_settling_timeout', 20.0)
        self.declare_parameter('translation_tracking_pause_position_error', 0.04)
        self.declare_parameter('translation_tracking_resume_position_error', 0.02)
        self.declare_parameter('control_period', 0.01)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('orientation_tolerance_deg', 2.0)
        self.declare_parameter('maximum_tracking_position_error', 0.10)
        self.declare_parameter(
            'maximum_tracking_orientation_error_deg', 15.0)
        self.declare_parameter('tracking_pause_orientation_error_deg', 2.0)
        self.declare_parameter('tracking_resume_orientation_error_deg', 1.0)
        self.declare_parameter('settling_goal_correction_gain', 2.0)
        self.declare_parameter('settling_goal_max_correction_deg', 4.0)
        self.declare_parameter('pose_timeout', 0.50)
        self.declare_parameter('settling_time', 2.0)
        self.declare_parameter('settling_timeout', 12.0)
        self.declare_parameter('payload_height', 0.443)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 2.0)
        self.declare_parameter('require_grasp', True)
        self.declare_parameter('require_support_removed', False)
        self.declare_parameter('pre_rotation_force_stability_duration', 2.0)
        self.declare_parameter('pre_rotation_minimum_normal_force', 8.0)
        self.declare_parameter('pre_rotation_target_normal_force', 14.0)
        self.declare_parameter('pre_rotation_target_tolerance', 5.0)
        self.declare_parameter('pre_rotation_force_variation_limit', 2.0)
        self.declare_parameter('pre_rotation_force_timeout', 0.25)
        self.declare_parameter('publish_markers', True)

    @staticmethod
    def _frame(frame: str) -> str:
        return frame.strip('/')

    @staticmethod
    def _pose_transform(pose: Pose) -> np.ndarray:
        return make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])

    def _on_odometry(self, robot: str, message: Odometry) -> None:
        self._base_transforms[robot] = self._pose_transform(message.pose.pose)
        self._base_times[robot] = time.monotonic()

    def _on_grasp(self, robot: str, message: Bool) -> None:
        self._grasped[robot] = message.data
        if (self._state in (
                'TRANSLATING', 'TRANSLATION_SETTLING', 'ROTATING', 'SETTLING',
                'POST_ROTATION_TRANSLATING', 'POST_ROTATION_SETTLING')
                and bool(self.get_parameter('require_grasp').value)
                and not message.data):
            self._enter_error(f'grasp lost: {robot}')

    def _on_finger_wrench(
            self, robot: str, finger: str, message: WrenchStamped) -> None:
        """Keep a short ROS-time history for the pre-rotation stability gate."""
        now = self.get_clock().now().nanoseconds * 1.0e-9
        history = self._finger_force_history[(robot, finger)]
        history.append((now, abs(message.wrench.force.x)))
        keep = max(
            2.0,
            2.0 * float(self.get_parameter(
                'pre_rotation_force_stability_duration').value))
        while history and now - history[0][0] > keep:
            history.popleft()

    def _on_tf(self, robot: str, message: TFMessage) -> None:
        for stamped in message.transforms:
            parent = self._frame(stamped.header.frame_id)
            child = self._frame(stamped.child_frame_id)
            if not parent or not child:
                continue
            transform = stamped.transform
            self._tf_edges[robot][(parent, child)] = make_transform(
                [transform.translation.x, transform.translation.y,
                 transform.translation.z],
                [transform.rotation.x, transform.rotation.y,
                 transform.rotation.z, transform.rotation.w])

    def _on_measured_payload(self, message: PoseStamped) -> None:
        self._measured_payload = self._pose_transform(message.pose)
        self._measured_payload_time = time.monotonic()

    def _on_emergency_stop(self, message: Bool) -> None:
        self._emergency_stop = message.data
        if (message.data and self._state in (
                'TRANSLATING', 'TRANSLATION_SETTLING', 'ROTATING', 'SETTLING',
                'POST_ROTATION_TRANSLATING', 'POST_ROTATION_SETTLING')):
            self._enter_error('cooperative transport emergency stop')

    def _on_support_removed(self, message: Bool) -> None:
        self._support_removed = message.data
        if (self._state in (
                'TRANSLATING', 'TRANSLATION_SETTLING', 'ROTATING', 'SETTLING',
                'POST_ROTATION_TRANSLATING', 'POST_ROTATION_SETTLING')
                and bool(self.get_parameter(
                    'require_support_removed').value)
                and not message.data):
            self._enter_error('temporary payload support is present')

    def _lookup_robot_transform(
            self, robot: str, source: str, target: str
    ) -> Optional[np.ndarray]:
        source = self._frame(source)
        target = self._frame(target)
        if source == target:
            return np.eye(4)
        adjacency: Dict[str, list] = {}
        for (parent, child), transform in self._tf_edges[robot].items():
            adjacency.setdefault(parent, []).append((child, transform))
            adjacency.setdefault(child, []).append(
                (parent, invert_transform(transform)))
        queue = deque([(source, np.eye(4))])
        visited = {source}
        while queue:
            frame, accumulated = queue.popleft()
            for neighbor, edge in adjacency.get(frame, []):
                if neighbor in visited:
                    continue
                result = accumulated @ edge
                if neighbor == target:
                    return result
                visited.add(neighbor)
                queue.append((neighbor, result))
        return None

    def _current_gripper(self, robot: str) -> Optional[np.ndarray]:
        base = self._base_transforms.get(robot)
        base_to_gripper = self._lookup_robot_transform(
            robot, self._base_frame, self._gripper_frame)
        if base is None or base_to_gripper is None:
            return None
        return base @ base_to_gripper

    def _poses_fresh(self) -> bool:
        timeout = float(self.get_parameter('pose_timeout').value)
        now = time.monotonic()
        return all(
            robot in self._base_transforms
            and now - self._base_times.get(robot, 0.0) <= timeout
            for robot in self._names)

    def _current_object(self) -> Optional[np.ndarray]:
        timeout = float(self.get_parameter('pose_timeout').value)
        if (self._measured_payload is not None
                and time.monotonic() - self._measured_payload_time <= timeout):
            return self._measured_payload.copy()
        if not self._poses_fresh():
            return None
        first = self._base_transforms[self._names[0]]
        second = self._base_transforms[self._names[1]]
        difference = second[:2, 3] - first[:2, 3]
        if float(np.linalg.norm(difference)) <= 1.0e-9:
            return None
        yaw = math.atan2(difference[1], difference[0])
        center = 0.5 * (first[:3, 3] + second[:3, 3])
        center[2] = float(self.get_parameter('payload_height').value)
        return make_transform(
            center, [0.0, 0.0, math.sin(0.5 * yaw),
                     math.cos(0.5 * yaw)])

    def _ready_reason(
            self, require_pre_rotation_stability: bool = False
    ) -> Optional[str]:
        if self._emergency_stop:
            return 'emergency stop is active'
        if not self._poses_fresh():
            return 'waiting for fresh odometry from both robots'
        for name in self._names:
            if not self._controllers_active[name]:
                return f'waiting for {name} controllers to become active'
        if bool(self.get_parameter('require_grasp').value):
            if not all(self._grasped[name] is True for name in self._names):
                return 'waiting for both grasp states'
        if (bool(self.get_parameter('require_support_removed').value)
                and self._support_removed is not True):
            return 'waiting for temporary payload support removal'
        if require_pre_rotation_stability:
            force_reason = self._pre_rotation_force_stability_reason()
            if force_reason is not None:
                return force_reason
        if self._current_object() is None:
            return 'payload pose cannot be estimated'
        for name in self._names:
            if self._current_gripper(name) is None:
                return f'waiting for {name} {self._gripper_frame} transform'
        return None

    def _pre_rotation_force_stability_reason(self) -> Optional[str]:
        """Require settled four-finger preload before commanding rotation."""
        duration = float(self.get_parameter(
            'pre_rotation_force_stability_duration').value)
        if duration <= 0.0:
            return None
        now = self.get_clock().now().nanoseconds * 1.0e-9
        minimum = float(self.get_parameter(
            'pre_rotation_minimum_normal_force').value)
        target = float(self.get_parameter(
            'pre_rotation_target_normal_force').value)
        target_tolerance = float(self.get_parameter(
            'pre_rotation_target_tolerance').value)
        variation_limit = float(self.get_parameter(
            'pre_rotation_force_variation_limit').value)
        timeout = float(self.get_parameter(
            'pre_rotation_force_timeout').value)
        latest_forces = []
        for history in self._finger_force_history.values():
            window = [force for stamp, force in history if now - stamp <= duration]
            if not window or now - history[-1][0] > timeout:
                return 'waiting for fresh finger-force samples'
            if now - history[0][0] < duration:
                return 'waiting for pre-rotation force settling'
            if min(window) < minimum:
                return 'waiting for all fingers above minimum normal force'
            if max(window) - min(window) > variation_limit:
                return 'waiting for grasp-force variation to settle'
            latest_forces.append(window[-1])
        if abs(sum(latest_forces) / len(latest_forces) - target) > target_tolerance:
            return 'waiting for mean grasp force near target'
        return None

    def _poll_controllers(self, now: float) -> None:
        required = set(self.get_parameter('required_controllers').value)
        for name in self._names:
            future = self._controller_futures[name]
            if future is not None and future.done():
                try:
                    response = future.result()
                    active = {
                        controller.name for controller in response.controller
                        if controller.state == 'active'
                    }
                    self._controllers_active[name] = required <= active
                except Exception as exception:  # service may restart at spawn
                    self.get_logger().debug(
                        f'{name} controller query failed: {exception}')
                    self._controllers_active[name] = False
                self._controller_futures[name] = None
                self._controller_request_times[name] = now
                continue
            if future is not None:
                if now - self._controller_request_times[name] > 5.0:
                    future.cancel()
                    self._controller_futures[name] = None
                continue
            if (now - self._controller_request_times[name] < 1.0
                    or not self._controller_clients[name].service_is_ready()):
                continue
            self._controller_futures[name] = self._controller_clients[
                name].call_async(ListControllers.Request())
            self._controller_request_times[name] = now

    def _on_start(
            self, _request: Trigger.Request,
            response: Trigger.Response) -> Trigger.Response:
        if self._state in (
                'TRANSLATING', 'TRANSLATION_SETTLING', 'ROTATING', 'SETTLING',
                'POST_ROTATION_TRANSLATING', 'POST_ROTATION_SETTLING'):
            response.success = False
            response.message = f'Rotation already active; state={self._state}.'
            return response
        reason = self._ready_reason(require_pre_rotation_stability=True)
        if reason is not None:
            response.success = False
            response.message = reason
            return response
        self._start_requested = True
        self._completion_published = False
        if self._state in ('FINISHED', 'ERROR'):
            self._state = 'GRASP_CHECK'
            self._ready_since = None
        response.success = True
        response.message = 'Rotation start accepted.'
        return response

    def _on_cancel(
            self, _request: Trigger.Request,
            response: Trigger.Response) -> Trigger.Response:
        if self._state not in (
                'INITIALIZE', 'TRANSLATING', 'TRANSLATION_SETTLING',
                'ROTATING', 'SETTLING', 'POST_ROTATION_TRANSLATING',
                'POST_ROTATION_SETTLING'):
            response.success = False
            response.message = f'No active rotation; state={self._state}.'
            return response
        self._set_transport_enabled(False)
        self._state = 'WAITING'
        self._status_detail = 'cancelled by operator'
        self._start_requested = False
        self._ready_since = None
        response.success = True
        response.message = 'Rotation cancelled and transport stopped.'
        return response

    def _capture_rotation_reference(self) -> bool:
        """Capture the physical object/gripper relation for the next rotation."""
        current_object = self._current_object()
        if current_object is None:
            self._enter_error('payload pose unavailable while capturing reference')
            return False
        self._object_initial = current_object
        self._object_to_gripper = {}
        for name in self._names:
            gripper = self._current_gripper(name)
            if gripper is None:
                self._enter_error(f'{name} gripper transform unavailable')
                return False
            self._object_to_gripper[name] = (
                invert_transform(current_object) @ gripper)
        if bool(self.get_parameter('axis_point_at_object_center').value):
            self._axis_point = current_object[:3, 3].copy()
        else:
            point = np.asarray(self.get_parameter('axis_point').value,
                               dtype=float)
            if point.shape != (3,):
                self._enter_error('axis_point must contain three values')
                return False
            self._axis_point = point
        return True

    def _initialize_rotation(self) -> bool:
        """Capture the initial payload pose and prepare its straight approach."""
        if abs(self._axis[2]) < 1.0 - 1.0e-6:
            self._enter_error(
                'current AMIR base command path supports a world-Z axis only')
            return False
        if not self._capture_rotation_reference():
            return False
        assert self._object_initial is not None
        direction = np.array([
            self._object_initial[0, 0], self._object_initial[1, 0], 0.0])
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm <= 1.0e-9:
            self._enter_error('payload transport-frame +X is vertical')
            return False
        direction /= direction_norm
        self._translation_target = self._object_initial.copy()
        self._translation_target[:3, 3] += (
            direction * self._translation_trajectory.target_angle)
        self._motion_started = time.monotonic()
        self._translation_elapsed = 0.0
        self._translation_within_tolerance_since = None
        self._translation_clock_time = (
            self.get_clock().now().nanoseconds * 1.0e-9)
        self._trajectory_elapsed = 0.0
        self._trajectory_clock_time = None
        self._tracking_governor.paused = False
        self._translation_tracking_governor.paused = False
        self._theta_desired = 0.0
        self._theta_actual = 0.0
        self._within_tolerance_since = None
        self._completion_published = False
        self.get_logger().info(
            'シーケンス開始: 搬送（直進） '
            f'（{self._translation_trajectory.target_angle:.3f} m）')
        return True

    def _begin_rotation(self) -> bool:
        """Recapture the grasp at the translated pose, then start rotation."""
        if not self._capture_rotation_reference():
            return False
        self._trajectory_elapsed = 0.0
        self._trajectory_clock_time = (
            self.get_clock().now().nanoseconds * 1.0e-9)
        self._tracking_governor.paused = False
        self._theta_desired = 0.0
        self._theta_actual = 0.0
        self._within_tolerance_since = None
        self.get_logger().info(
            'シーケンス終了: 搬送（直進）')
        self.get_logger().info('シーケンス開始: 搬送（回転）')
        return True

    def _begin_post_rotation_translation(self) -> bool:
        """Capture the completed turn and prepare world-+Y transport."""
        if not self._capture_rotation_reference():
            return False
        assert self._object_initial is not None
        self._post_rotation_translation_target = self._object_initial.copy()
        self._post_rotation_translation_target[1, 3] += (
            self._post_rotation_translation_trajectory.target_angle)
        self._post_rotation_translation_elapsed = 0.0
        self._post_rotation_translation_clock_time = (
            self.get_clock().now().nanoseconds * 1.0e-9)
        self._post_rotation_within_tolerance_since = None
        self._translation_tracking_governor.paused = False
        self.get_logger().info(
            'シーケンス終了: 搬送（回転）')
        self.get_logger().info(
            'シーケンス開始: 搬送（Y方向直進） '
            f'{self._post_rotation_translation_trajectory.target_angle:.3f} m '
            '（world +Y）')
        return True

    def _tracking_errors(
            self, current: np.ndarray, target: np.ndarray
    ) -> Tuple[float, float]:
        position = float(np.linalg.norm(current[:3, 3] - target[:3, 3]))
        orientation = rotation_error(current[:3, :3], target[:3, :3])
        return position, orientation

    def _command_angle(
            self, angle: float,
            transport_angle: Optional[float] = None) -> None:
        assert self._object_initial is not None
        assert self._axis_point is not None
        target = rotate_transform_about_axis(
            self._object_initial, self._axis_point, self._axis, angle)
        if transport_angle is None:
            transport_target = target
        else:
            transport_target = rotate_transform_about_axis(
                self._object_initial, self._axis_point, self._axis,
                transport_angle)
        self._command_transform(target, transport_target)

    def _command_transform(
            self, target: np.ndarray,
            transport_target: Optional[np.ndarray] = None) -> None:
        """Command a rigid payload target and its matching TCP targets."""
        self._object_target = target
        self._publish_pose(self._object_target_publisher, target)
        if transport_target is None:
            transport_target = target
        self._publish_pose(self._goal_publisher, transport_target)
        for name in self._names:
            gripper_target = target @ self._object_to_gripper[name]
            self._publish_pose(
                self._gripper_target_publishers[name], gripper_target)
        self._set_transport_enabled(True)
        self._publish_markers(target)

    def _control_loop(self) -> None:
        now = time.monotonic()
        self._poll_controllers(now)
        reason = self._ready_reason(
            require_pre_rotation_stability=(self._state == 'GRASP_CHECK'))

        if self._state == 'WAITING':
            self._status_detail = reason or 'robot poses and TF are ready'
            if self._poses_fresh():
                self._state = 'GRASP_CHECK'

        elif self._state == 'GRASP_CHECK':
            self._status_detail = reason or 'ready to initialize'
            if reason is None:
                if self._ready_since is None:
                    self._ready_since = now
                auto_start = bool(self.get_parameter('auto_start').value)
                auto_delay = float(
                    self.get_parameter('auto_start_delay').value)
                auto_ready = (
                    auto_start and now - self._ready_since >= auto_delay)
                if self._start_requested or auto_ready:
                    self._state = 'INITIALIZE'
            else:
                self._ready_since = None

        elif self._state == 'INITIALIZE':
            if self._initialize_rotation():
                if self._translation_trajectory.complete(0.0):
                    if self._begin_rotation():
                        self._state = 'ROTATING'
                        self._status_detail = 'synchronized rotation in progress'
                else:
                    self._state = 'TRANSLATING'
                    self._status_detail = 'synchronized straight transport in progress'

        elif self._state == 'TRANSLATING':
            if reason is not None:
                self._enter_error(reason)
            else:
                assert self._object_initial is not None
                assert self._translation_target is not None
                assert self._translation_clock_time is not None
                trajectory_now = (
                    self.get_clock().now().nanoseconds * 1.0e-9)
                trajectory_dt = max(0.0, min(
                    0.1, trajectory_now - self._translation_clock_time))
                self._translation_clock_time = trajectory_now
                current = self._current_object()
                prior_position_error = 0.0
                if current is not None and self._object_target is not None:
                    prior_position_error, _ = self._tracking_errors(
                        current, self._object_target)
                if self._translation_tracking_governor.allow_progress(
                        prior_position_error):
                    self._translation_elapsed += trajectory_dt
                    self._status_detail = (
                        'synchronized straight transport in progress')
                else:
                    self._status_detail = (
                        'straight reference paused while tracking catches up')
                distance = self._translation_trajectory.sample(
                    self._translation_elapsed)
                total_distance = self._translation_trajectory.target_angle
                target = self._object_initial.copy()
                if abs(total_distance) > 1.0e-12:
                    ratio = distance / total_distance
                    target[:3, 3] += ratio * (
                        self._translation_target[:3, 3]
                        - self._object_initial[:3, 3])
                self._command_transform(target)
                current = self._current_object()
                if current is not None:
                    position_error, orientation_error = self._tracking_errors(
                        current, self._object_target)
                    if (position_error > float(self.get_parameter(
                            'maximum_tracking_position_error').value)
                            or orientation_error > math.radians(float(
                                self.get_parameter(
                                    'maximum_tracking_orientation_error_deg')
                                .value))):
                        self._enter_error(
                            'translation tracking error exceeded limit: '
                            f'{position_error:.3f} m, '
                            f'{math.degrees(orientation_error):.1f} deg')
                if (self._state == 'TRANSLATING'
                        and self._translation_trajectory.complete(
                            self._translation_elapsed)):
                    self._state = 'TRANSLATION_SETTLING'
                    self._translation_settling_started = trajectory_now
                    self._translation_within_tolerance_since = None
                    self._status_detail = (
                        'holding 1 m translation target before rotation')

        elif self._state == 'TRANSLATION_SETTLING':
            if reason is not None:
                self._enter_error(reason)
            else:
                assert self._translation_target is not None
                self._command_transform(self._translation_target)
                settling_now = (
                    self.get_clock().now().nanoseconds * 1.0e-9)
                current = self._current_object()
                position_error = math.inf
                if current is not None and self._object_target is not None:
                    position_error, orientation_error = self._tracking_errors(
                        current, self._object_target)
                    if orientation_error > math.radians(float(
                            self.get_parameter(
                                'maximum_tracking_orientation_error_deg').value)):
                        self._enter_error(
                            'translation changed payload orientation by '
                            f'{math.degrees(orientation_error):.1f} deg')
                if self._state == 'TRANSLATION_SETTLING':
                    position_ready = position_error <= float(
                        self.get_parameter('position_tolerance').value)
                    if position_ready:
                        if self._translation_within_tolerance_since is None:
                            self._translation_within_tolerance_since = settling_now
                    else:
                        self._translation_within_tolerance_since = None
                    if self._translation_within_tolerance_since is None:
                        self._status_detail = (
                            'waiting for 1 m translation target to settle')
                    elif (settling_now
                          - self._translation_within_tolerance_since >= float(
                              self.get_parameter(
                                  'translation_settling_time').value)):
                        if self._begin_rotation():
                            self._state = 'ROTATING'
                            self._status_detail = (
                                'straight transport complete; rotation in progress')
                if (self._state == 'TRANSLATION_SETTLING'
                        and settling_now - self._translation_settling_started
                        > float(self.get_parameter(
                            'translation_settling_timeout').value)):
                    self._enter_error('straight-transport settling timeout')

        elif self._state == 'ROTATING':
            if reason is not None:
                self._enter_error(reason)
            else:
                # Advance the reference in ROS time so use_sim_time follows
                # Gazebo even when its real-time factor drops. If the payload
                # falls behind, freeze the reference and let the closed-loop
                # coordinator catch up before proceeding.
                trajectory_now = (
                    self.get_clock().now().nanoseconds * 1.0e-9)
                trajectory_dt = max(0.0, min(
                    0.1, trajectory_now - self._trajectory_clock_time))
                self._trajectory_clock_time = trajectory_now
                current = self._current_object()
                prior_orientation_error = 0.0
                if current is not None and self._object_target is not None:
                    _, prior_orientation_error = self._tracking_errors(
                        current, self._object_target)
                if self._tracking_governor.allow_progress(
                        prior_orientation_error):
                    self._trajectory_elapsed += trajectory_dt
                    self._status_detail = 'synchronized rotation in progress'
                else:
                    self._status_detail = (
                        'reference paused while tracking catches up')
                self._theta_desired = self._trajectory.sample(
                    self._trajectory_elapsed)
                self._command_angle(self._theta_desired)
                current = self._current_object()
                if current is not None and self._object_initial is not None:
                    self._theta_actual = signed_axis_rotation(
                        self._object_initial[:3, :3], current[:3, :3],
                        self._axis)
                    position_error, orientation_error = self._tracking_errors(
                        current, self._object_target)
                    if (position_error > float(self.get_parameter(
                            'maximum_tracking_position_error').value)
                            or orientation_error > math.radians(float(
                                self.get_parameter(
                                    'maximum_tracking_orientation_error_deg')
                                .value))):
                        self._enter_error(
                            'tracking error exceeded limit: '
                            f'{position_error:.3f} m, '
                            f'{math.degrees(orientation_error):.1f} deg')
                if (self._state == 'ROTATING'
                        and self._trajectory.complete(
                            self._trajectory_elapsed)):
                    self._state = 'SETTLING'
                    self._settling_started = trajectory_now
                    self._status_detail = 'holding final target while settling'

        elif self._state == 'SETTLING':
            if reason is not None:
                self._enter_error(reason)
            else:
                self._theta_desired = self._target_angle
                current = self._current_object()
                if current is not None:
                    self._theta_actual = signed_axis_rotation(
                        self._object_initial[:3, :3], current[:3, :3],
                        self._axis)
                transport_angle = settling_transport_angle(
                    self._target_angle,
                    self._theta_actual,
                    float(self.get_parameter(
                        'settling_goal_correction_gain').value),
                    math.radians(float(self.get_parameter(
                        'settling_goal_max_correction_deg').value)),
                )
                self._command_angle(self._target_angle, transport_angle)
                settling_now = (
                    self.get_clock().now().nanoseconds * 1.0e-9)
                if current is not None and self._object_target is not None:
                    position_error, orientation_error = self._tracking_errors(
                        current, self._object_target)
                    within = (
                        position_error <= float(self.get_parameter(
                            'position_tolerance').value)
                        and orientation_error <= math.radians(float(
                            self.get_parameter('orientation_tolerance_deg')
                            .value)))
                    if within:
                        if self._within_tolerance_since is None:
                            self._within_tolerance_since = settling_now
                        if settling_now - self._within_tolerance_since >= float(
                                self.get_parameter('settling_time').value):
                            if self._post_rotation_translation_trajectory.complete(
                                    0.0):
                                self._state = 'FINISHED'
                                self._status_detail = (
                                    'target reached and final pose is held')
                                self.get_logger().info(
                                    'シーケンス終了: 搬送（回転）')
                                self.get_logger().info(
                                    'シーケンス開始: 搬送完了')
                            elif self._begin_post_rotation_translation():
                                self._state = 'POST_ROTATION_TRANSLATING'
                                self._status_detail = (
                                    'post-rotation world-+Y transport in progress')
                    else:
                        self._within_tolerance_since = None
                if (self._state == 'SETTLING'
                        and settling_now - self._settling_started > float(
                            self.get_parameter('settling_timeout').value)):
                    self._enter_error('settling timeout')

        elif self._state == 'POST_ROTATION_TRANSLATING':
            if reason is not None:
                self._enter_error(reason)
            else:
                assert self._object_initial is not None
                assert self._post_rotation_translation_target is not None
                assert self._post_rotation_translation_clock_time is not None
                trajectory_now = (
                    self.get_clock().now().nanoseconds * 1.0e-9)
                trajectory_dt = max(0.0, min(
                    0.1, trajectory_now
                    - self._post_rotation_translation_clock_time))
                self._post_rotation_translation_clock_time = trajectory_now
                current = self._current_object()
                prior_position_error = 0.0
                if current is not None and self._object_target is not None:
                    prior_position_error, _ = self._tracking_errors(
                        current, self._object_target)
                if self._translation_tracking_governor.allow_progress(
                        prior_position_error):
                    self._post_rotation_translation_elapsed += trajectory_dt
                    self._status_detail = (
                        'post-rotation world-+Y transport in progress')
                else:
                    self._status_detail = (
                        'world-+Y reference paused while tracking catches up')
                distance = self._post_rotation_translation_trajectory.sample(
                    self._post_rotation_translation_elapsed)
                total_distance = (
                    self._post_rotation_translation_trajectory.target_angle)
                target = self._object_initial.copy()
                if abs(total_distance) > 1.0e-12:
                    ratio = distance / total_distance
                    target[:3, 3] += ratio * (
                        self._post_rotation_translation_target[:3, 3]
                        - self._object_initial[:3, 3])
                self._command_transform(target)
                current = self._current_object()
                if current is not None:
                    position_error, orientation_error = self._tracking_errors(
                        current, self._object_target)
                    if (position_error > float(self.get_parameter(
                            'maximum_tracking_position_error').value)
                            or orientation_error > math.radians(float(
                                self.get_parameter(
                                    'maximum_tracking_orientation_error_deg')
                                .value))):
                        self._enter_error(
                            'post-rotation translation tracking error exceeded '
                            f'limit: {position_error:.3f} m, '
                            f'{math.degrees(orientation_error):.1f} deg')
                if (self._state == 'POST_ROTATION_TRANSLATING'
                        and self._post_rotation_translation_trajectory.complete(
                            self._post_rotation_translation_elapsed)):
                    self._state = 'POST_ROTATION_SETTLING'
                    self._post_rotation_settling_started = trajectory_now
                    self._post_rotation_within_tolerance_since = None
                    self._status_detail = (
                        'holding world-+Y translation target before completion')

        elif self._state == 'POST_ROTATION_SETTLING':
            if reason is not None:
                self._enter_error(reason)
            else:
                assert self._post_rotation_translation_target is not None
                self._command_transform(self._post_rotation_translation_target)
                settling_now = (
                    self.get_clock().now().nanoseconds * 1.0e-9)
                current = self._current_object()
                position_error = math.inf
                if current is not None and self._object_target is not None:
                    position_error, orientation_error = self._tracking_errors(
                        current, self._object_target)
                    if orientation_error > math.radians(float(
                            self.get_parameter(
                                'maximum_tracking_orientation_error_deg').value)):
                        self._enter_error(
                            'world-+Y translation changed payload orientation by '
                            f'{math.degrees(orientation_error):.1f} deg')
                if self._state == 'POST_ROTATION_SETTLING':
                    if position_error <= float(
                            self.get_parameter('position_tolerance').value):
                        if self._post_rotation_within_tolerance_since is None:
                            self._post_rotation_within_tolerance_since = settling_now
                    else:
                        self._post_rotation_within_tolerance_since = None
                    if self._post_rotation_within_tolerance_since is None:
                        self._status_detail = (
                            'waiting for world-+Y translation target to settle')
                    elif (settling_now
                          - self._post_rotation_within_tolerance_since >= float(
                              self.get_parameter(
                                  'translation_settling_time').value)):
                        self._state = 'FINISHED'
                        self._status_detail = (
                            'world-+Y target reached and final pose is held')
                        self.get_logger().info(
                            'シーケンス終了: 搬送（Y方向直進）')
                        self.get_logger().info('シーケンス開始: 搬送完了')
                if (self._state == 'POST_ROTATION_SETTLING'
                        and settling_now - self._post_rotation_settling_started
                        > float(self.get_parameter(
                            'translation_settling_timeout').value)):
                    self._enter_error(
                        'post-rotation straight-transport settling timeout')

        elif self._state == 'FINISHED':
            if not self._completion_published:
                message = Bool()
                message.data = True
                self._complete_publisher.publish(message)
                self._completion_published = True

        self._publish_status()

    def _enter_error(self, detail: str) -> None:
        if self._state == 'ERROR':
            return
        self._state = 'ERROR'
        self._status_detail = detail
        self._set_transport_enabled(False)
        self.get_logger().error(f'Rotation stopped: {detail}')

    def _set_transport_enabled(self, enabled: bool) -> None:
        message = Bool()
        message.data = enabled
        self._enable_publisher.publish(message)

    def _publish_pose(self, publisher, transform: np.ndarray) -> None:
        translation, quaternion = transform_components(transform)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._world_frame
        message.pose.position.x = float(translation[0])
        message.pose.position.y = float(translation[1])
        message.pose.position.z = float(translation[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        publisher.publish(message)

    def _publish_status(self) -> None:
        state = String()
        state.data = self._state
        self._state_publisher.publish(state)
        status = String()
        status.data = (
            f'{self._state}: {self._status_detail}; '
            f'theta_des={math.degrees(self._theta_desired):.2f} deg; '
            f'theta_actual={math.degrees(self._theta_actual):.2f} deg')
        self._status_publisher.publish(status)
        angle = Float64()
        angle.data = math.degrees(self._theta_actual)
        self._angle_publisher.publish(angle)
        desired = Float64()
        desired.data = math.degrees(self._theta_desired)
        self._desired_angle_publisher.publish(desired)

    def _marker(
            self, marker_id: int, namespace: str, marker_type: int,
            transform: np.ndarray, color: Tuple[float, float, float],
            scale: Tuple[float, float, float]) -> Marker:
        translation, quaternion = transform_components(transform)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._world_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(translation[0])
        marker.pose.position.y = float(translation[1])
        marker.pose.position.z = float(translation[2])
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.85
        marker.lifetime = Duration(seconds=0.2).to_msg()
        return marker

    def _publish_markers(self, object_target: np.ndarray) -> None:
        if not bool(self.get_parameter('publish_markers').value):
            return
        markers = MarkerArray()
        markers.markers.append(self._marker(
            0, 'object_target', Marker.CUBE, object_target,
            (0.0, 1.0, 0.0), (1.2, 0.08, 0.08)))
        colors = ((1.0, 0.0, 0.0), (0.0, 0.2, 1.0))
        for index, (name, color) in enumerate(zip(self._names, colors), 1):
            target = object_target @ self._object_to_gripper[name]
            markers.markers.append(self._marker(
                index, f'{name}_gripper_target', Marker.ARROW, target,
                color, (0.18, 0.03, 0.03)))
        axis_transform = np.eye(4)
        axis_transform[:3, 3] = self._axis_point
        z_axis = np.array([0.0, 0.0, 1.0])
        cross = np.cross(z_axis, self._axis)
        dot = float(np.dot(z_axis, self._axis))
        if np.linalg.norm(cross) > 1.0e-9:
            axis_transform[:3, :3] = rotate_transform_about_axis(
                np.eye(4), [0.0, 0.0, 0.0], cross,
                math.acos(max(-1.0, min(1.0, dot))))[:3, :3]
        elif dot < 0.0:
            axis_transform[:3, :3] = rotate_transform_about_axis(
                np.eye(4), [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                math.pi)[:3, :3]
        markers.markers.append(self._marker(
            3, 'rotation_axis', Marker.CYLINDER, axis_transform,
            (1.0, 0.85, 0.0), (0.025, 0.025, 0.8)))
        self._marker_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CooperativeRotationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._set_transport_enabled(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
