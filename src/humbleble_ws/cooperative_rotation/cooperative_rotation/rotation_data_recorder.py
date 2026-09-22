"""Record cooperative rotation angles and grasp forces to Excel."""

from datetime import datetime
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from .excel_report import RotationSample, write_rotation_workbook
from .transform_utils import quaternion_to_rpy


class RotationDataRecorder(Node):
    """Synchronously sample desired/actual angles and four finger wrenches."""

    def __init__(self) -> None:
        super().__init__('cooperative_rotation_data_recorder')
        self.declare_parameter('target_angle_deg', 90.0)
        self.declare_parameter('sample_rate', 10.0)
        self.declare_parameter(
            'output_directory', '~/ros2_humble_ws/rotation_measurements')
        self.declare_parameter('output_file', '')

        self._target_angle = float(
            self.get_parameter('target_angle_deg').value)
        self._desired_angle = 0.0
        self._actual_angle = 0.0
        self._state = 'WAITING'
        self._grasp_state = 'WAITING_FOR_STATUS'
        self._support_removed: Optional[bool] = None
        self._payload_center_xy: Optional[Tuple[float, float]] = None
        self._payload_rpy_deg: Optional[Tuple[float, float, float]] = None
        self._forces: Dict[str, Tuple[float, float, float]] = {}
        self._calibrated_forces: Dict[str, Tuple[float, float, float]] = {}
        self._samples: List[RotationSample] = []
        # Start at recorder creation, rather than at the ROTATING transition.
        # This makes the report cover simulator startup, gripper closing,
        # lifting, support removal, and the rotation itself.
        self._recording_started: Optional[float] = self._now()
        self._recording_complete = False
        self._saved_path: Optional[Path] = None
        self._saved_sample_count = 0

        self.create_subscription(
            Float64, '/cooperative_rotation/desired_angle_deg',
            lambda message: setattr(self, '_desired_angle', message.data), 20)
        self.create_subscription(
            Float64, '/cooperative_rotation/angle_deg',
            lambda message: setattr(self, '_actual_angle', message.data), 20)
        self.create_subscription(
            String, '/cooperative_rotation/state', self._on_state, 20)
        self.create_subscription(
            String, '/cooperative_transport/friction_grasp_status',
            self._on_grasp_status, 20)
        self.create_subscription(
            PoseStamped, '/cooperative_transport/measured_payload_pose',
            self._on_payload_pose, 20)
        support_qos = QoSProfile(depth=1)
        support_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_transport/support_removed',
            self._on_support_removed, support_qos)
        for robot in ('amir1', 'amir2'):
            for side in ('left', 'right'):
                point = f'{robot}_{side}'
                self.create_subscription(
                    WrenchStamped, f'/{robot}/finger_{side}_wrench',
                    lambda message, key=point: self._on_force(key, message), 20)
                self.create_subscription(
                    WrenchStamped,
                    f'/{robot}/finger_{side}_wrench_calibrated',
                    lambda message, key=point:
                    self._on_calibrated_force(key, message), 20)

        result_qos = QoSProfile(depth=1)
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/cooperative_rotation/complete', self._on_complete,
            result_qos)
        self._report_publisher = self.create_publisher(
            String, '/cooperative_rotation/excel_report', result_qos)
        self.create_service(
            Trigger, '/cooperative_rotation/save_excel', self._on_save)

        sample_rate = float(self.get_parameter('sample_rate').value)
        if not math.isfinite(sample_rate) or sample_rate <= 0.0:
            raise ValueError('sample_rate must be positive and finite.')
        self.create_timer(1.0 / sample_rate, self._sample)
        self.get_logger().info(
            'Rotation Excel recorder ready; recording starts at node startup.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_force(self, point: str, message: WrenchStamped) -> None:
        force = message.wrench.force
        self._forces[point] = (force.x, force.y, force.z)

    def _on_calibrated_force(
            self, point: str, message: WrenchStamped) -> None:
        force = message.wrench.force
        self._calibrated_forces[point] = (force.x, force.y, force.z)

    def _on_state(self, message: String) -> None:
        self._state = message.data
        if (message.data in ('FINISHED', 'ERROR')
                and self._recording_started is not None
                and not self._recording_complete):
            self._sample()
            self._recording_complete = True
            self.save()

    def _on_grasp_status(self, message: String) -> None:
        # The manager appends diagnostics after a semicolon; retain its state.
        self._grasp_state = message.data.split(';', 1)[0].strip()

    def _on_support_removed(self, message: Bool) -> None:
        self._support_removed = message.data

    def _on_payload_pose(self, message: PoseStamped) -> None:
        """Keep the latest measured payload centre and world-frame attitude."""
        pose = message.pose
        orientation = pose.orientation
        roll, pitch, yaw = quaternion_to_rpy((
            orientation.x, orientation.y, orientation.z, orientation.w))
        self._payload_center_xy = (pose.position.x, pose.position.y)
        self._payload_rpy_deg = tuple(
            math.degrees(value) for value in (roll, pitch, yaw))

    def _sequence(self) -> str:
        """Return the user-facing phase represented by current ROS states."""
        if self._state == 'ERROR':
            return '異常終了'
        if self._state == 'FINISHED':
            return '搬送完了'
        if self._state in ('TRANSLATING', 'TRANSLATION_SETTLING'):
            return '搬送（直進）'
        if self._state in (
                'POST_ROTATION_TRANSLATING', 'POST_ROTATION_SETTLING'):
            return '搬送（Y方向直進）'
        if self._state in ('ROTATING', 'SETTLING'):
            return '搬送（回転）'
        if self._support_removed is True:
            return '把持完了・搬送準備'
        if self._grasp_state in (
                'CLOSING_GRIPPERS', 'LIFTING', 'HOLDING'):
            return '把持'
        return 'ロボット・物体生成'

    def _on_complete(self, message: Bool) -> None:
        if (message.data and self._recording_started is not None
                and not self._recording_complete):
            self._sample()
            self._recording_complete = True
            self.save()

    def _sample(self) -> None:
        if self._recording_started is None or self._recording_complete:
            return
        elapsed = max(0.0, self._now() - self._recording_started)
        sequence = self._sequence()
        previous_sequence = getattr(self, '_last_sequence', None)
        sequence_event = (
            f'開始: {sequence}' if sequence != previous_sequence else '')
        self._last_sequence = sequence
        self._samples.append(RotationSample(
            elapsed=elapsed,
            state=self._state,
            final_target_deg=self._target_angle,
            desired_deg=self._desired_angle,
            actual_deg=self._actual_angle,
            forces=dict(self._forces),
            calibrated_forces=dict(self._calibrated_forces),
            sequence=sequence,
            sequence_event=sequence_event,
            grasp_state=self._grasp_state,
            support_removed=self._support_removed,
            payload_center_x=(self._payload_center_xy[0]
                              if self._payload_center_xy is not None else None),
            payload_center_y=(self._payload_center_xy[1]
                              if self._payload_center_xy is not None else None),
            payload_roll_deg=(self._payload_rpy_deg[0]
                              if self._payload_rpy_deg is not None else None),
            payload_pitch_deg=(self._payload_rpy_deg[1]
                               if self._payload_rpy_deg is not None else None),
            payload_yaw_deg=(self._payload_rpy_deg[2]
                             if self._payload_rpy_deg is not None else None),
        ))

    def _output_path(self) -> Path:
        configured_file = str(self.get_parameter('output_file').value).strip()
        if configured_file:
            return Path(configured_file).expanduser()
        directory = Path(str(
            self.get_parameter('output_directory').value)).expanduser()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        angle = f'{self._target_angle:g}'.replace('-', 'minus_').replace('.', 'p')
        return directory / f'rotation_{angle}deg_{timestamp}.xlsx'

    def save(self) -> Optional[Path]:
        """Write all collected samples and publish the resulting path."""
        if not self._samples:
            return None
        if self._saved_path is None:
            self._saved_path = self._output_path().resolve()
        # Completion and node shutdown can both call save().  Avoid rebuilding
        # the whole XLSX when no samples were added since the previous save.
        if (self._saved_path.exists()
                and self._saved_sample_count == len(self._samples)):
            return self._saved_path
        write_rotation_workbook(self._saved_path, self._samples)
        self._saved_sample_count = len(self._samples)
        if rclpy.ok():
            report = String()
            report.data = str(self._saved_path)
            self._report_publisher.publish(report)
            self.get_logger().info(
                f'Rotation Excel report saved: {self._saved_path}')
        return self._saved_path

    def _on_save(
            self, _request: Trigger.Request,
            response: Trigger.Response) -> Trigger.Response:
        path = self.save()
        response.success = path is not None
        response.message = (
            str(path) if path is not None else 'No rotation samples recorded yet.')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotationDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
