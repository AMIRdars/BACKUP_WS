"""Bounded read-only capture of Fortress GT and leader encoder source times.

The installed Pose_V -> TFMessage bridge loses the vector header timestamp.
Subscribe to original Pose_V via `ign topic --echo --json-output` and to the
existing leader encoder Odometry. Neither stream is republished to control.
"""
import argparse
from collections import deque
import csv
import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import time

import numpy as np

from amir_rotation_center_attribution import transform


ROBOTS = ('leader', 'follower', 'cooperative_payload')
TOPIC = '/world/cooperative_transport/pose/info'
ODOM = '/leader/mecanum_drive_controller/odometry'


def stamp_ns(sec, nsec):
    def integer(value):
        if isinstance(value, bool) or not str(value).isdigit():
            raise ValueError('nonnegative integral source stamp required')
        return int(value)
    sec, nsec = integer(sec), integer(nsec)
    if nsec >= 10**9:
        raise ValueError('nanoseconds outside source stamp range')
    return sec*10**9+nsec


def matrix(position, orientation):
    # Protobuf JSON omits scalar fields with their declared zero default.
    xyz = np.array([position.get(k, 0.) for k in 'xyz'], dtype=float)
    x, y, z, w = [float(orientation.get(k, 0.)) for k in 'xyzw']
    if not np.isfinite(xyz).all() or not np.isfinite([x, y, z, w]).all() or abs(x*x+y*y+z*z+w*w-1.) > 1e-6:
        raise ValueError('finite position and unit source quaternion required')
    result = np.eye(4)
    result[:3, :3] = [[1.-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1.-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1.-2*(x*x+y*y)]]
    result[:3, 3] = xyz
    return transform(result)


def parse_gt(message):
    header = message['header']['stamp']
    stamp = stamp_ns(header.get('sec', 0), header.get('nsec', 0))
    if stamp <= 0:
        raise ValueError('nonzero original Pose_V source stamp required')
    found = {}
    for item in message['pose']:
        name = item.get('name')
        if name in ROBOTS:
            if name in found:
                raise ValueError('duplicate evaluated model name')
            found[name] = matrix(item['position'], item['orientation']).tolist()
    if set(found) != set(ROBOTS):
        raise ValueError('all three evaluated poses must share one Pose_V stamp')
    return dict(gt_source_time_ns=stamp, gt_source_time_s=stamp*1e-9, poses=found)


def match_odom(history, gt_stamp):
    """Use previous own source sample, within existing 20ms source skew bound."""
    previous = [sample for sample in history if 0 <= gt_stamp-sample['stamp_ns'] <= 20_000_000]
    return max(previous, key=lambda sample: sample['stamp_ns']) if previous else None


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except (OSError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--owner-pid', type=int, required=True)
    parser.add_argument('--partition', required=True)
    parser.add_argument('--wall-duration-s', type=float, default=30.)
    args = parser.parse_args()
    if (os.getenv('ROS_DISTRO') != 'humble' or os.getenv('ROS_DOMAIN_ID') != '207'
            or os.getenv('ROS_LOCALHOST_ONLY') != '1'
            or not args.partition.startswith('amir_cp8r3_ch23_equivariant_')
            or not math.isfinite(args.wall_duration_s) or not 0. < args.wall_duration_s <= 60.):
        parser.error('isolated existing fixed trial and bounded read-only capture required')
    owner_start = identity(args.owner_pid)
    if owner_start is None or b'sweep.py' not in Path(f'/proc/{args.owner_pid}/cmdline').read_bytes():
        parser.error('live fixed-sweep owner required')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    rclpy.init(args=[])
    node = rclpy.create_node('rotation_source_time_evaluator', namespace='/experiment')
    history = deque(maxlen=256)
    odom_errors = []

    def receive(msg):
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            history.append(dict(stamp_ns=stamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec),
                frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id,
                pose=matrix(dict(x=p.x, y=p.y, z=p.z), dict(x=q.x, y=q.y, z=q.z, w=q.w)).tolist()))
        except (ValueError, KeyError) as exc:
            odom_errors.append(str(exc))

    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscription = node.create_subscription(Odometry, ODOM, receive, qos)
    environment = dict(os.environ, IGN_PARTITION=args.partition)
    command = ['ign', 'topic', '-t', TOPIC, '-e', '--json-output']
    stderr = (output/'ign_topic_stderr.log').open('x')
    process = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE, stderr=stderr,
                               start_new_session=True)
    summary = dict(evaluation_only=True, used_by_control=False, physical_center_fix_achieved=False,
        full_program_evaluated=False, precision_qualified=False,
        purpose='bounded window during the already running trial; not a new motion test',
        ROS_DOMAIN_ID='207', IGN_PARTITION=args.partition, owner_pid=args.owner_pid, owner_start_id=owner_start,
        gt_topic=TOPIC, encoder_topic=ODOM, gt_transport='original Fortress Pose_V.header.stamp',
        encoder_transport='original ROS Odometry.header.stamp',
        command=command, started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        matched_samples=0, unmatched_samples=0, rejected_gt_records=0)
    pending = b''
    first, last = None, None
    next_record_stamp = 0
    maximum_skew = 0
    initial_alignment = None
    maximum_discrepancy = 0.
    deadline = time.monotonic()+args.wall_duration_s
    try:
        with (output/'source_times.jsonl').open('x') as stream, (output/'source_times.csv').open('x', newline='') as csv_stream:
            writer = None
            while time.monotonic() < deadline and identity(args.owner_pid) == owner_start:
                rclpy.spin_once(node, timeout_sec=.001)
                if process.poll() is not None:
                    raise RuntimeError('owned read-only ign echo process exited')
                ready, _, _ = select.select([process.stdout], [], [], .005)
                if not ready:
                    continue
                pending += os.read(process.stdout.fileno(), 262144)
                if len(pending) > 4*1024*1024:
                    raise ValueError('unexpected unbounded Pose_V JSON record')
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    if not line.strip():
                        continue
                    try:
                        gt = parse_gt(json.loads(line))
                    except (ValueError, KeyError):
                        summary['rejected_gt_records'] += 1
                        continue
                    stamp = gt['gt_source_time_ns']
                    if stamp < next_record_stamp:
                        continue
                    next_record_stamp = stamp+100_000_000  # 10 Hz evaluation sampling, not a control cap.
                    own = match_odom(history, stamp)
                    record = dict(**gt, received_monotonic_s=time.monotonic(), encoder=own)
                    stream.write(json.dumps(record)+'\n')
                    if own is None:
                        summary['unmatched_samples'] += 1
                        continue
                    summary['matched_samples'] += 1
                    skew = stamp-own['stamp_ns']
                    maximum_skew = max(maximum_skew, skew)
                    base, obj = [np.asarray(gt['poses'][name]) for name in ('leader', 'cooperative_payload')]
                    encoded = np.asarray(own['pose'])
                    if initial_alignment is None:
                        initial_alignment = base @ np.linalg.inv(encoded)
                    discrepancy = float(np.linalg.norm((initial_alignment@encoded)[:2, 3]-base[:2, 3]))
                    maximum_discrepancy = max(maximum_discrepancy, discrepancy)
                    value = dict(gt_source_time_ns=stamp, encoder_source_time_ns=own['stamp_ns'],
                        source_skew_ns=skew, object_x_m=obj[0, 3], object_y_m=obj[1, 3], object_z_m=obj[2, 3],
                        leader_GT_yaw_rad=math.atan2(base[1, 0], base[0, 0]),
                        leader_encoder_yaw_rad=math.atan2(encoded[1, 0], encoded[0, 0]),
                        leader_window_GT_vs_encoder_xy_m=discrepancy)
                    if writer is None:
                        writer = csv.DictWriter(csv_stream, fieldnames=list(value))
                        writer.writeheader()
                    writer.writerow(value)
                    if first is None:
                        first = value
                    last = value
        summary.update(capture_complete=summary['matched_samples'] >= 2,
            maximum_matched_source_skew_ns=maximum_skew,
            maximum_window_leader_GT_vs_encoder_xy_m=maximum_discrepancy,
            first_matched_record=first, last_matched_record=last)
        if first is not None and last is not None:
            change = lambda key: math.atan2(math.sin(last[key]-first[key]), math.cos(last[key]-first[key]))
            physical, own = change('leader_GT_yaw_rad'), change('leader_encoder_yaw_rad')
            summary.update(window_GT_yaw_change_rad=physical, window_encoder_yaw_change_rad=own,
                window_yaw_ratio_diagnostic_only=physical/own if own else None)
    except BaseException as exc:
        summary.update(capture_complete=False, error=repr(exc))
    finally:
        summary['encoder_decode_errors'] = odom_errors[:8]
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if process.poll() is not None:
                break
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                continue
        stderr.close()
        summary['echo_returncode'] = process.poll()
        (output/'capture_result.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary.get('capture_complete') else 1


if __name__ == '__main__':
    raise SystemExit(main())
