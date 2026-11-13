#!/usr/bin/env python3
"""Read saved trial logs; export measured joint posture and URDF FK plots.

Offline only: never starts ROS or changes a controller. TCP orientation is FK
reconstruction from measured joints, not an independent orientation measurement.
"""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/humbleble_ws/amir_compliance_control'))
from amir_compliance_control.wrench import LocalModel


def number(row, key):
    return float(row.get(key) or 'nan')


def vector(row, prefix, size):
    return [number(row, f'{prefix}_{i}') for i in range(size)]


def fk_batch(model, q):
    """Full base_footprint -> tcp_link transform, including all fixed joints."""
    t = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
    points = []
    index = 0
    for joint, fixed in zip(model.chain, model._origins):
        t = t @ fixed
        if joint.get('type') != 'fixed':
            if joint.get('type') != 'revolute':
                raise ValueError('unsupported moving joint')
            points.append(t[:, :3, 3].copy())
            moving = np.broadcast_to(np.eye(4), t.shape).copy()
            axis = model.axes[index]
            moving[:, :3, :3] = Rotation.from_rotvec(q[:, index, None] * axis).as_matrix()
            t = t @ moving
            index += 1
    points.append(t[:, :3, 3].copy())
    return t, np.stack(points, axis=1)


def write_csv(path, fields, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def read_case(case):
    arms = {robot: {} for robot in ('leader', 'follower')}
    observations = []
    with (case / 'timeseries.csv').open() as stream:
        for row in csv.DictReader(stream):
            time = float(row['sim_time'])
            enabled = row.get('admittance_enabled') == 'True'
            observation = {'time': time, 'phase': row['phase'], 'active': enabled}
            for robot in arms:
                stamp = number(row, robot + '_joint_stamp_s')
                q = vector(row, robot + '_q_actual', 5)
                if np.isfinite(stamp) and np.all(np.isfinite(q)):
                    # Repeated observer rows carry the same JointState sample.
                    arms[robot].setdefault(stamp, {
                        'stamp': stamp, 'receipt': time, 'q': q,
                        'phase': row['phase'], 'active': enabled,
                        'logged_p': vector(row, robot + '_p_actual', 3)})
                observation[robot] = {
                    'odom_stamp': number(row, robot + '_odom_stamp_s'),
                    'odom': vector(row, robot + '_odom_pose', 4),
                    'gt_receipt': number(row, robot + '_gt_receipt_sim_time'),
                    'gt_source': number(row, robot + '_gt_source_stamp_s'),
                    'gt_xyz': vector(row, robot + '_gt_position', 3),
                    'gt_yaw': number(row, robot + '_gt_yaw_rad')}
            observation['object_q'] = vector(row, 'payload_model_gt_orientation_xyzw', 4)
            observation['object_receipt'] = number(row, 'payload_model_gt_receipt_sim_time')
            observation['object_source'] = number(row, 'payload_model_gt_source_stamp_s')
            observations.append(observation)
    return arms, observations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, type=Path, help='trial directory with timeseries.csv')
    parser.add_argument('--output', required=True, type=Path, help='new output directory')
    args = parser.parse_args()
    case, out = args.case.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    raw, observations = read_case(case)
    active = [r for r in observations if r['active']]
    motion = [r for r in active if r['phase'] == 'leader_translation']
    scope = motion or active or observations
    start, end = scope[0]['time'], scope[-1]['time']
    epoch = active[0]['time'] if active else start
    result_name = 'held_result.json' if (case / 'held_result.json').exists() else 'result.json'
    result = json.loads((case / result_name).read_text())
    summary = {
        'case': str(case), 'source_csv_sha256': hashlib.sha256((case / 'timeseries.csv').read_bytes()).hexdigest(),
        'result_source': result_name, 'procedure_passed': result.get('passed'),
        'leader_completed': result.get('leader_completed', False), 'error': result.get('error'),
        'scope': 'leader_translation' if motion else ('active_admittance' if active else 'recorded_preparation'),
        'scope_sim_time_s': [start, end], 'plot_epoch_sim_time_s': epoch,
        'TCP_orientation_source': 'measured JointState q + generated URDF full TCP FK in own base_footprint',
        'base_orientation_available': 'recorded planar yaw only; roll/pitch and full quaternion were not saved',
        'arms': {}, 'artifacts': ['joints.png', 'tcp_base.png', 'arm_shapes.png', 'index.html']}
    data = {}
    colors = plt.cm.tab10.colors
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    shapes, shape_axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    posefig, pose_axes = plt.subplots(4, 1, figsize=(13, 13), sharex=True, constrained_layout=True)
    for column, (robot, samples_by_stamp) in enumerate(raw.items()):
        samples = sorted(samples_by_stamp.values(), key=lambda r: r['stamp'])
        if not samples:
            raise ValueError(f'no measured joint samples: {robot}')
        t = np.array([r['stamp'] for r in samples])
        receipt = np.array([r['receipt'] for r in samples])
        q = np.array([r['q'] for r in samples])
        urdf = case / 'generated' / (robot + '.urdf')
        model = LocalModel(urdf.read_text())
        transforms, points = fk_batch(model, q)
        p = transforms[:, :3, 3]
        rotations = Rotation.from_matrix(transforms[:, :3, :3])
        quat = rotations.as_quat()
        # TCPs can sit at pitch +/-90 degrees. Euler angles are non-unique
        # there; rotation vectors/angles below remain well-defined.
        baseline = max(0, int(np.searchsorted(receipt, epoch, side='right') - 1))
        change = (rotations * rotations[baseline].inv()).as_rotvec()
        angular_change = np.rad2deg(np.linalg.norm(change, axis=1))
        selected = np.flatnonzero((receipt >= start) & (receipt <= end))
        if not len(selected):
            raise ValueError(f'no joint samples in selected scope: {robot}')
        first, last = selected[0], selected[-1]
        margin = np.minimum(q - model.lower, model.upper - q)
        logged = np.array([r['logged_p'] for r in samples])
        valid = np.all(np.isfinite(logged), axis=1)
        residual = float(np.max(np.linalg.norm(p[valid] - logged[valid], axis=1))) if np.any(valid) else None
        if residual is not None and residual > 1e-9:
            raise ValueError(f'{robot} FK position mismatch: {residual}')
        info = {
            'URDF_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(), 'joint_names': list(model.names),
            'distinct_joint_samples': len(samples), 'scope_joint_stamp_s': [float(t[first]), float(t[last])],
            'joint_stamp_minus_observer_sim_range_s': [float(np.min(t-receipt)), float(np.max(t-receipt))],
            'joint_stamp_ahead_of_observer_count': int(np.sum(t-receipt > 1e-8)),
            'joint_start_deg': np.rad2deg(q[first]).tolist(), 'joint_end_deg': np.rad2deg(q[last]).tolist(),
            'joint_change_deg': np.rad2deg(q[last] - q[first]).tolist(),
            'joint_min_deg': np.rad2deg(np.min(q[selected], axis=0)).tolist(),
            'joint_max_deg': np.rad2deg(np.max(q[selected], axis=0)).tolist(),
            'minimum_URDF_margin_deg': float(np.rad2deg(np.min(margin[selected]))),
            'minimum_URDF_margin_joint': model.names[int(np.unravel_index(np.argmin(margin[selected]), margin[selected].shape)[1])],
            'TCP_displacement_mm': ((p[last] - p[first]) * 1000).tolist(),
            'TCP_rotation_change_from_first_active_peak_deg': float(np.max(angular_change[selected])),
            'FK_vs_logged_TCP_position_max_residual_m': residual}
        summary['arms'][robot] = info
        fields = ['joint_stamp_s', 'first_observed_sim_s', 'phase_at_first_observation']
        fields += [f'Joint_{i}_{unit}' for unit in ('rad', 'deg') for i in range(1, 6)]
        fields += ['tcp_x_m', 'tcp_y_m', 'tcp_z_m'] + ['tcp_q' + k for k in ('x', 'y', 'z', 'w')]
        fields += ['tcp_delta_rotvec_' + k + '_deg' for k in 'xyz'] + ['tcp_rotation_change_deg', 'minimum_joint_margin_deg']
        write_csv(out / (robot + '_posture.csv'), fields,
                  ([t[i], receipt[i], samples[i]['phase'], *q[i], *np.rad2deg(q[i]), *p[i], *quat[i],
                    *np.rad2deg(change[i]), angular_change[i], np.rad2deg(np.min(margin[i]))] for i in range(len(q))))
        for j, name in enumerate(model.names):
            axes[0, column].plot(t - epoch, np.rad2deg(q[:, j]), label=name, color=colors[j])
            axes[1, column].plot(t[selected] - epoch, np.rad2deg(q[selected, j] - q[first, j]), label=name, color=colors[j])
        axes[0, column].set_title(robot + ': actual joint configuration (all recorded phases)')
        axes[1, column].set_title(robot + ': actual joint change (' + summary['scope'] + ')')
        for row in range(2):
            ax = axes[row, column]
            ax.set(xlabel='Simulation time from first active sample [s]', ylabel='Angle [deg]')
            ax.grid(alpha=.25)
            ax.legend(fontsize=8, ncol=3)
        axes[0, column].axvspan(start - epoch, end - epoch, color='grey', alpha=.12)
        for index, label, color in zip([first, selected[len(selected)//2], last], ['start', 'middle', 'end'], colors):
            line = points[index] * 1000
            shape_axes[column].plot(line[:, 0], line[:, 2], 'o-', label=f'{label}: t={t[index]:.3f} s', color=color)
        shape_axes[column].set(title=robot + ': measured arm configurations (local x-z projection)', xlabel='Local x [mm]', ylabel='Local z [mm]')
        shape_axes[column].axis('equal')
        shape_axes[column].grid(alpha=.25)
        shape_axes[column].legend(fontsize=8)
        for j, axis in enumerate('xyz'):
            pose_axes[column].plot(t[selected] - epoch, np.rad2deg(change[selected, j]), label=robot + ' dR_' + axis)
        pose_axes[column].set_ylabel('TCP rotation vector [deg]')
        pose_axes[column].set_title(robot + ': orientation change from first active sample (own base frame)')
        data[robot] = {'t': t, 'receipt': receipt, 'q': q, 'points': points, 'selected': selected}
    joints_title = 'Measured JointState q; shaded interval = selected scope; source timestamps retained'
    fig.suptitle(joints_title)
    fig.savefig(out / 'joints.png', dpi=140)
    fig.savefig(out / 'joints.pdf')
    shapes.savefig(out / 'arm_shapes.png', dpi=140)
    shapes.savefig(out / 'arm_shapes.pdf')

    base_fields = ['observer_sim_s', 'phase']
    for robot in raw:
        base_fields += [robot + '_' + key for key in ('odom_stamp_s', 'odom_yaw_deg', 'GT_receipt_s', 'GT_source_stamp_s', 'GT_yaw_deg', 'GT_x_m', 'GT_y_m', 'GT_z_m')]
    base_fields += ['object_GT_receipt_s', 'object_GT_source_stamp_s', 'object_roll_deg', 'object_pitch_deg', 'object_yaw_deg']
    base_rows = []
    observation_t = np.array([r['time'] for r in observations])
    indices = np.flatnonzero((observation_t >= start) & (observation_t <= end))
    for r in observations:
        values = [r['time'], r['phase']]
        for robot in raw:
            entry = r[robot]
            yaw = 2 * np.arctan2(entry['odom'][2], entry['odom'][3])
            values += [entry['odom_stamp'], np.rad2deg(yaw), entry['gt_receipt'], entry['gt_source'], np.rad2deg(entry['gt_yaw']), *entry['gt_xyz']]
        oq = np.array(r['object_q'])
        object_rpy = Rotation.from_quat(oq).as_euler('xyz', degrees=True) if np.all(np.isfinite(oq)) and np.linalg.norm(oq) > 0 else [np.nan] * 3
        values += [r['object_receipt'], r['object_source'], *object_rpy]
        base_rows.append(values)
    write_csv(out / 'base_object_posture.csv', base_fields, base_rows)
    for robot in raw:
        yaw = np.array([r[robot]['gt_yaw'] for r in observations])[indices]
        yaw_delta = np.arctan2(np.sin(yaw - yaw[0]), np.cos(yaw - yaw[0]))
        pose_axes[2].plot(observation_t[indices] - epoch, np.rad2deg(yaw_delta), label=robot + ' recorded GT planar yaw')
        xyz = np.array([r[robot]['gt_xyz'] for r in observations])[indices]
        summary['arms'][robot]['base_GT_yaw_change_peak_deg'] = float(np.nanmax(abs(np.rad2deg(yaw_delta))))
        summary['arms'][robot]['base_GT_xy_drift_peak_mm'] = float(1000*np.nanmax(np.linalg.norm(xyz[:, :2] - xyz[0, :2], axis=1)))
    pose_axes[2].set_ylabel('Base yaw change [deg]')
    oq = np.array([r['object_q'] for r in observations])[indices]
    valid = np.all(np.isfinite(oq), axis=1) & (np.linalg.norm(oq, axis=1) > 0)
    if np.any(valid):
        object_rotations = Rotation.from_quat(oq[valid])
        object_change = np.rad2deg((object_rotations * object_rotations[0].inv()).as_rotvec())
        for j, axis in enumerate('xyz'):
            pose_axes[3].plot(observation_t[indices][valid] - epoch, object_change[:, j], label='object dR_' + axis + ' (GT, evaluation only)')
        summary['object_rotation_change_peak_deg'] = float(np.max(np.linalg.norm(object_change, axis=1)))
    pose_axes[3].set_ylabel('Object rotation vector [deg]')
    for ax in pose_axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8, ncol=3)
    pose_axes[-1].set_xlabel('Simulation time from first active sample [s]')
    posefig.savefig(out / 'tcp_base.png', dpi=140)
    posefig.savefig(out / 'tcp_base.pdf')
    plt.close('all')
    evidence = case / 'dual_arm_evidence.json'
    if evidence.exists():
        checked = json.loads(evidence.read_text())
        summary['strict_acceptance_passed'] = checked['passed']
        summary['failed_checks'] = [k for k, v in checked['checks'].items() if not v]

    # Display measured configurations with last available samples at each
    # observer time. No interpolation or shared source timestamp is invented.
    frames = []
    for i in np.unique(np.linspace(indices[0], indices[-1], min(501, len(indices))).astype(int)):
        sample = {'time': observations[i]['time'], 'phase': observations[i]['phase']}
        for robot, d in data.items():
            at = max(0, int(np.searchsorted(d['receipt'], sample['time'], side='right') - 1))
            sample[robot] = {'stamp': float(d['t'][at]), 'q_deg': np.rad2deg(d['q'][at]).tolist(), 'points_mm': (d['points'][at] * 1000).tolist()}
        frames.append(sample)
    bounds = np.concatenate([np.array([f[r]['points_mm'] for f in frames])[:, :, [0, 2]].reshape(-1, 2) for r in raw])
    low, high = bounds.min(axis=0)-40, bounds.max(axis=0)+40
    viewer = '''<!doctype html><html lang="ja"><meta charset="utf-8"><title>固定追従・姿勢ログ</title>
<style>body{font:16px sans-serif;max-width:1250px;margin:30px auto;padding:0 20px;color:#172331;background:#f8fafc}section{background:white;padding:20px;margin:20px 0;border:1px solid #dde3eb;border-radius:10px}img{max-width:100%}svg{width:49%;height:360px;background:#f2f5f9}input{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:right}th:first-child,td:first-child{text-align:left}a{color:#1455a0}p{line-height:1.65}</style>
<h1>固定追従・姿勢ログ</h1><p>CASE_TEXT</p>
<p>関節角は実測値、手先位置・姿勢は保存URDFと実測関節角によるFKです。アーム図は各台車座標のx-z投影です。台車のroll/pitchは記録されていません。</p>
<section><h2>動作区間の姿勢</h2><p id="when"></p><input id="time" type="range" min="0" max="FRAME_MAX" value="0" step="1"><svg id="leader"></svg><svg id="follower"></svg><table id="q"></table><p id="stamp"></p></section>
<section><h2>関節角度の時系列</h2><img src="joints.png"><p><a href="leader_posture.csv">リーダーCSV</a> / <a href="follower_posture.csv">フォロワーCSV</a></p></section>
<section><h2>手先・台車・物体の姿勢変化</h2><img src="tcp_base.png"><p>回転ベクトルは初期姿勢からの相対回転です。ノルムが回転角を表します。台車・物体GTは評価用です。</p><a href="base_object_posture.csv">台車・物体CSV</a></section>
<section><h2>開始・中間・終了のアーム姿勢</h2><img src="arm_shapes.png"><a href="summary.json">数値・検証結果JSON</a> / <a href="REPORT.md">説明と実行コマンド</a></section>
<script>const frames=FRAME_DATA,low=LOW_DATA,high=HIGH_DATA;const input=document.getElementById('time');function show(){const f=frames[Number(input.value)];document.getElementById('when').textContent=`シミュレーション時刻 ${f.time.toFixed(3)} s / ${f.phase}`;for(const robot of ['leader','follower']){const pts=f[robot].points_mm;const scale=Math.min(500/(high[0]-low[0]),280/(high[1]-low[1]));const xy=pts.map(p=>[40+(p[0]-low[0])*scale,320-(p[2]-low[1])*scale]);const svg=document.getElementById(robot);svg.setAttribute('viewBox','0 0 580 360');svg.innerHTML=`<text x="20" y="25">${robot}: local x-z [mm]</text><polyline points="${xy.map(p=>p.join(',')).join(' ')}" fill="none" stroke="${robot==='leader'?'#2461b2':'#d77313'}" stroke-width="6"/>`+xy.map((p,i)=>`<circle cx="${p[0]}" cy="${p[1]}" r="6" fill="#172331"/><text x="${p[0]+10}" y="${p[1]-8}" font-size="13">${i<5?'J'+(i+1):'TCP'}</text>`).join('');}document.getElementById('q').innerHTML='<tr><th>関節</th><th>Leader [deg]</th><th>Follower [deg]</th></tr>'+[0,1,2,3,4].map(i=>`<tr><td>Joint_${i+1}</td><td>${f.leader.q_deg[i].toFixed(4)}</td><td>${f.follower.q_deg[i].toFixed(4)}</td></tr>`).join('');document.getElementById('stamp').textContent=`実測JointState時刻: leader ${f.leader.stamp.toFixed(3)} s / follower ${f.follower.stamp.toFixed(3)} s（各時刻の直近記録、補間なし）`;}input.addEventListener('input',show);show();</script></html>'''
    viewer = viewer.replace('CASE_TEXT', html.escape(str(case))).replace('FRAME_MAX', str(len(frames)-1))
    viewer = viewer.replace('FRAME_DATA', json.dumps(frames, allow_nan=False)).replace('LOW_DATA', json.dumps(low.tolist())).replace('HIGH_DATA', json.dumps(high.tolist()))
    (out / 'index.html').write_text(viewer)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    report = ['# 固定追従：姿勢ログ', '', f'対象: `{case}`', '',
              f'解析区間: {summary["scope"]}, sim {start:.3f}–{end:.3f} s。図の0秒は sim {epoch:.3f} s。', '',
              '関節角はJointState実測値。TCPの位置・クォータニオン・相対回転ベクトルは、保存されたgenerated URDFのbase_footprint→tcp_link全チェーンで再計算した値です。独立した姿勢センサの測定ではありません。', '',
              'TCPはEuler角の特異姿勢付近を取るため、姿勢変化は相対回転ベクトル[deg]とそのノルムで表示します。左右はそれぞれ自身の台車座標です。', '',
              '同じJointState時刻の繰り返し行は除去し、各CSVに元の関節時刻と最初の観測時刻を残しました。観測時刻はロガーのsim時刻であり、非同期のclock更新により関節スタンプより数ms遅れる行もそのまま保持しています（時刻差範囲はsummary.json）。台車は元ログの平面yawのみを出力し、未記録のroll/pitchを補っていません。台車・物体の相対回転は解析区間開始を基準にしています。GTは評価専用で、GT受信時刻と元スタンプをCSVに保持しています。', '',
              '| Robot | Joint | 開始 [deg] | 終了 [deg] | 変化 [deg] |', '|---|---|---:|---:|---:|']
    for robot, info in summary['arms'].items():
        for i, name in enumerate(info['joint_names']):
            report.append(f'| {robot} | {name} | {info["joint_start_deg"][i]:.5f} | {info["joint_end_deg"][i]:.5f} | {info["joint_change_deg"][i]:.5f} |')
        report += ['', f'{robot}: TCP変位 {info["TCP_displacement_mm"]} mm。初回active姿勢からの回転角最大 {info["TCP_rotation_change_from_first_active_peak_deg"]:.7f} deg。最小URDF関節余裕 {info["minimum_URDF_margin_deg"]:.5f} deg ({info["minimum_URDF_margin_joint"]})。', '']
    report += [f'元手順判定: {summary["procedure_passed"]}。リーダー完了: {summary["leader_completed"]}。error: {summary["error"]}。', '',
               f'厳密受入判定: {summary.get("strict_acceptance_passed", "未評価")}。未合格項目: {summary.get("failed_checks", [])}。姿勢図の作成によって元の判定は変更していません。', '',
               '再生成:', '```bash', 'cd /home/dars5070/ros2_humble_ws',
               f'PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/amir_posture_logs.py --case {case} --output /tmp/amir_posture_new', '```', '',
               '閲覧: `index.html`をブラウザで開き、スライダで実測姿勢を確認できます。PNG/PDFは時系列・静止姿勢図、CSVは全保存区間を収録しています。']
    (out / 'REPORT.md').write_text('\n'.join(report) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
