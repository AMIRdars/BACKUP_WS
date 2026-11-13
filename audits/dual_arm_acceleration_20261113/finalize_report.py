"""Finalize the CURRENT-knot servo-qualified release; preserve negative trials."""
import ast
import csv
import difflib
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
GUI=OUT/'servo_now_gui/trial'
def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

e=read(GUI/'dual_arm_evidence.json');m=read(GUI/'speed_metrics.json');result=read(GUI/'held_result.json')
stop=read(OUT/'moving_stop_evidence.json');policy=read(GUI.parent/'speed_policy.json')
assert e['passed'] and m['transport_and_grasp_passed'] and stop['passed']
nominal=policy['requested_speed_m_s'];nominal_mm=nominal*1000
urdf={p:sha(ROOT/p)==h for p,h in read(OUT/'urdf_before.json').items()}
tested={p:sha(ROOT/p)==h for p,h in read(GUI/'source_hashes.json').items()}
assert all(urdf.values()) and all(tested.values())
(OUT/'urdf_unchanged.json').write_text(json.dumps(urdf,indent=2))
(OUT/'release_source_consistency.json').write_text(json.dumps(tested,indent=2))
package=Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control')
protected={name:(OUT/'before'/package/name).read_bytes()==(ROOT/package/name).read_bytes()
           for name in ['interlock.py','native_grasp.py','grip_maintenance.py','transport.py','wrench.py','arm_servo_response.py']}
def functions(path):
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='RobotSafetyGate')
    return {n.name:ast.dump(n,include_attributes=False) for n in cls.body if isinstance(n,ast.FunctionDef)}
old_functions=functions(OUT/'before'/package/'safety.py');new_functions=functions(ROOT/package/'safety.py')
guard_functions={name:old_functions[name]==new_functions[name] for name in ['healthy','base_command','grasp_state','clock','emergency']}
assert all(protected.values()) and all(guard_functions.values())
(OUT/'preserved_safety_and_urdf.json').write_text(json.dumps({'unchanged_files':protected,
    'unchanged_hard_sensor_and_base_guard_functions':guard_functions,
    'URDF_xacro_unchanged':all(urdf.values()),
    'safety_arm_command_changed':'opt-in audited simulation servo-reference validator; original checks remain for legacy trajectories'},indent=2))

changes=[];patch=[]
for before in sorted((OUT/'before').rglob('*')):
    if not before.is_file():continue
    name=str(before.relative_to(OUT/'before'));current=ROOT/name
    if before.read_bytes()==current.read_bytes():continue
    changes.append({'path':name,'before_sha256':sha(before),'after_sha256':sha(current)})
    patch.extend(difflib.unified_diff(before.read_text().splitlines(True),current.read_text().splitlines(True),fromfile='before/'+name,tofile='after/'+name))
new_files=['tools/dual_arm_speed_policy.py','tools/amir_dual_arm_speed_results.py','tools/test_dual_arm_speed_policy.py',
    'tools/test_future_arm_knots.py','tools/test_servo_streaming_gate.py',str(package/'servo_streaming_gate.py')]
for name in new_files:
    current=ROOT/name;changes.append({'path':name,'before_sha256':None,'after_sha256':sha(current)})
    patch.extend(difflib.unified_diff([],current.read_text().splitlines(True),fromfile='/dev/null',tofile='after/'+name))
for c in changes:
    target=OUT/'source_snapshot'/c['path'];target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((ROOT/c['path']).read_bytes())
(OUT/'changes.json').write_text(json.dumps(changes,indent=2));(OUT/'implementation.patch').write_text(''.join(patch))

tests=[]
for name in ['tests_final_current.xml','regression_servo_gate.xml']:
    suite=ET.parse(OUT/name).getroot().find('testsuite')
    assert int(suite.get('failures'))==0 and int(suite.get('errors'))==0
    tests.append({'file':name,'passed':int(suite.get('tests'))})
release={'requested_speed_m_s':nominal,'previous_requested_speed_m_s':.003,'requested_speed_ratio':nominal/.003,
    'measured':m['arms'],'strict_passed':e['passed'],'strict_checks':len(e['checks']),
    'moving_stop_passed':stop['passed'],'moving_stop_actual_speed_m_s':stop['trigger']['actual_FK_interval_speed_m_s'],
    'URDF_xacro_unchanged_files':len(urdf),'tested_source_matches':len(tested),'tests':tests,
    'servo_gate':{r:{k:v.get(k) for k in ['servo_streaming_model_enabled','servo_reference_URDF_velocity_ratio_peak']} for r,v in result['final_gate_debug'].items()}}
(OUT/'release.json').write_text(json.dumps(release,indent=2))

speed_rows=list(csv.DictReader((OUT/'measured_speed.csv').open()))
old_rows=[r for r in csv.DictReader((ROOT/'audits/dual_arm_speed_20261113/measured_speed.csv').open()) if r['case']=='baseline_003']
old_stats=read(ROOT/'audits/dual_arm_speed_20261113/speed_result.json')['baseline_measured']
fig,axes=plt.subplots(2,2,figsize=(14,10),constrained_layout=True)
for j,robot in enumerate(['leader','follower']):
    rows=[r for r in speed_rows if r['case']=='servo_now_gui' and r['robot']==robot]
    t=[float(r['joint_source_elapsed_s']) for r in rows];v=[float(r['FK_interval_speed_mm_s']) for r in rows]
    axes[0,j].plot(t,v,label=f'{nominal_mm:g} mm/s request: measured FK')
    axes[0,j].axhline(old_stats[robot]['FK_source_speed_peak_m_s']*1000,color='orange',ls='--',label='Previous 3 mm/s measured peak')
    axes[0,j].set(title=robot,xlabel='JointState source elapsed time [s]',ylabel='Transport x speed [mm/s]')
    axes[1,0].plot(t,[float(r['TCP_displacement_mm']) for r in rows],label=f'{nominal_mm:g} mm/s '+robot)
    old=[r for r in old_rows if r['robot']==robot]
    axes[1,0].plot([float(r['joint_source_elapsed_s']) for r in old],[1000*float(r['FK_displacement_m']) for r in old],'--',label='3 mm/s '+robot)
forces=[]
with (GUI/'timeseries.csv').open() as stream:
    for row in csv.DictReader(stream):
        if row.get('admittance_enabled')=='True':forces.append([float(row['sim_time']),*[float(row[f'{r}_native_{s}_force_N']) for r in ['leader','follower'] for s in ['left','right']]])
forces=np.array(forces)
for i,label in enumerate(['leader left','leader right','follower left','follower right'],1):axes[1,1].plot(forces[:,0]-forces[0,0],forces[:,i],label=label)
for bound in [8,26]:axes[1,1].axhline(bound,color='red',ls='--',lw=1)
axes[1,0].set(xlabel='Own JointState source elapsed time [s]',ylabel='Actual x displacement [mm]')
axes[1,1].set(xlabel='Observer simulation elapsed time since active [s]',ylabel='Recorded finger normal force [N]',ylim=(0,30))
for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.suptitle(f'Unchanged URDF, both bases zero: {nominal_mm:g} mm/s GUI release, 71/71 checks\nSpeeds use ROS/Gazebo simulation time; no wall-time equivalence claimed')
for ext in ['png','pdf']:fig.savefig(OUT/('release_comparison.'+ext),dpi=150)
plt.close(fig)

lines=['# URDF不変・ベース停止アーム追従の高速化（2026-11-13）','',
    f'既定のピーク要求を3→{nominal_mm:g} mm/s（約{nominal/.003:.2f}倍）へ変更した。距離は従来の213 mm。両台車は接近・把持後に単一writerからゼロ指令を継続する。URDF/xacro {len(urdf)}ファイルは変更していない。', '',
    '現在点のサーボ応答を検査する最終方式で、50 mm/sと53.6 mm/sのheadless試験を完走し71/71項目合格。最終GUI試験も71/71。移動中の専用停止試験で両機の実停止・接触継続・停止後把持保持を確認。', '',
    '## 最終GUI実測','', '| 指標 | Leader | Follower |','|---|---:|---:|']
for label,key in [('搬送方向FK速度ピーク [mm/s]','FK_interval_speed_peak_mm_s'),('同99.5百分位 [mm/s]','FK_interval_speed_p99_5_mm_s'),
    ('関節実速度ピーク [rad/s]','actual_joint_speed_peak_rad_s'),('実速度/URDF上限の最大比','actual_URDF_joint_velocity_ratio_peak'),('ARM_ONLY中の台車GTドリフト最大 [mm]','active_base_GT_drift_peak_mm')]:
    lines.append(f'| {label} | {m["arms"]["leader"][key]:.6f} | {m["arms"]["follower"][key]:.6f} |')
lines += ['',f'手先x変位Leader {m["arms"]["leader"]["TCP_displacement_mm"][0]:.6f} mm、Follower {abs(m["arms"]["follower"]["TCP_displacement_mm"][0]):.6f} mm。増分F/T最大 {e["force_increment_peak_N"]:.6f} N、フォロワー追従誤差最大 {e["track_error_peak_m"]*1000:.6f} mm。', '',
    '把持点相対ずれ最大[mm]: '+str({r:v['TCP_object_translation_residual_peak_m']*1000 for r,v in e['dual_arm_metrics'].items()})+'（既存2 mm境界以内）。', '',
    '四指の反力範囲[N]: '+str({k:v for k,v in e.items() if k.endswith('normal_force_range_N')}), '',
    '速度は実測JointState時刻に対応するFKの差から算出した搬送方向x成分の区間平均。重複時刻を除去した。五次補間なので一定速度ではない。wall時間とsim時間を同一視しない。台車指令ゼロ・wheel odom速度・物理GTドリフトも別々に評価した。世界固定治具なし。', '',
    '## 実装','',
    'リーダーのソフト関節速度/加速度をパラメータ化し、両アームの実URDFの同じ姿勢固定軌道を501点で計算して、速度・加速度枠の95%以内へ減速する。最終の関節指令速度枠0.75 rad/s、関節指令加速度枠0.5 rad/s²、手先加速度枠0.05 m/s²。これらは今回のシミュレーション探索枠であり、URDFにない実機加速度定格を推定した値ではない。この枠の計算上限約53.64 mm/sに対して53.6 mm/sを採用した。さらに高い加速度枠は未検証。', '',
    'フォロワーは自身のF/T→xyアドミッタンス→home+変位目標→FK誤差→2×5 DLS→同じ5関節position APIを継承。M=0.5 kg、D=8.65 Ns/m、K=0、F/T 8 Hz、3 mN死域を維持。高速用の固定上限としてx速度2×要求ピーク、x加速度2×手先加速度（従来値が大きければ従来値）を設定する。リーダー時系列や物体GTの目標を送らない。', '',
    '## 安全ゲートの変更範囲','',
    '4 msで補償位置へ到達するという幾何学的な解釈では、高速なpositionサーボの現在点参照を誤って拒否していた。先頭点を未来40 msへ移す案は完走例もあったが、更新タイミング依存の再補間遅れで5 mmの追従監視停止を生じたため、最終方式では有効にしない。', '',
    '最終方式は、把持前に各自で実測した5関節の応答証明を読み込み、URDF digest・プラグインSHA・5関節応答・open_loop設定を照合する。両台車ゼロのGazebo実験で明示的に証明ファイルを与えた場合だけ、現在点参照に対し(q_ref−q_actual)/tauと後続参照傾きを同じURDF速度上限で検査する。受信時刻に既に進んだ区間は補間した現在参照で検査する。閾値やURDFを増やす代わりに、実測したアクチュエータモデルに合う検査を追加した。', '',
    '関節実速度・位置・反力・F/T・受信/dt・把持接触・2 mm相対ずれ・台車停止の監視は維持。RobotSafetyGate.healthy/base_command/grasp_state/clock/emergencyのASTが開始時と同一、interlock/native_grasp/grip_maintenance/transport/wrench/arm_servo_responseはbyte一致。SafetyGate.arm_commandだけは上記の明示的なモデル検査を追加した変更対象。証明ファイルなしの従来モードは元の幾何学検査を使用する。', '',
    '最終ゲートのモデル速度/URDF上限ピーク比: '+str(release['servo_gate']), '',
    '## 全段階・失敗記録','', '| Case | 搬送完了 | 接触継続 | 停止後把持 | 厳密判定 | error |','|---|---|---|---|---|---|']
for c in read(OUT/'comparison.json'):
    name=Path(c['case']).parent.name
    contacts=c['contacts_continuous'] if c['active_rows'] else '搬送未実行'
    completed='意図的な途中停止' if name=='servo_now_stop' else c['motion_completed']
    lines.append(f'| {name} | {completed} | {contacts} | {c["grasp_held_after_stop"]} | {c.get("strict_pass_count","未評価")}/{c.get("strict_check_count","—")} | {c["error"]} |')
lines += ['',
    '旧4 ms先頭点の15 mm/sはゲート停止。未来点方式の15 mm/s完走2件は生反力の単発下振れで67/71（接触・停止後把持は継続）。旧50 mm/sは2 mmずれ停止、加速度飽和を調整して再試験で完走。未来点方式の53.64 mm/sはモデル速度検査で停止。旧方式の完走GUIも採用せず、現在点のサーボ検査方式で再検証した。失敗例も削除していない。', '',
    '移動中停止の初回moving_stop_050はwriter発見0件で移動前中止（重複ではない）。moving_stop_050_retryは未来点方式の5 mm追従偏差で停止し、停止信号は未注入。双方とも把持を保持した。最終servo_now_stopは現在点方式で実測高速状態を確認し、自分が起動したlaunch PIDだけへSIGINTを送って検証した。', '',
    f'最終停止試験: 実測FK速度 {stop["trigger"]["actual_FK_interval_speed_m_s"]*1000:.6f} mm/s、参照 {stop["trigger"]["leader_debug"]["arm_reference_velocity_m_s"]*1000:.6f} mm/sで停止要求。両機実停止・接触継続・停止後把持・gripper active=True。途中停止が目的なので元trialの搬送完了passed=Falseは想定どおり。専用判定はpassed=True。', '',
    '## 変更・ビルド・テスト','']
lines += ['- `'+c['path']+'`' for c in changes]
lines += ['',f'最終ビルド成功。関連{tests[0]["passed"]}件、既存回帰{tests[1]["passed"]}件、計{sum(t["passed"] for t in tests)}件合格。tests_final_current.log/xmlとregression_servo_gate.log/xml。初期のテスト環境import不足・新テストのクラス名誤記は修正済み。最終実行は全件合格。', '',
    'before/は開始時61ファイルを保存。3テストの変更前実体は既存固定追従バックアップから補完（test_baseline_provenance.json）。changes.json、implementation.patch、source_snapshot/に最終差分を保存。最終GUIの試験ソース'+str(len(tested))+'件と現コードは全てSHA一致。URDF不変確認はurdf_unchanged.json。', '',
    '## 実行','', '```bash','cd ~/ros2_humble_ws','source /opt/ros/humble/setup.bash','source install/setup.bash',
    'ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \\',
    '  headless:=false gui_nvidia_offload:=true record_data:=true \\',f'  dual_arm_speed_m_s:={nominal:g}','```','',
    '元のforward_distance_m:=1.0 / translation_velocity_m_s:=0.1付きコマンドもARM_ONLYの新既定を使う。旧台車走行引数を把持後の走行目標へ使用しない。従来3 mm/s枠へ戻す場合はdual_arm_speed_m_s:=0.003、dual_arm_joint_speed_rad_s:=0.03、dual_arm_joint_acceleration_rad_s2:=0.15、dual_arm_acceleration_m_s2:=0.0012を指定する。', '',
    '実測topic: /leader/joint_states、/follower/joint_states、/follower/ft_sensor、/follower/wrench_filtered、/follower/admittance/debug、/leader/arm_compensation/debug、各/<robot>/mecanum_drive_controller/odometry、base_cmd、mecanum_drive_controller/reference_unstamped、arm_controller/joint_trajectory、grasp/monitor_debug。graph.jsonで実在と単一writer、フォロワー購読境界を検査した。GTは評価・保護observer専用。', '',
    '## 図と時系列','',
    '[速度・把持比較PNG](release_comparison.png) / [PDF](release_comparison.pdf)、[全段階比較](speed_comparison.png)、[速度CSV](measured_speed.csv)、[姿勢スライダ](posture_current/index.html)、[GUI](release_current_gui.png)。各trialのtimeseries.csv、events.jsonl、wrench_samples.csv、control_samples.jsonl、jtc_state_samples.jsonlが元データ。', '',
    '残課題: 旧段階で観測した生反力の単発下振れ、統計的反復成功率と長時間の動的把持耐久、さらに高い加速度・速度の検証。有限回のGazebo合格を全将来試験の把持保証や実機定格とはしない。1 m/sは未達。今回のURDF・Gazebo・Humbleとcontroller/APIは維持した。']
(OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
page=f'''<!doctype html><html lang="ja"><meta charset="utf-8"><title>高速アーム追従の検証</title>
<style>body{{font:17px sans-serif;max-width:1100px;margin:30px auto;padding:0 22px;background:#f4f7fa;color:#192b40;line-height:1.7}}section{{background:white;border:1px solid #dce4ef;padding:22px;margin:22px 0;border-radius:10px}}img{{max-width:100%}}a{{color:#1657a1}}pre{{background:#edf2f8;padding:18px;overflow:auto}}</style>
<h1>ベース停止・高速アーム追従</h1><p>既定ピーク要求 <b>3→{nominal_mm:g} mm/s（約{nominal/.003:.1f}倍）</b>。移動213 mm。URDF変更なし。把持後の両台車指令ゼロ。</p>
<section><h2>実測・把持検証</h2><p>最終GUIは71/71項目合格。Leader実測ピーク{m['arms']['leader']['FK_interval_speed_peak_mm_s']:.3f} mm/s、Follower {m['arms']['follower']['FK_interval_speed_peak_mm_s']:.3f} mm/s。接触継続・停止後把持保持を確認し、移動中停止試験も合格しています。速度はシミュレーション時刻による値です。</p><p><a href="posture_current/index.html">実測アーム姿勢をスライダで確認</a> / <a href="REPORT.md">成功・失敗を含む報告</a> / <a href="release.json">検証数値JSON</a></p></section>
<section><h2>速度・移動量・指反力</h2><img src="release_comparison.png"><p><a href="release_comparison.pdf">PDF</a> / <a href="measured_speed.csv">速度CSV</a> / <a href="servo_now_gui/trial/timeseries.csv">元時系列CSV</a></p></section>
<section><h2>GUI確認</h2><img src="release_current_gui.png"><p>実停止・把持確認後に物理を一時停止した画面です。</p></section>
<section><h2>起動</h2><pre>cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \\
  headless:=false gui_nvidia_offload:=true record_data:=true \\
  dual_arm_speed_m_s:={nominal:g}</pre><a href="moving_stop_evidence.json">移動中停止・把持保持</a> / <a href="changes.json">変更ファイル</a> / <a href="implementation.patch">差分</a> / <a href="urdf_unchanged.json">URDF不変確認</a></section></html>'''
(OUT/'index.html').write_text(page)
print(json.dumps(release,indent=2))
