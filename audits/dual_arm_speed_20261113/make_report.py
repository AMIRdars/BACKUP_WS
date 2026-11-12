"""Preserve actual speed evidence and report unqualified requests explicitly."""
from pathlib import Path
import csv
import difflib
import hashlib
import json
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
NEW=OUT/'speed_004_async/trial'
BASE=ROOT/'audits/dual_arm_range_20261112/max_213_smooth/trial'


def measurements(case,result_name):
    result=json.loads((case/result_name).read_text())
    rows=[r for r in csv.DictReader((case/'timeseries.csv').open())
          if r.get('admittance_enabled')=='True' and r.get('adm_p_actual_m_0')]
    motion=[r for r in rows if r['phase']=='leader_translation']
    measured={};series={}
    for robot,sign in [('leader',1),('follower',-1)]:
        # FK displacement is paired with its actual JointState source time.
        # Repeated observer rows do not become zero-speed measurements.
        samples={float(r[robot+'_joint_stamp_s']):float(r[robot+'_p_actual_0']) for r in motion}
        t=np.array(sorted(samples));x=np.array([samples[k] for k in t]);v=sign*np.diff(x)/np.diff(t)
        series[robot]={'t':t-t[0],'x':sign*(x-x[0]),'v':v}
        measured[robot]={'FK_source_speed_peak_m_s':float(np.max(v)),
            'FK_source_speed_p99_5_m_s':float(np.quantile(v,.995)),
            'joint_speed_actual_peak_rad_s':max(abs(float(r[robot+'_qdot_actual_'+str(i)])) for r in motion for i in range(5)),
            'joint_source_samples':len(samples)}
    evidence=json.loads((case/'dual_arm_evidence.json').read_text())
    return result,measured,series,evidence


base,bm,bs,be=measurements(BASE,'held_result.json')
result,measured,series,evidence=measurements(NEW,'held_result.json')
target=json.loads((OUT/'target_1m_s.json').read_text())
with (NEW/'timeseries.csv').open() as f:
    active_rows=[r for r in csv.DictReader(f) if r.get('admittance_enabled')=='True' and r.get('leader_comp_qdot_rad_s_0')]
reference_joint_peak=max(abs(float(r['leader_comp_qdot_rad_s_'+str(i)])) for r in active_rows for i in range(5))
preflight_elapsed=float(active_rows[0]['leader_comp_preflight_wall_s'])
assert result['passed'] and result['safe_stop_verified'] and result['grasp_held_after_stop']
checks={}
for case in [BASE,NEW]:
    expected=json.loads((case/'source_hashes.json').read_text())
    checks[str(case.relative_to(ROOT))]={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==sha for p,sha in expected.items()}
assert all(checks[str(NEW.relative_to(ROOT))].values())
expected_baseline_changes={'src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_only_leader.py','src/humbleble_ws/amir_compliance_control/amir_compliance_control/stroke_kinematics.py'}
assert {p for p,good in checks[str(BASE.relative_to(ROOT))].items() if not good}==expected_baseline_changes
(OUT/'tested_source_consistency.json').write_text(json.dumps(checks,indent=2))
metrics={'motion_completed':result['passed'],'strict_acceptance_passed':evidence['passed'],
    'strict_checks_passed':sum(evidence['checks'].values()),'strict_checks_total':len(evidence['checks']),
    'failed_checks':[k for k,v in evidence['checks'].items() if not v],
    'requested_speed_m_s':.004,'baseline_requested_speed_m_s':.003,
    'measured':measured,'baseline_measured':bm,'dual_arm_metrics':evidence['dual_arm_metrics'],
    'leader_actual_displacement_m':result['leader_actual_arm_displacement_m'],
    'payload_displacement_xyz_m':result['payload_model_gt_displacement_xyz_m'],
    'safe_stop_verified':result['safe_stop_verified'],'grasp_held_after_stop':result['grasp_held_after_stop'],
    'leader_joint_reference_speed_peak_rad_s':reference_joint_peak,'async_preflight_wall_s':preflight_elapsed,
    'p99_5_speed_ratio':measured['leader']['FK_source_speed_p99_5_m_s']/bm['leader']['FK_source_speed_p99_5_m_s']}
(OUT/'speed_result.json').write_text(json.dumps(metrics,indent=2))
fig,axes=plt.subplots(3,1,figsize=(12,10),constrained_layout=True)
for label,data in [('3 mm/s baseline',bs),('4 mm/s trial',series)]:
    for i,robot in enumerate(['leader','follower']):
        s=data[robot];axes[i].plot(s['t'][1:],s['v']*1000,label=label)
        axes[2].plot(s['t'],s['x']*1000,label=label+' '+robot)
for i,robot in enumerate(['leader','follower']):axes[i].set_ylabel(robot+' FK speed [mm/s]')
axes[2].set_ylabel('Actual FK displacement [mm]')
for ax in axes:ax.set_xlabel('JointState source time from first motion sample [s]');ax.grid(alpha=.3);ax.legend()
fig.suptitle('Same 213 mm arm-only stroke: measured 3 vs 4 mm/s\n4 mm/s strict acceptance passed='+str(evidence['passed']))
for ext in ['png','pdf']:fig.savefig(OUT/('speed_comparison.'+ext),dpi=150)
plt.close(fig)
with (OUT/'measured_speed.csv').open('w') as f:
    writer=csv.writer(f);writer.writerow(['case','robot','joint_source_elapsed_s','FK_displacement_m','FK_interval_velocity_m_s'])
    for label,data in [('baseline_003',bs),('trial_004',series)]:
        for robot,s in data.items():
            for i in range(1,len(s['t'])):writer.writerow([label,robot,s['t'][i],s['x'][i],s['v'][i-1]])
paths=['tools/amir_dual_arm_speed_audit.py','tools/test_dual_arm_speed_audit.py',
       'src/humbleble_ws/cooperative_transport_bringup/README_dual_arm_only.md','tools/test_arm_preflight.py','src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_only_leader.py','src/humbleble_ws/amir_compliance_control/amir_compliance_control/stroke_kinematics.py']
manifest={};patch=''
for name in paths:
    p=ROOT/name;b=OUT/'before'/name;s=OUT/'source_snapshot'/name;s.parent.mkdir(parents=True,exist_ok=True);s.write_bytes(p.read_bytes())
    manifest[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    patch+=''.join(difflib.unified_diff(b.read_text().splitlines(True) if b.exists() else [],p.read_text().splitlines(True),fromfile='a/'+name if b.exists() else '/dev/null',tofile='b/'+name))
(OUT/'changed_files.json').write_text(json.dumps(manifest,indent=2));(OUT/'changes.patch').write_text(patch)
assert subprocess.check_output(['git','diff','--binary'],cwd=ROOT)==(OUT/'tracked_before.patch').read_bytes()
first=json.loads((OUT/'speed_004/trial/result.json').read_text())
report=f'''# 同一213mm軌道の速度検証

2026-11-13。1m/sは搬送方向の手先ピーク速度として評価した。既存の両台車停止・局所F/T追従・把持を維持し、既定3mm/sを変えず4mm/sを比較した。

## 結果

4mm/s要求のGUI試験は動作完了、安全停止、停止後把持保持を確認。厳密判定は{metrics['strict_checks_passed']}/{metrics['strict_checks_total']}項目合格、全項目合格={evidence['passed']}。
不合格項目: {metrics['failed_checks']}。

| 実測 | 3mm/s基準 | 4mm/s GUI試験 |
|---|---:|---:|
| L手先速度ピーク [mm/s] | {bm['leader']['FK_source_speed_peak_m_s']*1000:.6f} | {measured['leader']['FK_source_speed_peak_m_s']*1000:.6f} |
| L手先速度99.5百分位 [mm/s] | {bm['leader']['FK_source_speed_p99_5_m_s']*1000:.6f} | {measured['leader']['FK_source_speed_p99_5_m_s']*1000:.6f} |
| F手先速度ピーク [mm/s] | {bm['follower']['FK_source_speed_peak_m_s']*1000:.6f} | {measured['follower']['FK_source_speed_peak_m_s']*1000:.6f} |
| L関節速度実測最大 [rad/s] | {bm['leader']['joint_speed_actual_peak_rad_s']:.6f} | {measured['leader']['joint_speed_actual_peak_rad_s']:.6f} |
| F関節速度実測最大 [rad/s] | {bm['follower']['joint_speed_actual_peak_rad_s']:.6f} | {measured['follower']['joint_speed_actual_peak_rad_s']:.6f} |

Lの関節速度参照ピークは{reference_joint_peak:.6f}rad/sで0.03rad/sの**指令**上限以内。一方、JointState実測速度には最大{measured['leader']['joint_speed_actual_peak_rad_s']:.6f}rad/sの瞬間的な上振れがある。0.03rad/sは指令の上限であり、全実測サンプルがその値以下とは確認できていない。URDF速度上限以内だが、瞬間値の原因と再現性は残課題。

手先速度は指令値でなくFK実測位置の差を対応するJointState source stampの差で割った区間平均。観測行の重複を除いた。99.5百分位でL速度は{metrics['p99_5_speed_ratio']:.4f}倍。理論上の五次補間時間は133.125→99.84375sim秒で、wall秒の等倍再生を意味しない。

L実測移動{result['leader_actual_arm_displacement_m']*1000:.6f}mm、物体{result['payload_model_gt_displacement_xyz_m'][0]*1000:.6f}mm。
両台車の指令は接近停止後ゼロ。ARM_ONLY中の物理GTドリフト最大L={evidence['dual_arm_metrics']['leader']['body_drift_during_ARM_ONLY_peak_m']*1000:.6f}mm、F={evidence['dual_arm_metrics']['follower']['body_drift_during_ARM_ONLY_peak_m']*1000:.6f}mm。把持ずれ・指反力・単一writer・購読境界はtrial/dual_arm_evidence.jsonへ保存。

## 1m/sを設定しない理由

同じ213mmの姿勢固定軌道を実際のピーク1m/sにするには五次補間時間{target['target_nominal_duration_s']:.6f}秒、Cartesian最大加速度{target['target_required_TCP_peak_acceleration_m_s2']:.6f}m/s²が必要。実URDFのFK/Jで計算した関節速度ピークはL=6.51538rad/s、F=3.42370rad/s。該当関節のURDF速度上限1rad/sを超える。

現行の実験用関節指令速度0.03rad/s・指令加速度0.15rad/s²・Cartesian加速度0.0012m/s²の枠を維持した軌道の速度目安は約4.37mm/s。URDF速度だけで求めた約0.15348m/sは、荷重・反力・遅れ・把持で合格した速度ではない。1m/s要求を現在のlaunchへ渡すと拒否する。加速度制約で実際は遅くなる軌道を1m/s達成とは扱わない。URDFや力停止値の書換えで目標を満たしたことにはしない。

## 試験と変更

最初の4mm/s試験speed_004は移動開始時の準備応答異常で安全停止。result.error={first.get('error')}。停止・把持は確認。保存された最初の局所異常はendpointのactuator acknowledgement not readyで、後続のgateにはlocal grasp detachedが記録された。物理接触と指反力は保持しており、当初のログだけでは最初の応答欠落の原因は未確定だった。headless再試験speed_004_retryも移動前に同じ準備応答異常で停止した（停止・把持を確認）。他のテストとの同時実行を終え、元の許可CPU集合を継承するnative_cpu_policy=inheritを用いても解消しなかった。3回目speed_004_guiもGUIあり・元のSMT設定で移動前に同じ異常で停止した。追加観測startup_transitions.jsonlと単独計算preflight_timing.jsonから、goalコールバックの同期31点IK確認が約0.165秒受信処理を止め、0.15秒のローカル受信監視を超える問題を確認した。31点確認をワーカーへ移し、ROS側は従来の期限で受信とゼロ出力を続けるよう修正した。完了時に現在の局所健全性と初期関節位置を再確認してから受理する。停止中・局所不健全・位置変化・IK失敗なら受理しない。計算自体は2秒で打ち切り判定し、既存のセンサ・permit・interlock・dt等の期限は維持した。4回目speed_004_asyncでGUI試験した。実行時の事前確認所要時間は{preflight_elapsed:.6f}wall秒で0.15秒を超えたが、受信処理を継続して移動を完了した。繰返し成功率を保証するものではない。

final_tests.log:45件合格。事前確認中のゼロ出力、停止等の拒否、速度制約、1m/sの拒否、既存可動域・両台車ゼロ・native動作回帰を含む。旧tests.logは修正前39件合格。ソースSHA整合はtested_source_consistency.json。基準は同期事前確認方式の実測で、運動制御則・速度監視は同じ。基準3mm/sはaudits/dual_arm_range_20261112/max_213_smooth/trialの既存実測で、統計的な対照試験・繰返し成功率ではない。

- tools/amir_dual_arm_speed_audit.py:ROS起動なしで、指定速度に必要な関節速度・加速度を実URDFから評価。
- tools/test_dual_arm_speed_audit.py:1m/sを実現したと誤表示しないことと既存上限を確認。
- README_dual_arm_only.md:速度引数、分析コマンド、制約を追記。
- arm_only_leader.py、stroke_kinematics.py:同じ31点IK事前確認を非同期化し、完了時の再検査を追加。
- test_arm_preflight.py:計算中のゼロ出力、停止・状態変化・IK失敗時の拒否、実モデルの事前確認を検証。

アームcontroller/API、F/T、ゲイン、関節名、URDF、Gazebo/Humble、台車停止条件と既定3mm/sは維持。制御packageをビルド済み。別環境では `colcon build --packages-select amir_compliance_control` 後にinstall/setup.bashをsourceする。利用者の既存GUI/プロセスには終了要求を送っていない。

## 実行

既存GUIは使用中の端末でCtrl+C後、次を実行。今回のGUI試験と同じ設定を明示する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \\
  headless:=false gui_nvidia_offload:=true \\
  forward_distance_m:=1.0 translation_velocity_m_s:=0.1 record_data:=true \\
  dual_arm_distance_m:=0.213 dual_arm_speed_m_s:=0.004
```

最終4mm/s試験は別domainのGUIありで実施し、停止後に物理を一時停止してGUIを保持。元のユーザーGUIも維持。5mm/s要求は約4.37mm/sへ自動減速する。既定へ戻すには速度引数を省略する。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \\
  tools/amir_dual_arm_speed_audit.py --target-speed 1.0 --output <新規JSON保存先>
```

## 証拠と残課題

target_1m_s.jsonとtarget_004m_s.jsonは計算、speed_result.jsonは実測結果。speed_comparison.png/pdf、measured_speed.csv、trial/timeseries.csv、dual_arm_evidence.jsonに全項目の合否を保存。changes.patch/source_snapshot/に変更を保存し、既存tracked差分を維持。

1m/sは未達。現在の同姿勢軌道ではURDF速度上限を超える。さらなる高速化は姿勢・軌道、実験用速度枠、加速時間、F/Tとservo帯域、摩擦把持の段階的な再検証を要する。瞬間的な指反力低下、実測関節速度の上振れ、準備応答の再現性、繰返し成功率も残課題。ベース移動による速度増加は今回行っていない。
'''
(OUT/'REPORT.md').write_text(report)
print(OUT/'REPORT.md')
