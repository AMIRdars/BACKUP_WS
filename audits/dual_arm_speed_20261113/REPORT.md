# 同一213mm軌道の速度検証

2026-11-13。1m/sは搬送方向の手先ピーク速度として評価した。既存の両台車停止・局所F/T追従・把持を維持し、既定3mm/sを変えず4mm/sを比較した。

## 結果

4mm/s要求のGUI試験は動作完了、安全停止、停止後把持保持を確認。厳密判定は67/71項目合格、全項目合格=False。
不合格項目: ['native_force_band:leader/left', 'native_force_band:leader/right', 'native_force_band:follower/left', 'native_force_band:follower/right']。

| 実測 | 3mm/s基準 | 4mm/s GUI試験 |
|---|---:|---:|
| L手先速度ピーク [mm/s] | 3.000128 | 4.000308 |
| L手先速度99.5百分位 [mm/s] | 2.999819 | 3.999710 |
| F手先速度ピーク [mm/s] | 3.591279 | 4.890452 |
| L関節速度実測最大 [rad/s] | 0.019550 | 0.034236 |
| F関節速度実測最大 [rad/s] | 0.017773 | 0.017820 |

Lの関節速度参照ピークは0.026062rad/sで0.03rad/sの**指令**上限以内。一方、JointState実測速度には最大0.034236rad/sの瞬間的な上振れがある。0.03rad/sは指令の上限であり、全実測サンプルがその値以下とは確認できていない。URDF速度上限以内だが、瞬間値の原因と再現性は残課題。

手先速度は指令値でなくFK実測位置の差を対応するJointState source stampの差で割った区間平均。観測行の重複を除いた。99.5百分位でL速度は1.3333倍。理論上の五次補間時間は133.125→99.84375sim秒で、wall秒の等倍再生を意味しない。

L実測移動212.999169mm、物体211.480535mm。
両台車の指令は接近停止後ゼロ。ARM_ONLY中の物理GTドリフト最大L=0.366276mm、F=0.365502mm。把持ずれ・指反力・単一writer・購読境界はtrial/dual_arm_evidence.jsonへ保存。

## 1m/sを設定しない理由

同じ213mmの姿勢固定軌道を実際のピーク1m/sにするには五次補間時間0.399375秒、Cartesian最大加速度7.710051m/s²が必要。実URDFのFK/Jで計算した関節速度ピークはL=6.51538rad/s、F=3.42370rad/s。該当関節のURDF速度上限1rad/sを超える。

現行の実験用関節指令速度0.03rad/s・指令加速度0.15rad/s²・Cartesian加速度0.0012m/s²の枠を維持した軌道の速度目安は約4.37mm/s。URDF速度だけで求めた約0.15348m/sは、荷重・反力・遅れ・把持で合格した速度ではない。1m/s要求を現在のlaunchへ渡すと拒否する。加速度制約で実際は遅くなる軌道を1m/s達成とは扱わない。URDFや力停止値の書換えで目標を満たしたことにはしない。

## 試験と変更

最初の4mm/s試験speed_004は移動開始時の準備応答異常で安全停止。result.error=shared stop: requested stop。停止・把持は確認。保存された最初の局所異常はendpointのactuator acknowledgement not readyで、後続のgateにはlocal grasp detachedが記録された。物理接触と指反力は保持しており、当初のログだけでは最初の応答欠落の原因は未確定だった。headless再試験speed_004_retryも移動前に同じ準備応答異常で停止した（停止・把持を確認）。他のテストとの同時実行を終え、元の許可CPU集合を継承するnative_cpu_policy=inheritを用いても解消しなかった。3回目speed_004_guiもGUIあり・元のSMT設定で移動前に同じ異常で停止した。追加観測startup_transitions.jsonlと単独計算preflight_timing.jsonから、goalコールバックの同期31点IK確認が約0.165秒受信処理を止め、0.15秒のローカル受信監視を超える問題を確認した。31点確認をワーカーへ移し、ROS側は従来の期限で受信とゼロ出力を続けるよう修正した。完了時に現在の局所健全性と初期関節位置を再確認してから受理する。停止中・局所不健全・位置変化・IK失敗なら受理しない。計算自体は2秒で打ち切り判定し、既存のセンサ・permit・interlock・dt等の期限は維持した。4回目speed_004_asyncでGUI試験した。実行時の事前確認所要時間は0.297470wall秒で0.15秒を超えたが、受信処理を継続して移動を完了した。繰返し成功率を保証するものではない。

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
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true \
  forward_distance_m:=1.0 translation_velocity_m_s:=0.1 record_data:=true \
  dual_arm_distance_m:=0.213 dual_arm_speed_m_s:=0.004
```

最終4mm/s試験は別domainのGUIありで実施し、停止後に物理を一時停止してGUIを保持。元のユーザーGUIも維持。5mm/s要求は約4.37mm/sへ自動減速する。既定へ戻すには速度引数を省略する。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \
  tools/amir_dual_arm_speed_audit.py --target-speed 1.0 --output <新規JSON保存先>
```

## 証拠と残課題

target_1m_s.jsonとtarget_004m_s.jsonは計算、speed_result.jsonは実測結果。speed_comparison.png/pdf、measured_speed.csv、trial/timeseries.csv、dual_arm_evidence.jsonに全項目の合否を保存。changes.patch/source_snapshot/に変更を保存し、既存tracked差分を維持。

1m/sは未達。現在の同姿勢軌道ではURDF速度上限を超える。さらなる高速化は姿勢・軌道、実験用速度枠、加速時間、F/Tとservo帯域、摩擦把持の段階的な再検証を要する。瞬間的な指反力低下、実測関節速度の上振れ、準備応答の再現性、繰返し成功率も残課題。ベース移動による速度増加は今回行っていない。
