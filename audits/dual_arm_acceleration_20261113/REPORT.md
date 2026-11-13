# URDF不変・ベース停止アーム追従の高速化（2026-11-13）

既定のピーク要求を3→53.6 mm/s（約17.87倍）へ変更した。距離は従来の213 mm。両台車は接近・把持後に単一writerからゼロ指令を継続する。URDF/xacro 42ファイルは変更していない。

現在点のサーボ応答を検査する最終方式で、50 mm/sと53.6 mm/sのheadless試験を完走し71/71項目合格。最終GUI試験も71/71。移動中の専用停止試験で両機の実停止・接触継続・停止後把持保持を確認。

## 最終GUI実測

| 指標 | Leader | Follower |
|---|---:|---:|
| 搬送方向FK速度ピーク [mm/s] | 53.600124 | 57.144180 |
| 同99.5百分位 [mm/s] | 53.594414 | 54.251807 |
| 関節実速度ピーク [rad/s] | 0.349251 | 0.196961 |
| 実速度/URDF上限の最大比 | 0.349251 | 0.196961 |
| ARM_ONLY中の台車GTドリフト最大 [mm] | 0.402079 | 0.402093 |

手先x変位Leader 213.000131 mm、Follower 210.887765 mm。増分F/T最大 4.190847 N、フォロワー追従誤差最大 0.057933 mm。

把持点相対ずれ最大[mm]: {'leader': 0.5131303701222247, 'follower': 1.460178260579079}（既存2 mm境界以内）。

四指の反力範囲[N]: {'leader_left_normal_force_range_N': [16.129121596479536, 18.511890850248527], 'leader_right_normal_force_range_N': [19.535017331792407, 21.36412180192061], 'follower_left_normal_force_range_N': [15.97037590915103, 17.639276159901303], 'follower_right_normal_force_range_N': [19.594124332023988, 20.59986118405211]}

速度は実測JointState時刻に対応するFKの差から算出した搬送方向x成分の区間平均。重複時刻を除去した。五次補間なので一定速度ではない。wall時間とsim時間を同一視しない。台車指令ゼロ・wheel odom速度・物理GTドリフトも別々に評価した。世界固定治具なし。

## 実装

リーダーのソフト関節速度/加速度をパラメータ化し、両アームの実URDFの同じ姿勢固定軌道を501点で計算して、速度・加速度枠の95%以内へ減速する。最終の関節指令速度枠0.75 rad/s、関節指令加速度枠0.5 rad/s²、手先加速度枠0.05 m/s²。これらは今回のシミュレーション探索枠であり、URDFにない実機加速度定格を推定した値ではない。この枠の計算上限約53.64 mm/sに対して53.6 mm/sを採用した。さらに高い加速度枠は未検証。

フォロワーは自身のF/T→xyアドミッタンス→home+変位目標→FK誤差→2×5 DLS→同じ5関節position APIを継承。M=0.5 kg、D=8.65 Ns/m、K=0、F/T 8 Hz、3 mN死域を維持。高速用の固定上限としてx速度2×要求ピーク、x加速度2×手先加速度（従来値が大きければ従来値）を設定する。リーダー時系列や物体GTの目標を送らない。

## 安全ゲートの変更範囲

4 msで補償位置へ到達するという幾何学的な解釈では、高速なpositionサーボの現在点参照を誤って拒否していた。先頭点を未来40 msへ移す案は完走例もあったが、更新タイミング依存の再補間遅れで5 mmの追従監視停止を生じたため、最終方式では有効にしない。

最終方式は、把持前に各自で実測した5関節の応答証明を読み込み、URDF digest・プラグインSHA・5関節応答・open_loop設定を照合する。両台車ゼロのGazebo実験で明示的に証明ファイルを与えた場合だけ、現在点参照に対し(q_ref−q_actual)/tauと後続参照傾きを同じURDF速度上限で検査する。受信時刻に既に進んだ区間は補間した現在参照で検査する。閾値やURDFを増やす代わりに、実測したアクチュエータモデルに合う検査を追加した。

関節実速度・位置・反力・F/T・受信/dt・把持接触・2 mm相対ずれ・台車停止の監視は維持。RobotSafetyGate.healthy/base_command/grasp_state/clock/emergencyのASTが開始時と同一、interlock/native_grasp/grip_maintenance/transport/wrench/arm_servo_responseはbyte一致。SafetyGate.arm_commandだけは上記の明示的なモデル検査を追加した変更対象。証明ファイルなしの従来モードは元の幾何学検査を使用する。

最終ゲートのモデル速度/URDF上限ピーク比: {'follower': {'servo_streaming_model_enabled': True, 'servo_reference_URDF_velocity_ratio_peak': 0.2153674636421965}, 'leader': {'servo_streaming_model_enabled': True, 'servo_reference_URDF_velocity_ratio_peak': 0.384172029663693}}

## 全段階・失敗記録

| Case | 搬送完了 | 接触継続 | 停止後把持 | 厳密判定 | error |
|---|---|---|---|---|---|
| baseline_005 | True | True | True | 71/71 | None |
| moving_stop_050 | False | 搬送未実行 | True | 未評価/— | single-writer violation |
| moving_stop_050_retry | False | True | True | 未評価/— | shared stop: requested stop |
| release_gui | True | True | True | 71/71 | None |
| servo_now_050 | True | True | True | 71/71 | None |
| servo_now_0536 | True | True | True | 71/71 | None |
| servo_now_gui | True | True | True | 71/71 | None |
| servo_now_stop | 意図的な途中停止 | True | True | 未評価/— | requested stop signal: SIGINT |
| speed_008 | True | True | True | 71/71 | None |
| speed_015 | False | True | True | 66/71 | shared stop: leader: arm command violates model limits |
| speed_015_repeat | True | True | True | 67/71 | None |
| speed_015_retry | True | True | True | 67/71 | None |
| speed_025 | True | True | True | 71/71 | None |
| speed_035 | True | True | True | 71/71 | None |
| speed_050 | False | True | True | 65/71 | shared stop: requested stop |
| speed_050_retry | True | True | True | 71/71 | None |
| speed_075 | False | True | True | 66/71 | shared stop: leader: arm command violates model limits |

旧4 ms先頭点の15 mm/sはゲート停止。未来点方式の15 mm/s完走2件は生反力の単発下振れで67/71（接触・停止後把持は継続）。旧50 mm/sは2 mmずれ停止、加速度飽和を調整して再試験で完走。未来点方式の53.64 mm/sはモデル速度検査で停止。旧方式の完走GUIも採用せず、現在点のサーボ検査方式で再検証した。失敗例も削除していない。

移動中停止の初回moving_stop_050はwriter発見0件で移動前中止（重複ではない）。moving_stop_050_retryは未来点方式の5 mm追従偏差で停止し、停止信号は未注入。双方とも把持を保持した。最終servo_now_stopは現在点方式で実測高速状態を確認し、自分が起動したlaunch PIDだけへSIGINTを送って検証した。

最終停止試験: 実測FK速度 50.914731 mm/s、参照 50.967166 mm/sで停止要求。両機実停止・接触継続・停止後把持・gripper active=True。途中停止が目的なので元trialの搬送完了passed=Falseは想定どおり。専用判定はpassed=True。

## 変更・ビルド・テスト

- `src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_only_leader.py`
- `src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_trajectory_bounds.py`
- `src/humbleble_ws/amir_compliance_control/amir_compliance_control/safety.py`
- `src/humbleble_ws/amir_compliance_control/amir_compliance_control/strict_arm_only.py`
- `src/humbleble_ws/amir_compliance_control/launch/arm_only_af.launch.py`
- `src/humbleble_ws/amir_compliance_control/launch/guarded_safety.launch.py`
- `src/humbleble_ws/cooperative_transport_bringup/README_dual_arm_only.md`
- `src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py`
- `tools/amir_af_trial.py`
- `tools/amir_dual_arm_speed_audit.py`
- `tools/dual_arm_only_runtime.py`
- `tools/dual_arm_workspace.py`
- `tools/test_dual_arm_only.py`
- `tools/test_dual_arm_range.py`
- `tools/test_dual_arm_speed_audit.py`
- `tools/dual_arm_speed_policy.py`
- `tools/amir_dual_arm_speed_results.py`
- `tools/test_dual_arm_speed_policy.py`
- `tools/test_future_arm_knots.py`
- `tools/test_servo_streaming_gate.py`
- `src/humbleble_ws/amir_compliance_control/amir_compliance_control/servo_streaming_gate.py`

最終ビルド成功。関連85件、既存回帰76件、計161件合格。tests_final_current.log/xmlとregression_servo_gate.log/xml。初期のテスト環境import不足・新テストのクラス名誤記は修正済み。最終実行は全件合格。

before/は開始時61ファイルを保存。3テストの変更前実体は既存固定追従バックアップから補完（test_baseline_provenance.json）。changes.json、implementation.patch、source_snapshot/に最終差分を保存。最終GUIの試験ソース82件と現コードは全てSHA一致。URDF不変確認はurdf_unchanged.json。

## 実行

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true record_data:=true \
  dual_arm_speed_m_s:=0.0536
```

元のforward_distance_m:=1.0 / translation_velocity_m_s:=0.1付きコマンドもARM_ONLYの新既定を使う。旧台車走行引数を把持後の走行目標へ使用しない。従来3 mm/s枠へ戻す場合はdual_arm_speed_m_s:=0.003、dual_arm_joint_speed_rad_s:=0.03、dual_arm_joint_acceleration_rad_s2:=0.15、dual_arm_acceleration_m_s2:=0.0012を指定する。

実測topic: /leader/joint_states、/follower/joint_states、/follower/ft_sensor、/follower/wrench_filtered、/follower/admittance/debug、/leader/arm_compensation/debug、各/<robot>/mecanum_drive_controller/odometry、base_cmd、mecanum_drive_controller/reference_unstamped、arm_controller/joint_trajectory、grasp/monitor_debug。graph.jsonで実在と単一writer、フォロワー購読境界を検査した。GTは評価・保護observer専用。

## 図と時系列

[速度・把持比較PNG](release_comparison.png) / [PDF](release_comparison.pdf)、[全段階比較](speed_comparison.png)、[速度CSV](measured_speed.csv)、[姿勢スライダ](posture_current/index.html)、[GUI](release_current_gui.png)。各trialのtimeseries.csv、events.jsonl、wrench_samples.csv、control_samples.jsonl、jtc_state_samples.jsonlが元データ。

残課題: 旧段階で観測した生反力の単発下振れ、統計的反復成功率と長時間の動的把持耐久、さらに高い加速度・速度の検証。有限回のGazebo合格を全将来試験の把持保証や実機定格とはしない。1 m/sは未達。今回のURDF・Gazebo・Humbleとcontroller/APIは維持した。
