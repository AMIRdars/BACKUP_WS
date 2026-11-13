# 接近後、両台車停止・両アーム協調追従

`compliance_round_trip.launch.py` の既定 `ARM_ONLY / dual_arm` は、車輪で接近後に両台車を停止し、リーダーアームの最大213 mm水平移動へフォロワーアームが自身のF/Tから追従するGazebo実験です。速度の既定値を3 mm/sから53.6 mm/sへ変更しました。URDF/xacro、既存controller/API、力・接触・関節実速度・台車停止の監視閾値は変更していません。高速な補償参照の検査には、実測サーボ応答を使う明示的な検査方式を追加しました。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select amir_compliance_control cooperative_transport_bringup
source install/setup.bash

ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true \
  forward_distance_m:=1.0 \
  translation_velocity_m_s:=0.1 record_data:=true
```

このモードでは旧走行引数 `forward_distance_m` と `translation_velocity_m_s` は走行目標に使用しません。既定は `dual_arm_distance_m:=0.213`、`dual_arm_speed_m_s:=0.0536` です。速度は五次補間のピーク要求であり、一定速度ではありません。実URDFの連続IKで求めた共通水平可動域を超える距離は拒否します。小距離の回帰試験は `dual_arm_distance_m:=0.03` で実行できます。往復は行いません。

高速側のソフト設定は関節指令速度0.75 rad/s、関節指令加速度0.5 rad/s²、手先加速度0.05 m/s²です。URDFの定格を書き換えた値ではなく、段階試験で明示したシミュレーション設定です。両アームのURDFから速度・加速度を501点で計算し、設定枠の95%を超える要求は減速します。この枠の計算上限約53.64 mm/sに対し53.6 mm/sを採用しています。フォロワーの局所アドミッタンスはx速度上限0.1072 m/s、x加速度上限0.1 m/s²を使用し、M/D/Kと力・把持停止境界は維持します。

最終の現在点サーボ検査方式で、50 mm/s・53.6 mm/sの213 mm搬送に合格しました。53.6 mm/sはGUIでも搬送を完了し、接触継続・停止後把持保持を含む厳密71/71項目を検査しています。以前の方式では50 mm/sの2 mmずれ停止や、約53.64 mm/sの先頭点検査停止があり、両者とも把持は保持されました。これらの失敗も `audits/dual_arm_acceleration_20261113/REPORT.md` に残します。

15 mm/sの完走試験2回では接触は継続しましたが、指反力の単発下振れにより厳密判定は67/71でした。以前の3〜4 mm/s試験でも同様の下振れが記録されています。全試験が合格したとは扱いません。反力の生値、接触、実ずれ、停止後把持を別々に記録しています。

従来の3 mm/sと元のソフト枠を明示して再現する場合:

```bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  dual_arm_speed_m_s:=0.003 dual_arm_joint_speed_rad_s:=0.03 \
  dual_arm_joint_acceleration_rad_s2:=0.15 dual_arm_acceleration_m_s2:=0.0012
```

動かさずに速度目標を分析する場合:

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \
  tools/amir_dual_arm_speed_audit.py --target-speed 1.0 \
  --output <未作成の分析JSON保存先>
```

53.6 mm/s設定の解析は `--target-speed .0536 --joint-speed .75 --joint-acceleration .5 --acceleration .05` を指定します。URDF上限だけで求めた速度は把持搬送の合格速度ではありません。1 m/s要求は起動前に拒否します。

開始時の31点IK事前確認は別スレッドで実行し、受信期限を延長しません。先頭点を未来40 msへ配置する案は、更新周期に依存する再補間遅れを生じたため最終設定では使用しません。補償した現在点の位置参照に対し、各自の把持前実測から得たtauで(q_ref−q_actual)/tauを検査し、後続点の参照傾きと共に同じURDF速度上限へ制限します。受信時刻に進んだ区間は補間した参照を検査します。5関節の応答・URDF digest・プラグインSHA・open_loop設定が合致し、両台車ゼロの明示的なGazebo実験である場合だけ有効です。従来モードは元の検査を維持し、関節実速度・反力・位置・接触の独立監視も残しています。

Humble 2.53.1 JTCの既存ワークスペース修正版が必要です。未ビルドの場合のみ先に実行してください。

```bash
colcon build --packages-select joint_trajectory_controller \
  --allow-overriding joint_trajectory_controller --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

INITで把持前10 mmアーム単体試験を実施して復帰し、BASE_APPROACH → BASE_STOP → DUAL_GRASP → SETTLE_AND_BIAS → ARM_ONLY → HOLDを進みます。異常時は共有停止し、グリッパを開きません。把持後に両台車へ出す最終速度はゼロに限定します。世界固定治具は使用しません。台車の物理的な微小ずれはwheel odomとは別に記録します。

フォロワーは既存 `StrictArmOnly` を流用します。自身のWrist F/TをTCP参照点・台車座標軸へ変換し、静止荷重をtare後、`M=0.5 kg, D=8.65 Ns/m, K=0`の速度型アドミッタンスと2×5 DLSを使用します。仮想M・Dは従来値の1/4にして応答時定数M/Dを維持し、追従に必要な反力を抑えます。`dual_arm_admittance_scale:=1.0`で従来M=2,D=34.6に戻せます。3 mN死域は過去の静止ログに基づく暫定実験設定です。両機のアームAPI、関節名、Gazebo物理設定を継承しています。

終了後は停止・把持確認して物理を一時停止し、GUIを残します。GUIを閉じるかCtrl+Cで終了します。長時間の動的把持耐久試験にはなりません。出力先は起動ログの`Dual arm output`です。

```bash
# 記録された試験を再評価（GUI保持中）
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \
  tools/amir_dual_arm_evidence.py --case <出力先>/trial --held

# GUIを閉じた試験は --held を付けずに再評価
# この試験だけのGazeboウィンドウを動画化する手順
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/amir_gazebo_window_video.py \
  --case <出力先>/trial --output <新規保存先>.avi \
  --include-preparation --duration-s 900 --fps 2
```

以前のモードは `arm_only_trial:=native_stroke` または `arm_only_trial:=micro` で選択できます。`arm_only_experiment.enabled:=false` は旧native_strokeを選択します。`control_mode:=WHOLE_BODY`は既存アーカイブの検証経路を使用します。今回、台車追従・回転は動かしません。

最新の実測値、成功・失敗ログ、差分、残課題は `audits/dual_arm_acceleration_20261113/REPORT.md` を参照してください。可動域の監査は `audits/dual_arm_range_20261112/REPORT.md`、以前の3〜4 mm/s比較は `audits/dual_arm_speed_20261113/REPORT.md` に残しています。力・トルクの実験停止境界は既存設定を継承しており、実機の校正済み安全定格ではありません。
