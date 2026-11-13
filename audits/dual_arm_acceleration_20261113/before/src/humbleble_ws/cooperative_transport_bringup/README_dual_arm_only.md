# 接近後、両台車停止・両アーム協調追従

`compliance_round_trip.launch.py` の既定 `ARM_ONLY / dual_arm` は、車輪で接近後に両台車を停止し、リーダーアームの最大213 mm水平移動へフォロワーアームが自身のF/Tから追従するGazebo実験です。

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

このモードでは旧走行引数 `forward_distance_m` と `translation_velocity_m_s` は走行目標に使用しません。既定は `dual_arm_distance_m:=0.213`、要求速度上限は `dual_arm_speed_m_s:=0.003` です。実URDFの連続IKで求めた共通水平可動域を超える距離は拒否します。関節速度0.03rad/sを守るため、要求速度が高い場合は軌道を自動で減速します。小距離の回帰試験は `dual_arm_distance_m:=0.03` で実行できます。往復は行いません。

213mmでは動作完了・接触継続・停止後把持保持を確認しましたが、最終ログの指反力が1サンプルで既存8N下限を下回り、独立評価は67/71項目合格です。全項目合格は未達であり、瞬間的な反力低下の原因は報告書の残課題に記録しています。

速度比較は起動引数 `dual_arm_speed_m_s:=0.004` で行えます。既定の3mm/sは維持しています。5mm/s要求は関節速度に合わせて約4.37mm/sへ自動減速し、1m/s要求は起動前に拒否します。同じ213mmの姿勢固定軌道を実際のピーク1m/sにすると、計算上リーダー関節速度が約6.52rad/sとなり、該当関節のURDF上限1rad/sを超えます。URDF上限だけで求めた速度は接触搬送の合格速度ではありません。

動かさずに速度目標を分析する場合:

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \
  tools/amir_dual_arm_speed_audit.py --target-speed 1.0 \
  --output <未作成の分析JSON保存先>
```

速度比較の実測・合否は `audits/dual_arm_speed_20261113/REPORT.md` に保存します。

4mm/sのGUI試験では213mmの移動と停止後把持保持を確認し、厳密判定は67/71項目合格でした。指反力の下限条件は未達です。開始時の31点IK事前確認が受信処理を止める問題は、同じ確認を別スレッドへ移して修正しました。反映には `colcon build --packages-select amir_compliance_control` 後、`source install/setup.bash` を実行します。関節指令速度上限と実測関節速度は別々に記録しており、瞬間的な実測速度の上振れも報告書に記載しています。

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

監査表、実測値、成功・失敗ログ、差分、残課題はワークスペースの `audits/dual_arm_range_20261112/REPORT.md` を参照してください。力・トルクの実験停止境界は既存設定を継承しており、実機の校正済み安全定格ではありません。
