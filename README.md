# 固定追従

ROS 2 Humble + Gazebo FortressのAMIR740協調搬送。接近・摩擦把持後は両ベースの速度指令をゼロに保ち、リーダーアームの動きへフォロワーアームが自身のF/Tから追従します。ユーザーが動作確認した **53.6 mm/s版** のバックアップです。

- 既定: アーム移動213 mm、ピーク要求53.6 mm/s（従来3 mm/sの約17.9倍）。URDF変更なし。
- 最終GUI・headlessとも厳密評価71/71項目合格。接触継続・停止後把持保持、移動中停止を検証。
- GUI実測速度ピークLeader 53.600 mm/s、Follower 57.144 mm/s。五次補間のため一定速ではありません。
- 関連単体・回帰161件合格。F/T・実関節・接触・台車ドリフトの監視を維持。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true record_data:=true \
  dual_arm_speed_m_s:=0.0536
```

[復元・ビルド・ログ展開手順](固定追従/README.md) / [検証報告・残課題](audits/dual_arm_acceleration_20261113/REPORT.md) / [速度・把持グラフ](audits/dual_arm_acceleration_20261113/release_comparison.png) / [姿勢ログ](audits/dual_arm_acceleration_20261113/posture_current/REPORT.md)

HTMLの姿勢スライダはclone後にローカルで開いてください。元CSV/JSONLはgzip圧縮して保存しています。3〜4 mm/s版は過去コミットと比較ログに残しています。
