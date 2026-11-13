# 固定追従 — 53.6 mm/s版の復元と実行

## 保存内容

ユーザー確認済みの高速アーム追従を既存「固定追従」ブランチへ更新しました。接近・把持までは従来の台車動作を使用し、その後は両台車へ単一writerからゼロ指令を継続します。フォロワーは自身のF/T→xyアドミッタンス→ラッチしたhome+変位目標→実測FK追従誤差→2×5 DLS→既存5関節APIで追従します。リーダー状態・物体GTはフォロワー制御入力に使用しません。回転・WHOLE_BODY・base assistは無効です。

`src/`、`tools/`、既存URDF・controller設定・F/T校正・安全証拠データ、Humble JTC 2.53.1修正版を保存しています。URDF/xacro 42件は高速化前とSHA一致。ソースやモデルをバックアップのために改変していません。

`build/`、`install/`、`log/`、キャッシュ、入れ子のGit管理情報、無関係な大容量実験履歴は対象外です。最終50/53.6 mm/s headless、53.6 mm/s GUI、移動中停止の4試験と既存3/4 mm/s基準ログを保存しました。途中段階の失敗もJSON判定と報告に残しています。

`source_manifest.json`はコピー元のSHA-256、`圧縮ログ.json`は圧縮前後のSHA-256、`snapshot.json`は設定・実測・保存元コミットを記録します。

## 復元・ビルド

既存と同じ `/home/dars5070/ros2_humble_ws`、ROS 2 Humble、Gazebo Fortress 6.16.0を前提とします。校正・安全証拠には絶対パスが含まれ、別パスでの動作には再検証が必要です。現在のワークスペースがある場所にはcloneせず、空の復元先で実行してください。

```bash
git clone --branch 固定追従 https://github.com/AMIRdars/BACKUP_WS.git \
  /home/dars5070/ros2_humble_ws
cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src \
  --packages-up-to cooperative_transport_bringup amir_follower_control joint_trajectory_controller \
  --allow-overriding joint_trajectory_controller \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

依存バージョンは `環境/debian_packages.tsv`、JTC修正内容は `src/vendor/joint_trajectory_controller/README.amir.md` と `AMIR_PATCH.json` を参照してください。バックアップ時はソース・証拠の整合性を検査し、空環境からの再ビルド・シミュレーション再実行は行っていません。161件のテストと71項目の実動作判定は元環境で取得した記録です。

## GUI起動

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true record_data:=true \
  dual_arm_distance_m:=0.213 dual_arm_speed_m_s:=0.0536
```

距離・速度は上記が既定値です。従来コマンドの `forward_distance_m:=1.0 translation_velocity_m_s:=0.1` を併記しても、このモードでは把持後の台車搬送目標には使いません。終了後は停止・把持を確認して物理を一時停止しGUIを残します。GUIを閉じるか起動端末のCtrl+Cでその試験を終了します。

関節指令速度枠0.75 rad/s、関節指令加速度枠0.5 rad/s²、手先加速度枠0.05 m/s²。実URDF軌道を501点で検査して減速し、計算上限約53.64 mm/sに対し53.6 mm/sを選択しています。これはシミュレーション検証枠です。把持前に実測したサーボ証明を照合した場合だけ現在点参照のモデル検査を用い、証明なしの従来検査は維持します。先頭点を40 ms未来にする旧試作は有効化しません。

## 実測・合否

- 最終50/53.6 mm/s headless、53.6 mm/s GUI: 各71/71項目合格。
- GUI: Leader手先213.000 mm、Follower手先210.888 mm。速度ピーク53.600/57.144 mm/s。関節実速度ピーク0.349/0.197 rad/s。
- 把持点相対ずれ最大Leader 0.513 mm、Follower 1.460 mm（既存2 mm境界内）。接触継続・停止後把持保持を確認。
- 台車指令ゼロと物理停止は別評価です。世界固定治具なしで実台車ドリフト最大約0.402 mm。
- 実測50.915 mm/sでの移動中停止: 両機停止・接触継続・把持保持を専用判定で確認。意図的中断なので元trialの搬送完了判定falseは想定どおりです。
- 関連85件＋既存回帰76件＝161件合格。生ログは `tests_final_current.log/xml`、`regression_servo_gate.log/xml`。

数値・成功／失敗履歴・変更21ファイル・差分・未解決事項は `audits/dual_arm_acceleration_20261113/REPORT.md` と `release.json` に保存しています。

## 時系列・姿勢ログ

```bash
python3 固定追従/展開ログ.py --verify
python3 固定追従/展開ログ.py --extract
```

圧縮前後のSHA-256を検証し、異なる既存ファイルは上書きしません。展開後は `audits/dual_arm_acceleration_20261113/index.html` をブラウザで開くと速度・把持反力・姿勢スライダを確認できます。姿勢CSVとPNG/PDFは `posture_current/` に非圧縮で保存しています。

最終ログ:

- `audits/dual_arm_acceleration_20261113/servo_now_050/trial/`: 最終50 mm/s。
- `audits/dual_arm_acceleration_20261113/servo_now_0536/trial/`: 最終53.6 mm/s headless。
- `audits/dual_arm_acceleration_20261113/servo_now_gui/trial/`: 確認済みGUI。
- `audits/dual_arm_acceleration_20261113/servo_now_stop/trial/`: 意図的停止。専用判定 `moving_stop_evidence.json`。

各trialのtimeseries、control_samples、events、wrench_samples、jtc_state_samplesを圧縮保存しています。記録されたPID・絶対パスは履歴情報です。旧3/4 mm/sログは比較用で、現在版の合否と混同しないでください。

実測トピックは `/leader/joint_states`、`/follower/joint_states`、`/follower/ft_sensor`、`/follower/wrench_filtered`、`/follower/admittance/debug`、`/leader/arm_compensation/debug`、各ロボットの `base_cmd`・`mecanum_drive_controller/reference_unstamped`・`mecanum_drive_controller/odometry`・`arm_controller/joint_trajectory`、`grasp/monitor_debug`。GTは評価・保護observer専用です。実グラフは各trialのgraph.jsonに保存しました。

未解決: 反復成功率、長時間の動的把持耐久、さらに高い速度・加速度の検証。1 m/sは未達で、URDFは変更していません。
