# 固定追従 — 復元と実行

## 保存対象

このブランチは現在のワークスペースのソースを保存しています。`src/`、`tools/`、必要なURDF・controller設定・F/T校正・安全根拠データ、JTC 2.53.1の既存修正版を含みます。モデル・ゲイン・APIをバックアップのために変更していません。

`build/`、`install/`、`log/`、キャッシュ、入れ子のGit管理情報、無関係な大容量実験履歴はバックアップの対象外です。記録済みの3mm/sと4mm/sのCSV/JSONLはgzip圧縮して同じ監査パスへ保存しました。`source_manifest.json`は保存元ファイルのSHA-256、`圧縮ログ.json`は圧縮前後のSHA-256、`snapshot.json`は設定と保存元コミットを記録しています。

## 復元条件

既存実行と同じ `/home/dars5070/ros2_humble_ws`、ROS 2 Humble、Gazebo Fortress 6.16.0の環境を前提とします。旧モデル・安全証拠ファイルにはこの絶対パスが含まれています。**別のディレクトリにcloneするだけではパス互換性を保証しません。** 別パスで動かす場合は、コントローラ参照・プラグイン参照・証拠ハッシュを整合させた再検証が必要です。元のモデル証拠を黙って書き換える処理は含めていません。

現在の作業ディレクトリを上書きせず、空の復元先にcloneしてください。元のディレクトリが存在すれば、次のcloneは安全に失敗します。

```bash
git clone --branch 固定追従 https://github.com/AMIRdars/BACKUP_WS.git   /home/dars5070/ros2_humble_ws
cd /home/dars5070/ros2_humble_ws
```

## ビルド

新しい端末でROS 2 Humbleをsourceします。必要なapt依存は `環境/debian_packages.tsv` に保存しています。ROS/Gazeboを新しい版へ更新する手順ではありません。

```bash
cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src   --packages-up-to cooperative_transport_bringup amir_follower_control joint_trajectory_controller   --allow-overriding joint_trajectory_controller   --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

同梱の `joint_trajectory_controller` はHumble 2.53.1で、2台間の関節コマンド混入を修正した既存版です。`src/vendor/joint_trajectory_controller/README.amir.md` と `AMIR_PATCH.json` を参照してください。実行時にはマーカーと実際に読み込まれたライブラリが照合されます。

## 起動: 既定3mm/s

他のGazeboを終了してから実行してください。把持のための接近では両ベースが動き、把持後は両ベースの最終速度指令をゼロにします。世界固定治具による拘束ではなく、物理的な微小ずれは別途監視・記録します。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py   headless:=false gui_nvidia_offload:=true   forward_distance_m:=1.0 translation_velocity_m_s:=0.1 record_data:=true   dual_arm_distance_m:=0.213 dual_arm_speed_m_s:=0.003
```

既存の `forward_distance_m:=1.0 translation_velocity_m_s:=0.1` はこのモードの把持後ベース走行には使用しません。回転・WHOLE_BODY・base assistは有効にしていません。

## 起動: 検証済み4mm/s比較

上のコマンドの速度を `dual_arm_speed_m_s:=0.004` に変更します。既定値自体は3mm/sです。213mmの移動完了・接触継続・停止後把持保持は確認していますが、指反力下限の4判定が未達で厳密評価は67/71項目合格です。全項目合格や繰返し成功率の保証とは区別してください。

終了後は停止・把持を確認して物理を一時停止し、GUIを残します。GUIを閉じるか起動端末でCtrl+Cを押すと、その試験のプロセスを終了します。

## 単体・回帰試験

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=191
export PYTHONPATH="$PWD/tools:$PWD/src/humbleble_ws/amir_compliance_control/test:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q   tools/test_arm_preflight.py tools/test_dual_arm_speed_audit.py   tools/test_dual_arm_range.py tools/test_dual_arm_only.py   tools/test_native_arm_stroke.py tools/test_af_cpu_affinity.py
```

元環境では45件合格。バックアップ作成時の確認結果は `backup_verification.json` に保存します。実機や新しいROS/Gazeboでの合格を意味しません。

## 保存した時系列ログ

```bash
python3 固定追従/展開ログ.py --verify
python3 固定追従/展開ログ.py --extract
```

検証は圧縮・展開後のSHA-256を照合します。展開は記録された監査パスだけを対象とし、異なる既存ファイルを上書きしません。展開後は `tools/amir_dual_arm_evidence.py --case <試験のtrialディレクトリ> --held` で保存時の結果を再評価できます。保存済みのPID・実行パスは履歴情報で、起動中のプロセスを示すものではありません。

代表的な記録:

- `audits/dual_arm_range_20261112/max_213_smooth/trial/`: 3mm/s基準。
- `audits/dual_arm_speed_20261113/speed_004_async/trial/`: 非同期IK事前確認後の4mm/s試験。
- `audits/dual_arm_speed_20261113/REPORT.md`: 失敗した開始試験、修正、実測速度、残課題。

1m/sへのURDF・制御上限の引き上げは行っていません。保存した状態はベース固定追従の版です。
