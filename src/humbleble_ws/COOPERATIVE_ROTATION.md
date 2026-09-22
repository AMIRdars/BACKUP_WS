# 指定軸まわり協調回転

`cooperative_rotation` は、把持開始時の搬送物と2つのTCPの相対変換を保存し、同一の
搬送物目標から2つのTCP目標を生成します。Gazeboの物体Poseは直接変更しません。

現在のAMIRシミュレーションはアームを固定したまま台車を同期旋回させる構成です。
そのため、生成した搬送物目標を既存の `cooperative_coordinator` へ連続送信し、同じ
角度サンプルから両台車を駆動します。任意軸の数学処理は実装済みですが、現在の
平面台車による実動作はワールドZ軸に限定しています。

## ビルドと10度テスト

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cooperative_transport_bringup
source install/setup.bash

ros2 launch cooperative_transport_bringup rotation_simulation.launch.py \
  headless:=false \
  target_angle_deg:=90.0 \
  angular_velocity_deg_s:=2.0
```

進捗確認:

```bash
ros2 topic echo /cooperative_rotation/status
ros2 topic echo /cooperative_rotation/complete --once
```

手動開始する場合は `auto_start:=false` で起動し、把持完了後に次を実行します。

```bash
ros2 service call /cooperative_rotation/start std_srvs/srv/Trigger '{}'
```

停止:

```bash
ros2 service call /cooperative_rotation/cancel std_srvs/srv/Trigger '{}'
```

30度、90度もlaunch引数だけで変更できます。

```bash
ros2 launch cooperative_transport_bringup rotation_simulation.launch.py \
  target_angle_deg:=30.0 angular_velocity_deg_s:=2.0
```

RVizでは `/cooperative_rotation/markers` を `MarkerArray` として追加すると、緑の
搬送物目標、赤・青のTCP目標、黄色の回転軸を確認できます。

## 接触・摩擦把持による回転（Stage C）

`friction_rotation_simulation.launch.py` は `DetachableJoint` と
`FrictionGripSystem`の両方を使わず、4指の接触と摩擦だけで搬送物を保持してから
回転します。初期試験値は5度、1度/秒です。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup \
  friction_rotation_simulation.launch.py \
  headless:=false target_angle_deg:=5.0 angular_velocity_deg_s:=1.0
```

回転ノードは次の全条件が揃うまで指令を出しません。

- 両グリッパの物理把持シーケンスが `HOLDING`
- 左右4指で搬送物との接触と法線反力が継続している
- 把持前の仮支持台が撤去済み
- 両ロボットの台車・アームコントローラがactive
- 搬送物Pose、オドメトリ、TFが取得可能
- 非常停止が解除済み

確認用トピック:

```bash
ros2 topic echo /cooperative_transport/friction_grasp_status
ros2 topic echo /cooperative_transport/contact_status
ros2 topic echo /cooperative_transport/support_removed
ros2 topic echo /cooperative_transport/slip_status
ros2 topic echo /cooperative_transport/safety_status
ros2 topic echo /cooperative_rotation/status
```

## Excelによる角度・把持力記録

`friction_rotation_simulation.launch.py`は既定で回転開始から完了までを10 Hzで記録し、
`~/ros2_humble_ws/rotation_measurements/`へExcelファイルを自動保存します。

Excelの`概要`シートには最終角度、最大追従誤差、4把持点の法線力統計と2つのグラフ、
`時系列データ`シートには次の値が保存されます。

- 最終目標角度、時刻ごとの指令角度、現在角度、角度誤差
- AMIR1・AMIR2の左右指におけるFx、Fy、Fz、合力、法線力`|Fx|`

出力先と記録周波数はlaunch引数で変更できます。

```bash
ros2 launch cooperative_transport_bringup friction_rotation_simulation.launch.py \
  headless:=false target_angle_deg:=90.0 angular_velocity_deg_s:=2.0 \
  record_data:=true recording_rate:=10.0 \
  excel_output_directory:=$HOME/ros2_humble_ws/rotation_measurements
```

実行中に手動保存する場合は次を使用します。

```bash
ros2 service call /cooperative_rotation/save_excel std_srvs/srv/Trigger '{}'
```

保存先はログと`/cooperative_rotation/excel_report`トピックにも出力されます。

手首力覚値は常時表示されますが、現在のGazebo把持モデルでは把持予圧を含むため、
閾値停止は初期値で無効です。有効化する場合は
`enable_wrench_safety:=true`を指定します。滑り監視、把持解除監視、台車速度・間隔・
関節限界監視は初期値のまま有効です。

5度試験が成功してから、10度、20度、30度の順で拡大してください。接触把持では
固定把持より滑りと整定時間の影響が大きいため、90度を最初から指定しないでください。

長角度回転では、指定速度に対して台車の接線速度が不足すると目標角だけが先行します。
回転コントローラは姿勢追従誤差が6度に達すると目標軌道を一時停止し、3度まで回復後に
再開します。また軌道・整定時間にはROS時刻を用いるため、Gazeboの実時間係数が低下しても
目標速度がシミュレーションに対して速くなりません。終端では摩擦把持のねじれ残差に対し、
最大4度の台車目標補正を行います。15度の追従異常停止判定と物理把持の安全監視は維持されます。

2026-09-04の複合指形状によるヘッドレス5度試験では、4指の法線反力が
約722--777 Nで`contacts=verified`となった後に支持台を撤去した。搬送物高さは
支持台撤去前の`z=0.474 m`から、撤去・回転後も`z=0.477 m`を維持した。
回転結果は`complete=true`、実角度`4.34 deg`（許容差内）だった。

2026-09-04の90度・2度/秒試験では、追従ガバナが周期的に目標進行を待機させながら
全角度を走行し、終端補正後に`complete=true`、実角度`89.95 deg`で完了した。
最終状態は`HOLDING`、`contacts=verified`、安全監視`OK`で、搬送物高さは
`z=0.4771 m`、鉛直ずれは約3.6 mmだった。
