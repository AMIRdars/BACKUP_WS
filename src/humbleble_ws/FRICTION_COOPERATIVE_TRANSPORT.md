# 協調搬送・段階B/C（接触・摩擦把持）実行手順

軌道の自動実行とCSV評価へ進む場合は、`TRAJECTORY_EVALUATION.md`を参照する。

## 1. この段階で実装したもの

段階Aの `DetachableJoint` 版を残したまま、固定ジョイントを一切使わない段階Bを追加した。段階Bでは、AMIRの左右指と搬送物の接触力・摩擦だけで物体を保持する。

- 1 kg、長さ1.20 m、把持厚さ30 mmの摩擦把持用搬送物
- 搬送物を把持前だけ支える中央台座
- 両グリッパを開く、閉じる、予圧保持するシーケンス
- `Joint_2=0.10 rad`、`Joint_3=-0.10 rad`による約23 mmの実持上げ
- 搬送物と2台編隊の相対姿勢から滑りを検出する監視ノード
- 滑り検出を既存の2台同時安全停止へ接続
- Fortressで接触計算を安定・高速化する指先の単純衝突形状
- 左右4指のcontact sensorと指関節のforce/torque sensor
- 4指すべての搬送物接触と法線反力を確認してから成立する`HOLDING`
- 目標20 Nの閉ループ把持力制御、1.3 N mの通常指令上限、1.4 N mのURDF絶対上限
- 把持成立位置より回転中に開かないアンチスリップ制約（90 N超では安全開放）
- 把持直後の反力をゼロ点とする補正wrenchとExcelへの生値・補正値記録

指先は元のSTLのテーパー輪郭を9個のboxで近似している。各boxのvisualとcollisionは
位置・寸法が同一であり、9区間すべてにcontact sensorを割り当てている。したがって、
画面上の指とGazeboが接触判定する指形状に差はない。

段階Bの既存worldには比較試験用の`FrictionGripSystem`が残る。段階Cでは
`contact_only:=true`を指定し、この補助力プラグインを含まないworldを使用する。

## 2. ビルド

新しい端末で以下を実行する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  amir_description \
  cooperative_transport_description \
  cooperative_transport_gazebo \
  cooperative_transport_control \
  cooperative_transport_bringup

  
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. シミュレーション起動

GUIあり:

```bash
ros2 launch cooperative_transport_bringup \
  friction_simulation.launch.py
```

ヘッドレス:

```bash
ros2 launch cooperative_transport_bringup \
  friction_simulation.launch.py headless:=true contact_only:=true
```

起動後は自動的に次の順で進む。

1. `amir1`と`amir2`を生成する。
2. 両グリッパを `-1.0 rad` まで開く。
3. 台座上に搬送物を生成する。
4. 物体を生成する前から両グリッパを開位置 `-1.0 rad` にし、起動時衝突を防ぐ。
5. 両グリッパを小刻みに閉じ、左右接触後は各指約20 Nになるよう位置を補正する。
6. 左右4指のcontact sensorが指と搬送物の接触を検出する。
7. 各指のforce/torque sensorで把持面法線方向の反力が閾値以上か確認する。
8. 接触と反力が0.30秒安定した場合だけ、両アームを同期して持ち上げる。
9. 持上げ後も4指の条件を満たす場合だけ`HOLDING`にし、反力バイアスと把持位置下限を保存する。
10. `HOLDING`後に仮支持台を撤去し、搬送物相対姿勢を滑り監視の基準にする。

`DetachableJoint`のプラグイン、attach topic、attach serviceはこのlaunchでは使用しない。

## 4. 把持完了の確認

別端末で次を実行する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /cooperative_transport/friction_grasp_status
```

次のような表示になれば搬送指令を出せる。

```text
HOLDING; held=True; contacts=verified; grippers: amir1=-0.341, amir2=-0.335; normal_forces: ...
```

接触判定だけを短く確認する場合:

```bash
ros2 topic echo /cooperative_transport/contact_status
ros2 service call /cooperative_transport/friction_grasp_status \
  std_srvs/srv/Trigger '{}'
```

正常時は`CONTACT_OK`で、service応答には4指の法線反力がN単位で表示される。
`hold_floors`は回転中にこれ以上グリッパーを開かない位置下限である。

安全系も確認する。

```bash
ros2 topic echo --once /cooperative_transport/slip_status
ros2 topic echo --once /cooperative_transport/safety_status
ros2 topic echo --once /cooperative_transport/state
```

正常時の期待値は順に `OK`、`OK`、`WAITING_FOR_GOAL` である。

## 5. 低速直進

DDS discoveryで一回だけのpublishを取りこぼさないよう、目標を3回送信する。

```bash
ros2 topic pub -r 2 -t 3 \
  /cooperative_transport/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.10, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

ros2 service call /cooperative_transport/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

この初期設定では最大速度を `0.035 m/s`、最大角速度を `0.040 rad/s` に制限している。最初は必ず10 cm程度の直進から確認する。

今回の実機環境上の検証結果は次の通りだった。

- 搬送前の搬送物: `x=0.00045 m`, `z=0.46554 m`
- 搬送後の搬送物: `x=0.08058 m`, `z=0.46548 m`
- ロボット中点: `x=0.08013 m`
- 横・鉛直滑り監視: `OK`
- 安全監視: `OK`

目標より約20 mm手前で止まるのは、初期設定の `position_tolerance=0.02` による。

### 5.1 20 N把持で90度回転しExcelへ記録

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  cooperative_transport_bringup cooperative_rotation
source install/setup.bash

ros2 launch cooperative_transport_bringup \
  friction_rotation_simulation.launch.py \
  headless:=false \
  target_angle_deg:=90.0 \
  angular_velocity_deg_s:=2.0 \
  target_normal_force:=17.0 \
  gripper_maximum_effort:=1.3 \
  record_data:=true
```

回転完了後、`~/ros2_humble_ws/rotation_measurements`にExcelファイルが生成される。
「概要」シートには角度・生法線力・バイアス補正後の変動量とグラフ、
「時系列データ」シートには全46列のサンプルが入る。補正値は把持力そのものではなく、
`HOLDING`成立直後を0 Nとした荷重変化である。

## 6. 搬送停止と把持解除

先に台車を停止し、その後に搬送物を台座へ下ろして指を開く。

```bash
ros2 service call /cooperative_transport/set_enabled \
  std_srvs/srv/SetBool "{data: false}"

ros2 service call /cooperative_transport/friction_grasp \
  std_srvs/srv/SetBool "{data: false}"
```

`friction_grasp_status` が `RELEASED; held=False` になれば完了である。搬送後は中央台座から離れているため、床や別の設置台の上で解除すること。

自動把持を無効にして手動で開始する場合:

```bash
ros2 launch cooperative_transport_bringup \
  friction_simulation.launch.py auto_grasp:=false

ros2 service call /cooperative_transport/friction_grasp \
  std_srvs/srv/SetBool "{data: true}"
```

## 7. 滑り検知と復旧

監視値は以下で確認できる。

```bash
ros2 topic echo /cooperative_transport/slip_status
ros2 topic echo /cooperative_transport/safety_status
```

初期しきい値:

| 監視項目 | しきい値 |
|---|---:|
| 水平相対ずれ | 30 mm |
| 鉛直相対ずれ | 30 mm |
| 相対yawずれ | 4 deg |

しきい値超過時は `slip_fault=true` となり、安全監視が緊急停止をラッチして両台車を同時停止する。原因を除去し、搬送を無効にしてから次の順で解除する。

```bash
ros2 service call /cooperative_transport/set_enabled \
  std_srvs/srv/SetBool "{data: false}"

ros2 service call /cooperative_transport/reset_slip std_srvs/srv/Trigger "{}"
ros2 service call /cooperative_transport/reset_safety std_srvs/srv/Trigger "{}"
```

## 8. 調整箇所

主な設定は `cooperative_transport_control/config/friction_transport.yaml` にある。

- 把持角: `open_position`、`close_position`
- 目標法線力と許容差: `target_normal_force`、`force_tolerance`
- 力制御ゲインと更新幅: `force_position_gain`、`force_control_max_step`
- 通常把持トルク上限: `maximum_effort`（初期値1.3 N m）
- 過大荷重時の安全開放: `maximum_normal_force`（初期値35 N）
- バイアス平均時間: `bias_sample_duration`
- 接触判定範囲: `minimum_contact_position`、`maximum_contact_position`
- 左右同期判定: `maximum_gripper_mismatch`
- 物理接触の必須化: `require_contact`
- 法線反力下限: `minimum_normal_force`
- 接触の鮮度・安定時間: `contact_timeout`、`contact_stability_duration`
- 把持後の接触喪失時間: `contact_loss_timeout`
- 持上げ量: `lift_joint_2`、`lift_joint_3`
- 搬送速度・加速度: `max_linear_speed`、`max_linear_acceleration`
- 滑りしきい値: `horizontal_slip_limit`、`vertical_slip_limit`、`yaw_slip_limit`

搬送物の質量と摩擦は `cooperative_payload_friction/model.sdf` にある。現在は1 kgである。質量を変更する際は、`mass`だけでなく慣性も同じ比率で変更し、各段階で直進・旋回・停止時の滑りを記録する。

## 9. 段階Aとの使い分け

- 固定把持版: `simulation.launch.py`
- 補助力付き摩擦把持版（段階B）: `friction_simulation.launch.py`
- 補助力なし物理接触版（段階C）: `friction_simulation.launch.py contact_only:=true`

段階Aは協調軌道や経路の再現性確認、段階Bは補助力との比較、段階Cは把持力、
滑り、接触安定性の評価に使う。複数のlaunchを同時に起動しないこと。
