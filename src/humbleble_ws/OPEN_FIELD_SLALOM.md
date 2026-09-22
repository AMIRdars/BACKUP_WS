# 開空間・大振幅S字協調搬送の実行手順

## 実装内容

障害物回避の影響を分離してS字運動そのものを確認するため、`slalom_open`を追加した。

- 専用world: `cooperative_transport_open.sdf`
- 円柱モデル、円柱接触センサ、円柱topicは存在しない
- 全長8.0 m
- 64ウェイポイント、0.125 m間隔
- 横振幅±1.0 m
- 開始と終了に各0.5 mの直進区間
- 1軸アドミッタンス制御を有効化
- 摩擦把持の滑り監視と2台同時安全停止は継続

目標とする搬送物中心軌道:

```text
(0.00,  0.00)
  → (0.50,  0.00)  直進導入
  → (2.25, -1.00)  第1ローブ頂点
  → (4.00,  0.00)  中央交差
  → (5.75, +1.00)  第2ローブ頂点
  → (7.50,  0.00)
  → (8.00,  0.00)  直進退出
```

ローブは`sin²`で接続しているため、開始、各頂点、中央交差、終了で横方向勾配が連続的にゼロになる。

## ビルド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  cooperative_transport_gazebo \
  cooperative_transport_control \
  cooperative_transport_bringup

source install/setup.bash
```

## GUI付き実行

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_open
```

`scenario:=slalom_obstacles`ではなく、必ず`scenario:=slalom_open`を指定する。

## ヘッドレス実行

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_open \
  headless:=true
```

## 状態確認

```bash
ros2 topic echo /cooperative_transport/evaluation_status
ros2 topic echo /cooperative_transport/measured_payload_pose
ros2 topic echo /cooperative_transport/slip_status
ros2 topic echo /cooperative_transport/admittance_status
```

完了結果:

```bash
ros2 topic echo --once \
  /cooperative_transport/evaluation_result \
  std_msgs/msg/String \
  --qos-durability transient_local
```

## このPCでの確認結果

2026年8月4日にヘッドレスGazeboで全64点を完走した。sceneには走行開始時点で次のモデルだけが存在し、円柱がないことを確認した。

```text
ground_plane
amir1
amir2
cooperative_payload
```

評価結果:

```text
PASS: all waypoints completed
duration                = 372.95 s
max_horizontal_slip     = 0.0021 m
max_vertical_slip       = 0.0238 m
max_yaw_slip            = 0.60 deg
max_separation_change   = 0.0383 m
wrench_force_rms        = 88.64 N
wrench_peak_force       = 156.99 N
obstacle_collisions     = 0
```

CSV:

```text
/tmp/cooperative_transport_results/20260804_003520_slalom_open.csv
```

このシナリオは開空間での大振幅S字を確認する段階であり、障害物クリアランスは評価対象外である。次段階で、この軌道を基準に円柱配置と経路振幅を組み合わせる。
