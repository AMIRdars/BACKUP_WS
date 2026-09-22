# 協調搬送・段階D（障害物スラローム）実行手順

作成日: 2026-08-03  
対象環境: Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress

## 1. この段階で追加したもの

短距離S字の次段階として、2本の円柱を避けながら6 m搬送する評価シナリオ
`slalom_obstacles`を追加した。

- 軌道長: 6.0 m
- ウェイポイント: 48点（0.125 m間隔）
- 横移動: 前半-0.55 m、後半+0.55 mの連続した2ローブ
- 直進導入・退出区間: 各0.50 m
- 搬送物ヨー目標: 最大約±2.8°（横移動主体）
- 障害物1: `(x, y) = (2.5, 1.0)`、半径0.18 m
- 障害物2: `(x, y) = (4.0, -1.0)`、半径0.18 m
- ロボット、搬送物、障害物間の最小クリアランスを20 HzでCSV記録
- 円柱接触センサによる衝突検出と両台車同時停止
- 台車・搬送物の外形を考慮したクリアランス計算
- 両ロボットの把持・持上げ完了後、初期把持台`grasp_support`だけを自動削除
- 把持台の削除成功を確認してから評価軌道を開始
- 両手首の6軸力覚値、RMS、ピーク値をCSVへ記録

搬送物とロボット間に`fixed`または`DetachableJoint`は作成しない。指の接触摩擦に加え、Gazebo Fortressで生じる接触摩擦の数値的なクリープを抑えるため、上限付きばね・減衰力を搬送物へ加えるbristle摩擦モデルを使用する。これは姿勢を固定する拘束ではなく、力・トルクに上限と変化率制限がある。

## 2. ビルド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  mecanumrover_description \
  amir_description \
  amir_gazebo \
  cooperative_transport_description \
  cooperative_transport_gazebo \
  cooperative_transport_control \
  cooperative_transport_bringup

source install/setup.bash
```

## 3. 実行

GUIあり:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles
```

ヘッドレス:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  headless:=true \
  scenario:=slalom_obstacles
```

同じPCで複数のGazebo評価を同時に起動しないこと。終了は`Ctrl+C`。

起動シーケンスは次の順で自動実行される。

1. 2台のロボットと搬送物を生成する。
2. 指を閉じ、アームで搬送物を持ち上げる。
3. 両ロボットが`HOLDING`になって0.5秒後、Gazeboの削除サービスで
   `grasp_support`だけを削除する。
4. `/cooperative_transport/support_removed`が`true`になってから2秒待ち、
   スラローム評価を開始する。

目標中心軌道は次の順で変化する。従来の±0.15 m軌道は直進に近く見えたため使用しない。

```text
(0.0, 0.0)
  → 0.5 m直進
  → (1.75, -0.55) 第1ローブ頂点
  → (3.00, 0.00)  中央交差
  → (4.25, +0.55) 第2ローブ頂点
  → (5.50, 0.00)
  → 0.5 m直進して(6.00, 0.00)
```

ロボット、搬送物、障害物は削除対象ではない。削除した把持台は次回の
launch起動時にworldファイルから再生成される。

## 4. 実行中の確認

別端末で環境を読み込む。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /cooperative_transport/evaluation_status
ros2 topic echo /cooperative_transport/slip_status
ros2 topic echo /cooperative_transport/safety_status
ros2 topic echo /cooperative_transport/obstacle_status
ros2 topic echo /cooperative_transport/minimum_obstacle_clearance
ros2 topic echo --once \
  /cooperative_transport/support_removed \
  std_msgs/msg/Bool \
  --qos-durability transient_local
```

把持台の削除が成功すると、起動端末には次のログが出る。

```text
Removed temporary support model "grasp_support"; transport may start.
```

最終結果:

```bash
ros2 topic echo --once \
  /cooperative_transport/evaluation_result \
  std_msgs/msg/String \
  --qos-durability transient_local
```

CSVは既定で`/tmp/cooperative_transport_results/`へ保存される。`obstacle_clearance_m`、`closest_obstacle_pair`、`obstacle_collision_count`列を含む。

## 5. 安全判定

| 項目 | 値 |
|---|---:|
| 指令最高速度 | 0.025 m/s |
| 指令最高加速度 | 0.025 m/s² |
| 水平把持ずれ停止 | 0.030 m |
| 鉛直把持ずれ停止 | 0.030 m |
| ヨー把持ずれ停止 | 8° |
| 台車速度停止 | 0.200 m/sが0.1秒継続 |
| 台車間距離 | 2.30～3.10 m |
| 障害物接触 | 1回で即時FAIL |

安全停止はラッチされ、2台を同時に停止する。

## 6. 現在の検証状況と重要な制約

軌道生成、障害物配置、接触ブリッジ、クリアランス評価、CSV出力、安全停止までは実装・単体試験済みである。

最初の実装では、搬送物を載せる初期把持台が把持後もworldに残っていた。そのため約0.75 m地点で台車と把持台が接触しており、搬送経路そのものとは無関係な衝突が発生していた。

2026-08-04に、直進に近かった振幅±0.15 mを廃止し、直進導入後に
`0 → -0.55 → 0 → +0.55 → 0 m`と移動する実S字軌道へ変更した。
搬送物を接線方向へ最大±8.7°回す試験では、両端の摩擦把持が上下にこじられ、
第1ローブ頂点で鉛直滑り停止した。このためメカナム台車の横移動を主体とし、
ヨー目標を最大約±2.8°へ制限した。安全しきい値は緩和していない。

最終ヘッドレス試験では48点、全長6 mを完走した。

```text
PASS: all waypoints completed
duration               = 262.65 s
max_horizontal_slip    = 0.0020 m
max_vertical_slip      = 0.0194 m
max_separation_change  = 0.0321 m
min_obstacle_clearance = 0.1313 m
obstacle_collisions    = 0
```

評価CSV:

```text
/tmp/cooperative_transport_results/20260804_001312_slalom_obstacles.csv
```

固定ジョイントや姿勢テレポートで合格を偽装する処理は採用していない。搬送物の把持は引き続き、指の接触摩擦と上限付きbristle摩擦力で行う。

## 7. 主な実装ファイル

- `cooperative_transport_control/cooperative_transport_control/evaluation_trajectories.py`
- `cooperative_transport_control/cooperative_transport_control/trajectory_evaluator.py`
- `cooperative_transport_control/cooperative_transport_control/grasp_support_remover.py`
- `cooperative_transport_control/cooperative_transport_control/obstacle_clearance.py`
- `cooperative_transport_control/cooperative_transport_control/safety_monitor.py`
- `cooperative_transport_gazebo/worlds/cooperative_transport_friction.sdf`
- `cooperative_transport_gazebo/src/friction_grip_system.cpp`
- `cooperative_transport_bringup/launch/trajectory_evaluation.launch.py`

## 8. テスト

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select cooperative_transport_control
colcon test-result --test-result-base \
  build/cooperative_transport_control --verbose
```
