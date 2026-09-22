# 協調搬送・段階C（軌道追従評価）実行手順

作成日: 2026-08-03  
対象環境: Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress

## 1. 実装内容

段階Bの接触・摩擦把持を使い、再現可能な軌道試験とCSV記録を行う段階Cを追加した。段階Aの固定拘束版もそのまま残している。

このlaunchでは`DetachableJoint`を使用しない。搬送物は2台のAMIRの指先との接触力と摩擦だけで支持・搬送される。

追加した主な機能は次のとおり。

- 搬送物の初期姿勢を基準にした相対ウェイポイント生成
- 直進、30°旋回、90°旋回、短距離S字軌道
- 摩擦把持完了後の自動試験開始
- 搬送物実姿勢、編隊推定姿勢、把持ずれ、2台間距離、各台車指令のCSV記録
- ウェイポイント到達、タイムアウト、安全停止による自動合否判定
- 試験開始・中止サービスと、最後の結果を保持するresult topic
- 搬送物の実測X/Yを使った閉ループ補正
- AMIRの簡略メカナム車輪に固定座標系の異方性摩擦方向を設定

S字軌道と90°旋回は、摩擦把持へ不連続な荷重を与えないよう隣接する目標ヨー角をそれぞれ最大4°、5°にしている。

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

## 3. 自動評価の実行

同じPC上で複数の評価launchを同時に起動しないこと。GUIありで30°旋回を実行する例:

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py scenario:=turn_30
```

ヘッドレスでS字軌道を実行する例:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  headless:=true \
  scenario:=slalom_short
```

利用できるシナリオ:

| `scenario` | 内容 | ウェイポイント数 |
|---|---:|---:|
| `straight_0_5m` | 0.5 m直進 | 1 |
| `turn_30` | 半径0.6 m、30°旋回 | 6 |
| `turn_90` | 半径0.8 m、90°旋回 | 18 |
| `slalom_short` | 全長0.6 m、振幅0.08 m、最大ヨー角±8°のS字 | 12 |
| `slalom_obstacles` | 全長6.0 m、横振幅±0.55 mの実S字障害物スラローム | 48 |
| `slalom_open` | 円柱なし、全長8.0 m、横振幅±1.0 mの大振幅S字 | 64 |

障害物スラローム固有の実行方法、判定値、現在の検証状況は
[`OBSTACLE_SLALOM.md`](OBSTACLE_SLALOM.md)を参照する。

起動後は、2台の生成、搬送物生成、両グリッパの摩擦把持、持上げを行い、`HOLDING`から2秒後に評価が始まる。端末に`PASS:`または`FAIL:`が表示されたら試験完了である。終了は`Ctrl+C`で行う。

## 4. 状態と結果の確認

別端末でワークスペースを読み込んでから確認する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /cooperative_transport/evaluation_status
```

実行中の状態は`RUNNING; scenario=...; waypoint=N/M`の形式で表示される。完了後の最終結果を取得する場合:

```bash
ros2 topic echo --once \
  /cooperative_transport/evaluation_result \
  std_msgs/msg/String \
  --qos-durability transient_local
```

安全系も同時に確認できる。

```bash
ros2 topic echo /cooperative_transport/slip_status
ros2 topic echo /cooperative_transport/safety_status
```

## 5. 手動開始・中止

把持完了後に任意の時点で開始する場合:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=turn_30 \
  auto_start:=false
```

`HOLDING`を確認してから別端末で開始する。

```bash
ros2 service call /cooperative_transport/start_evaluation \
  std_srvs/srv/Trigger '{}'
```

実行中の評価を停止する場合:

```bash
ros2 service call /cooperative_transport/cancel_evaluation \
  std_srvs/srv/Trigger '{}'
```

## 6. CSV結果

既定の保存先は次のディレクトリである。

```text
/tmp/cooperative_transport_results/
```

保存先はlaunch引数で変更できる。

```bash
mkdir -p ~/cooperative_transport_results

ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  headless:=true \
  scenario:=slalom_short \
  results_directory:=$HOME/cooperative_transport_results
```

CSVには20 Hzで次の値を記録する。

- 経過時間、シナリオ名、ウェイポイント番号
- 搬送物の目標姿勢、Gazebo実測姿勢、2台編隊からの推定姿勢
- 推定位置誤差、推定ヨー角誤差
- X/Y/Z方向の把持ずれ、把持ずれ角
- 2台間距離と各ロボットのX/Y/ヨー角
- 両ロボットへ送ったX/Y/ヨー速度指令
- コーディネータ、把持ずれ監視、安全監視の状態

## 7. 判定値

軌道評価と独立安全監視は役割を分けている。

| 項目 | 値 |
|---|---:|
| ウェイポイント位置許容誤差 | 0.025 m |
| ウェイポイントヨー許容誤差 | 5° |
| 水平把持ずれ停止値 | 0.030 m |
| 鉛直把持ずれ停止値 | 0.030 m |
| 把持ずれ角停止値 | 8° |
| ロボット間距離許容範囲 | 2.30～3.10 m |
| 台車速度安全上限 | 0.12 m/s |

安全違反はラッチされ、必ず両方の台車を同時停止する。片方だけを再始動してはならない。

## 8. 安全停止後の復帰

原因を確認し、搬送を無効にした状態で把持ずれ監視を先にリセットする。

```bash
ros2 service call /cooperative_transport/set_enabled \
  std_srvs/srv/SetBool '{data: false}'

ros2 service call /cooperative_transport/reset_slip \
  std_srvs/srv/Trigger '{}'

ros2 service call /cooperative_transport/reset_safety \
  std_srvs/srv/Trigger '{}'
```

把持を解除する場合:

```bash
ros2 service call /cooperative_transport/friction_grasp \
  std_srvs/srv/SetBool '{data: false}'
```

鉛直ずれや物体落下後は、そのまま評価を再開せずlaunchを終了して初期状態からやり直す。

## 9. このPCでの確認結果

2026-08-03にヘッドレス実行した結果:

| シナリオ | 結果 | 時間 | 最大水平ずれ | 最大鉛直ずれ | 最大角度ずれ |
|---|---|---:|---:|---:|---:|
| `turn_30` | PASS | 31.35 s | 0.0059 m | 0.0235 m | 3.84° |
| `slalom_short` | PASS | 67.80 s | 0.0223 m | 0.0174 m | 2.63° |
| `turn_90` | 35°地点で安全停止 | 39.40 s | 0.0093 m | 0.0300 m | 3.59° |

`turn_90`では水平・角度追従は安全範囲内だったが、長時間旋回中に物体が指先に対して30 mm沈み、鉛直ずれ監視が正常に停止させた。したがって現時点の摩擦把持モデルで90°完走を保証しない。次段階では力覚相当値の取得、把持力制御または鉛直方向アドミッタンスを追加して、この沈み込みを抑制する。

## 10. 主な実装ファイル

- `cooperative_transport_control/cooperative_transport_control/trajectory_evaluator.py`
- `cooperative_transport_control/cooperative_transport_control/evaluation_trajectories.py`
- `cooperative_transport_control/config/trajectory_evaluation.yaml`
- `cooperative_transport_bringup/launch/trajectory_evaluation.launch.py`
- `cooperative_transport_control/cooperative_transport_control/slip_monitor.py`
- `amir740_ros/amir_gazebo/launch/robot_bringup.launch.py`
- `mecanumrover3_ros2/mecanumrover_description/urdf/mecanum3.gazebo`

## 11. テスト

軌道生成と協調運動学の単体テスト:

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select cooperative_transport_control
colcon test-result --test-result-base \
  build/cooperative_transport_control --verbose
```

この手順で既存の段階A、段階Bを変更せず、段階Cだけを再ビルド・評価できる。
