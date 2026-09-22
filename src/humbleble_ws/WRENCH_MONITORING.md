# 協調搬送・段階E（手首力覚計測としきい値停止）実行手順

作成日: 2026-08-03  
対象環境: Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress

## 1. この段階で追加したもの

手順書の段階7「コンプライアンス」の導入順序1、2に相当する機能を追加した。

- 両AMIRの`Joint_5`に6軸F/Tセンサを追加
- Gazeboの手首力・モーメントを100 HzでROS 2へブリッジ
- 両センサの現在値、合力RMS、ピーク値を表示
- 評価CSVへ両手首の6軸値、RMS、ピーク値を記録
- 力・モーメントしきい値の0.1秒継続で両台を同時停止
- センサ更新停止も安全異常として検出

この段階ではアドミッタンス補正はまだ行わない。まず現在の内力・動的荷重を
観測し、しきい値停止が働くことを確認する段階である。

## 2. ビルド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  amir_description \
  amir_gazebo \
  cooperative_transport_gazebo \
  cooperative_transport_control \
  cooperative_transport_bringup

source install/setup.bash
```

## 3. 計測モードで実行

従来と同じコマンドでは、力覚値を計測・記録するが、力覚しきい値による停止は
行わない。滑り、速度、衝突など従来の安全停止は引き続き有効である。

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles
```

GUIが不要な場合:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  headless:=true \
  scenario:=slalom_obstacles
```

## 4. 力覚値の確認

別端末で実行する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /amir1/ft_sensor
ros2 topic echo /amir2/ft_sensor
ros2 topic echo /cooperative_transport/wrench_status
ros2 topic hz /amir1/ft_sensor
ros2 topic hz /amir2/ft_sensor
```

`wrench_status`が`MONITORING`で始まり、各ロボットについて次が表示される。

- `F=(Fx,Fy,Fz)N`
- `M=(Mx,My,Mz)Nm`
- `rms`: 搬送開始後の合力RMS
- `peak`: 搬送開始後の最大合力
- `WARNING`: `|Fx|`または`|Fy|`が30 Nを超過

## 5. しきい値停止試験

次のコマンドでは力覚安全停止を有効にする。

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles \
  enable_wrench_safety:=true
```

初期しきい値:

| 判定 | 値 |
|---|---:|
| 平面力警告 | `|Fx|`または`|Fy| > 30 N` |
| 平面力停止 | `|Fx|`または`|Fy| > 50 N`が0.1秒継続 |
| 鉛直力停止 | `|Fz| > 100 N`が0.1秒継続 |
| モーメント停止 | いずれかの軸が15 Nm超過を0.1秒継続 |
| センサタイムアウト | 0.25秒 |

停止時は`/cooperative_transport/wrench_fault`が`true`になり、既存の
`safety_monitor`が緊急停止をラッチして2台へ同時に停止指令を出す。

```bash
ros2 topic echo /cooperative_transport/wrench_fault
ros2 topic echo /cooperative_transport/safety_status
```

停止を解除する場合は、力超過の原因がなくなったことを確認してから実行する。

```bash
ros2 service call /cooperative_transport/reset_safety \
  std_srvs/srv/Trigger '{}'
```

## 6. CSV出力

CSVは従来どおり`/tmp/cooperative_transport_results/`へ保存され、次の列が追加される。

```text
robot1_fx_N ... robot1_mz_Nm
robot2_fx_N ... robot2_mz_Nm
wrench_force_rms_N
wrench_peak_force_N
wrench_torque_rms_Nm
wrench_peak_torque_Nm
```

最終評価結果にも合力・モーメントのRMSとピークが含まれる。

## 7. このPCでの確認結果

2026-08-03に次を確認した。

- `/amir1/ft_sensor`、`/amir2/ft_sensor`を約80～100 Hzで受信
- 把持台削除後も両センサ値を継続受信
- 計測モードでは従来の0.75 m地点を通過し、約1.25 mまで安全停止なし
- しきい値停止モードでは`amir1/Fy=-148.8 N`を検出
- 0.1秒継続後、`safety_monitor`が両台を同時停止
- 停止試験時の記録値は合力RMS 48.63 N、最大合力149.90 N
- 0.5 m直進では瞬間最大50.60 Nでも0.1秒継続しなかったため誤停止せず完走

手首センサ値には把持内力だけでなく、グリッパ・搬送物の重力、加速度による
慣性力、接触反力も含まれる。したがって現時点では「純粋な内力」ではなく、
手首で伝達される合成荷重として扱う。

## 8. 次の段階

通常搬送で手順書の初期停止値50 Nを上回ることが確認されたため、次は計測した
横力を使い、フォロワ側の横方向速度目標をわずかに逃がす1軸アドミッタンスを
実装する。最初から平面・ヨーの3自由度を同時に有効にはしない。

## 9. 主な実装ファイル

- `amir_description/urdf/amir_mecanum3_sim.xacro`
- `amir_gazebo/launch/robot_bringup.launch.py`
- `cooperative_transport_gazebo/worlds/cooperative_transport_friction.sdf`
- `cooperative_transport_control/cooperative_transport_control/wrench_monitor.py`
- `cooperative_transport_control/cooperative_transport_control/wrench_metrics.py`
- `cooperative_transport_control/cooperative_transport_control/safety_monitor.py`
- `cooperative_transport_control/cooperative_transport_control/trajectory_evaluator.py`
