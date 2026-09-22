# 1軸アドミッタンス協調搬送の実行手順

## 今回の実装範囲

手順書「段階7: コンプライアンス」のうち、F/Tセンサ表示と力しきい値停止に続く「1軸アドミッタンス」を実装した。

- amir1をリーダ、amir2をフォロワとする。
- 2台の手首F/TセンサのローカルY軸を使う。
- 把持完了直後の定常荷重をバイアスとして差し引く。
- 2台のY軸力誤差の平均を内力指標とする。
- amir2のローカルY速度だけに逃げ速度を加える。
- センサが0.25秒以上途絶えた場合は補正をゼロにする。
- 従来の速度上限、加速度上限、滑り監視、衝突監視はそのまま有効である。

制御式は次の1自由度の仮想質量・ばね・ダンパである。

```text
M_d * x_ddot + D_d * x_dot + K_d * x = F_error
```

初期値は手順書の横方向例を基準にし、安全のため速度と仮想変位を小さく制限した。

```text
M_d                 = 3.0 kg
D_d                 = 26.8 N s/m
K_d                 = 60.0 N/m
力デッドバンド      = 3.0 N
ローパス時定数       = 0.18 s
最大仮想変位         = 0.012 m
最大補正速度         = 0.004 m/s
```

## ビルド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select cooperative_transport_control cooperative_transport_bringup

source install/setup.bash
```

## アドミッタンス付きスラローム

`trajectory_evaluation.launch.py`では、`scenario:=slalom_obstacles`のときだけ1軸アドミッタンスを標準で有効にする。したがって従来と同じコマンドで実行できる。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles
```

GUIが不要な場合:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles \
  headless:=true
```

## 動作確認

別端末で次を実行する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /cooperative_transport/admittance_status
```

正常動作中の表示例:

```text
ACTIVE; axis=amir2/local_y; force_error=-2.54N;
filtered_force=-2.59N; displacement=-0.0030m;
correction=0.0040m/s
```

数値だけを確認する場合:

```bash
ros2 topic echo /cooperative_transport/admittance_correction
```

配列の意味は次のとおりである。

```text
[0] バイアス除去後の2台平均Y軸力誤差 [N]
[1] 符号補正・ローパス後の制御力 [N]
[2] 仮想変位 [m]
[3] amir2ローカルY軸への速度補正 [m/s]
```

補正速度の絶対値が`0.004 m/s`以下、仮想変位の絶対値が`0.012 m`以下であることを確認する。

## 従来制御との比較

アドミッタンスを無効にして同じ経路を実行する場合:

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles \
  enable_lateral_admittance:=false
```

各試験のCSVは次へ保存される。

```text
/tmp/cooperative_transport_results/<日時>_slalom_obstacles.csv
```

今回追加したCSV列:

```text
admittance_force_error_N
admittance_filtered_force_N
admittance_displacement_m
admittance_correction_mps
```

結果表示にも最大力誤差、最大仮想変位、最大補正速度を追加した。比較時は少なくとも次を確認する。

- `wrench_force_rms`
- `wrench_peak_force`
- `max_admittance_force_error`
- `max_horizontal_slip`
- `min_obstacle_clearance`
- `obstacle_collisions`

## このPCでの確認結果

2026年8月4日に、直進に近かった旧振幅±0.15 mを±0.55 mへ変更し、
ヘッドレスGazeboで48点、全長6 mの実S字障害物スラロームを完走した。

```text
PASS: all waypoints completed
duration                 = 262.65 s
max_horizontal_slip      = 0.0020 m
max_vertical_slip        = 0.0194 m
max_separation_change    = 0.0321 m
wrench_force_rms         = 84.13 N
wrench_peak_force        = 156.49 N
max_admittance_displacement = 0.0120 m
max_admittance_correction   = 0.0040 m/s
min_obstacle_clearance   = 0.1313 m
obstacle_collisions      = 0
```

CSV:

```text
/tmp/cooperative_transport_results/20260804_001312_slalom_obstacles.csv
```

この段階は「横1軸だけ」の初期実装である。ピーク力を消せたという意味ではなく、補正を安全な小範囲に限定したままスラロームを完走できる枠組みを確認した。次段階では無効時との複数回比較からパラメータを調整し、その後に搬送平面X・Yとヨーへ順に拡張する。

## 安全停止を同時に試す場合

力しきい値停止は別機能であり、明示的に有効にする。

```bash
ros2 launch cooperative_transport_bringup \
  trajectory_evaluation.launch.py \
  scenario:=slalom_obstacles \
  enable_lateral_admittance:=true \
  enable_wrench_safety:=true
```

現在のスラロームでは瞬間的に大きな力が観測されるため、設定した継続時間を超えると途中停止する可能性がある。まずは標準の計測モードでCSVを比較し、しきい値を実測に基づいて調整する。
