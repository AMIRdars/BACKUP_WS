# 協調搬送シミュレーション負荷改善 実装手順書

作成日: 2026-09-28  
対象環境: ROS 2 Humble / Gazebo Fortress / DART / cooperative_transport_bringup  
対象起動: `integrated_transport_simulation.launch.py`

---

## 0. 本ドキュメントの目的

本ドキュメントは、協調搬送シミュレーションで発生しているリアルタイム進行率（RTF）低下について、解析結果をもとに改善項目を優先順位順に実装・検証するための手順書である。

CodeXでは、**一度に複数の改善を適用せず、必ず1項目ずつ変更 → ビルド → 実行 → 測定 → 判定**を行うこと。

### 現状の主要な問題

解析結果では、把持後に以下の負荷が確認されている。

- `max_step_size = 0.002 s`
- 1ステップの時間予算 = `2.0 ms`
- Updateパス実測 = 約 `7.13 ms / step`
- 把持後 RTF = 約 `0.26`
- Gazebo主スレッドはほぼ1コア飽和
- DART本体 + LCP拘束ソルバがGazebo主スレッドCPUサンプルの大部分を占有
- GUI停止のみでは RTF ≒ `0.42` であり、実時間には追いつかない
- Python制御ノードは制御式より、全姿勢メッセージ変換・ROS executor処理の負荷が大きい

したがって、主改善方針は以下とする。

1. 接触・摩擦拘束の削減
2. 物理ステップの見直し
3. 不要な衝突判定の削減
4. 全姿勢メッセージの削減
5. contact通知の集約
6. GUI負荷の削減
7. 制御周期・ROS通信周期の調整
8. 起動シーケンスのイベント駆動化

---

# 1. 実装全体の原則

## 1.1 必ずベースラインを保存する

変更前に現在動作している状態をGitへ保存する。

```bash
cd /home/dars5070/ros2_humble_ws
git status
git add .
git commit -m "baseline before simulation performance optimization"
```

リモートへ保存可能であればpushする。

```bash
git push
```

### チェックポイント

- [ ] 現在の協調搬送が通常起動できる
- [ ] 2台のロボットが把持できる
- [ ] 支持台除去まで進む
- [ ] 現在の変更状態がGitへ保存されている
- [ ] 解析用データ `simulation_performance/20260928/` を残している

---

## 1.2 1項目ずつブランチを作る

推奨例:

```bash
git checkout -b perf/collision-simplification
```

各改善ごとに別ブランチを作る。

例:

```text
perf/collision-simplification
perf/physics-step
perf/collision-filter
perf/pose-filter
perf/contact-aggregation
perf/headless
perf/control-rate
perf/startup-sequence
```

---

## 1.3 評価指標

すべての改善で最低限以下を記録する。

### 性能指標

- RTF
- Gazebo server CPU
- Gazebo GUI CPU
- coordinator CPU
- slip_monitor CPU
- friction_grasp_manager CPU
- Update path ms/step

### 物理・制御品質

- 把持成功 / 失敗
- 搬送物の落下有無
- 搬送物の最大滑り量
- 把持中の左右指接触状態
- 把持力
- 搬送物の位置誤差
- 搬送物の姿勢誤差
- ロボット台車姿勢
- 指定軸回転が成立するか

### 比較用の推奨CSV

```text
condition,
max_step_size,
finger_collision_count,
rtf,
update_ms,
gazebo_server_cpu,
coordinator_cpu,
slip_monitor_cpu,
grasp_success,
max_slip,
position_error,
orientation_error,
notes
```

---

# 2. 改善項目1: 指のCollision形状の簡略化

## 優先度

**最優先**

## 目的

DART / LCPソルバで計算される接触・摩擦拘束を直接減らす。

現状は左右の指が複数のbox collisionに分割されており、2台合計で多数のcollision形状・contact sensorを使用している。

---

## 2.1 変更対象を特定する

CodeXへ以下を依頼する。

```text
AMIRのURDF/xacro/SDFを探索し、
左右グリッパ指のcollision形状を定義しているファイルを特定してください。

以下を一覧化してください。
- ファイルパス
- link名
- collision名
- geometry種類
- box寸法
- origin xyz/rpy
- collision数
- contact sensorとの対応関係

この段階では変更しないでください。
```

### チェックポイント

- [ ] 左右指のcollision定義ファイルが特定できた
- [ ] 1指あたりのcollision数が確認できた
- [ ] contact sensorとの対応を確認できた

---

## 2.2 9分割相当 → 3分割へ簡略化する

最初から1個へ減らさず、まず3個程度へ減らす。

基本方針:

```text
現在:
[box][box][box][box][box][box][box][box][box]

変更:
[   box   ][   box   ][   box   ]
```

接触面積・指長・指幅・指高さはできるだけ維持する。

### CodeXへの実装指示

```text
左右グリッパ指のcollisionを、現在の外形をできるだけ維持しながら
各指3個のbox collisionへ統合してください。

条件:
- visualは変更しない
- joint構造は変更しない
- 指先全長を維持
- 把持対象側の接触面位置を維持
- collisionの重なりを最小限にする
- inertialは変更しない
- 元のcollision定義はコメントまたはGit差分で復元可能にする
```

---

## 2.3 ビルド

```bash
cd /home/dars5070/ros2_humble_ws
colcon build --symlink-install
source install/setup.bash
```

必要に応じて対象パッケージのみ:

```bash
colcon build --symlink-install --packages-select <対象パッケージ>
```

---

## 2.4 起動確認

```bash
ros2 launch cooperative_transport_bringup \
  integrated_transport_simulation.launch.py \
  headless:=false
```

### チェック項目

- [ ] URDF/SDFロードエラーなし
- [ ] 指形状が異常な位置に出ていない
- [ ] 2台とも開閉できる
- [ ] 荷物を把持できる
- [ ] 荷物が落下しない
- [ ] 指が荷物を貫通しない
- [ ] 異常振動が発生しない
- [ ] 支持台除去後も保持できる

---

## 2.5 性能測定

```bash
/usr/bin/python3 simulation_performance/20260928/measure.py \
  simulation_performance/20260928/collision_3box 180 false
```

測定後:

```bash
/usr/bin/python3 simulation_performance/20260928/analyze.py
/usr/bin/python3 simulation_performance/20260928/summarize_profiles.py
PYTHONNOUSERSITE=1 \
/usr/bin/python3 simulation_performance/20260928/plot_results.py
```

---

## 2.6 3個 → 1個も比較する

3個で把持性能に問題がなければ1個のboxも検証する。

### 注意

1boxで以下が悪化した場合は3boxを採用する。

- 滑り
- 把持力の変動
- 荷物姿勢
- 指先接触判定
- 回転中の保持性能

---

## 合格条件

最低条件:

- RTFが改善する
- 把持成功率を維持
- 荷物落下なし
- 滑りが研究上許容できる
- 搬送動作に明確な悪影響なし

### 採用判断

```text
9box → 3boxで十分改善:
    3boxを候補

3box → 1boxでさらに改善し、物理挙動も維持:
    1boxを候補

1boxで滑り/貫通/姿勢誤差増大:
    3boxへ戻す
```

---

# 3. 改善項目2: 物理ステップ `max_step_size` の見直し

## 優先度

**最優先**

## 目的

物理計算の実行回数そのものを削減する。

現在:

```text
max_step_size = 0.002
physics update = 500 Hz
```

候補:

```text
0.002 s = 500 Hz
0.003 s ≒ 333 Hz
0.004 s = 250 Hz
```

---

## 3.1 設定箇所を特定する

CodeXへ依頼:

```text
cooperative_transport_contact.sdf または関連world/xacroから
max_step_size と real_time_factor を設定している箇所を特定してください。

変更前の値とファイルパスを示してください。
この段階では変更しないでください。
```

---

## 3.2 0.003 sを試す

```xml
<max_step_size>0.003</max_step_size>
```

または該当する設定形式へ変更。

### ビルド・起動

```bash
colcon build --symlink-install
source install/setup.bash

ros2 launch cooperative_transport_bringup \
  integrated_transport_simulation.launch.py \
  headless:=false
```

---

## 3.3 確認項目

- [ ] 把持成立
- [ ] 接触が飛ばない
- [ ] 荷物の異常振動なし
- [ ] 指の貫通なし
- [ ] 支持台除去後に安定
- [ ] 前進可能
- [ ] 回転可能
- [ ] 横移動可能
- [ ] 指定軸回転可能

---

## 3.4 0.004 sを試す

0.003 sが正常なら0.004 sも同条件で試す。

```xml
<max_step_size>0.004</max_step_size>
```

---

## 重要

`real_time_factor`だけを上げても計算負荷は減らない。

変更対象は主に:

```text
max_step_size
```

である。

---

## 合格条件

以下を同時に満たす最大のstepを採用候補とする。

- 把持失敗なし
- 滑り量が許容範囲
- 指定軸回転が成立
- 搬送物姿勢が不安定にならない
- RTFが改善

---

# 4. 改善項目3: 不要な衝突判定の削減

## 優先度

**高**

## 目的

把持に関係しないcollision pairをDARTへ解かせない。

---

## 4.1 衝突対象を分類する

CodeXへ依頼:

```text
協調搬送モデルの全collisionを次のカテゴリへ分類してください。

A. 把持に必須
B. 床接触に必須
C. 自己衝突防止に必須
D. 通常動作では不要
E. visual用途だけでcollision不要の可能性がある

特に以下を確認してください。
- 隣接リンク同士
- アーム内部
- グリッパ固定部
- ベース内部
- 装飾部
```

---

## 4.2 自己衝突を整理する

以下は慎重に除外する。

```text
常に隣接しているリンク
構造上接触しないリンク
固定部品同士
```

### 禁止

以下をむやみに除外しない。

```text
finger ↔ object
wheel ↔ ground
base ↔ obstacle
arm ↔ object
robot ↔ robot
```

---

## 4.3 collision bitmask等を利用する場合

使用中のGazebo/DART構成で利用可能なcollision filtering方式を確認した上で設定する。

CodeXへ:

```text
現在のGazebo Fortress + DART環境で使用可能な
collision filtering / category bitmask / collide_without_contact 等を確認し、
現在のモデル形式で安全に使える方法のみ実装してください。
```

---

## 合格条件

- 把持判定維持
- 障害物衝突判定維持
- 床接触維持
- ロボット同士の危険接触を検出可能
- RTF改善

---

# 5. 改善項目4: `/pose/info` の必要姿勢だけを配信

## 優先度

**高**

## 目的

Python側で大量のTransformオブジェクトを生成する処理を削減する。

現状では `coordinator` と `slip_monitor` がworld全体の姿勢情報を受信している。

---

## 5.1 現在の購読箇所を特定

CodeXへ:

```text
以下のノードから
/world/cooperative_transport_friction/pose/info
を購読しているコードを探索してください。

- coordinator
- slip_monitor
- その他の購読ノード

各ノードについて以下を整理してください。
- ファイル
- topic
- message type
- callback
- 実際に使用しているentity名
```

---

## 5.2 必要entityを確定する

原則、必要なのは以下のような最小集合とする。

```text
payload
robot1 base
robot2 base
```

必要なら:

```text
robot1 gripper
robot2 gripper
```

---

## 5.3 推奨構成

### Before

```text
Gazebo
  ↓
world全体 pose/info
  ↓
Python coordinator
  ↓
245程度のTransform変換
  ↓
必要な3～5姿勢だけ探索
```

### After

```text
Gazebo
  ↓
C++ / bridge / filter node
  ↓
必要entityのみ
  ↓
ROS
  ↓
Python coordinator / slip_monitor
```

---

## 5.4 専用メッセージ候補

例:

```text
geometry_msgs/PoseStamped
```

または複数entity用の独自msg:

```text
CooperativeTransportPose.msg
```

例:

```text
std_msgs/Header header
geometry_msgs/Pose payload
geometry_msgs/Pose robot1
geometry_msgs/Pose robot2
```

---

## 5.5 実装方針

CodeXへ:

```text
world pose/infoを直接Pythonノードで全件処理する構成を変更します。

Gazebo側またはC++中継ノード側で、
payload / robot1 / robot2 の必要姿勢だけ抽出してください。

既存coordinatorとslip_monitorは、
抽出済みの軽量topicを購読するよう変更してください。

制御ロジックそのものは変更しないでください。
```

---

## 5.6 検証

```bash
ros2 topic hz <新topic>
ros2 topic echo <新topic>
```

確認:

- [ ] payload位置一致
- [ ] robot1位置一致
- [ ] robot2位置一致
- [ ] frame_id整合
- [ ] timestamp整合
- [ ] 制御結果が変更前と一致

---

## 合格条件

- coordinator CPU低下
- slip_monitor CPU低下
- 制御結果維持
- RTFが悪化しない

---

# 6. 改善項目5: contact通知の集約

## 優先度

**高**

## 目的

多数のcontact topic・callback・Python object生成を削減する。

---

## 6.1 現状確認

CodeXへ:

```text
finger contact sensorの全定義と、
Gazebo→ROS bridge、
friction_grasp_manager側のsubscriberを一覧化してください。

以下を整理してください。
- sensor数
- topic数
- publish rate
- message type
- callback数
- 左右どの指に対応するか
```

---

## 6.2 意味単位へ集約する

推奨:

```text
robot1_left_contact
robot1_right_contact
robot2_left_contact
robot2_right_contact
```

必要であれば各指について:

```text
contact_count
max_normal_force
total_normal_force
contact_detected
```

だけを配信する。

---

## 6.3 推奨アーキテクチャ

### Before

```text
多数のcontact sensor
    ↓
多数のROS topic
    ↓
多数のPython callback
```

### After

```text
多数のcontact sensor
    ↓
C++ aggregate node
    ↓
左右指ごとの集約状態
    ↓
friction_grasp_manager
```

---

## 注意

**センサのpublish rateを下げるだけでは、DARTの物理接触計算そのものは必ずしも減らない。**

この改善の主目的は:

```text
ROS通信
message変換
executor
callback
```

の削減である。

---

## 合格条件

- 接触開始を検出できる
- 接触解除を検出できる
- 把持成功判定維持
- 把持力制御維持
- friction_grasp_manager CPU低下

---

# 7. 改善項目6: GUI・描画負荷の低減

## 優先度

**中**

## 目的

物理計算以外のCPU使用を削減し、Gazebo serverへCPU資源を回す。

---

## 7.1 headless起動を安定化する

解析時にはheadless試行でグリッパ初期化失敗があったため、単純にheadlessへ変更するだけでなく初期化依存を確認する。

CodeXへ:

```text
headless:=true で
"amir2 gripper did not reach safe-open position"
が発生する条件を調査してください。

GUIの有無に依存しているTimerAction、
起動タイミング、/clock、controller activation、
gripper state判定を確認してください。
```

---

## 7.2 描画設定を下げる

候補:

- shadow無効
- anti-aliasing低減
- GUI camera更新頻度低減
-不要visual非表示
- 必要時のみGUI起動

---

## 7.3 推奨運用

### デバッグ時

```text
GUI ON
```

### 性能測定・長時間実験

```text
headless
```

---

## 合格条件

- headlessでも把持開始可能
- 通常実験完走
- GUI有無で物理挙動に大差なし
- RTF改善

---

# 8. 改善項目7: 制御周期・ROS通信周期の見直し

## 優先度

**中～低**

## 目的

物理ボトルネック改善後に残るCPU負荷を削減する。

---

## 重要

解析結果では、制御計算自体は主要ボトルネックではない。

したがってこの項目は、Collision / physics / pose / contact改善後に行う。

---

## 8.1 周期一覧を作る

CodeXへ:

```text
現在起動している全制御ノードについて、
Timer、publisher、sensor update rate、controller update rateを一覧化してください。

最低限:
- coordinator
- slip_monitor
- rotation_controller
- pivot_rotation_controller
- friction_grasp_manager
- dual_base_approach
- safety_monitor
- controller_manager
- odometry
- contact sensor
- force torque sensor
```

---

## 8.2 周波数を分類する

例:

### 高速維持候補

```text
低レベルモータ制御
接触安定化に必要な制御
```

### 低減可能候補

```text
状態監視
ログ
安全状態表示
高レベルBehavior Tree
経路状態判定
```

---

## 8.3 例

```text
50 Hz → 30 Hz
30 Hz → 20 Hz
10 Hz → 5 Hz
```

必ず一つずつ変更する。

---

## 合格条件

- 応答性維持
- 把持制御維持
- 経路追従性能維持
- CPU低下

---

# 9. 改善項目8: `TimerAction` をイベント駆動へ変更

## 優先度

**中**

## 目的

RTF低下時に実時間とシミュレーション時間の差によって、
「ロボット準備前に次処理へ進む」問題を防ぐ。

---

## 問題

例:

```text
launch開始から28秒後に次処理
```

は実時間ベースで進む場合がある。

一方、ロボットの制御やGazebo内部はシミュレーション時間基準となるため、
RTFが低いと準備状態とのズレが発生する。

---

## 9.1 状態イベントを定義する

例:

```text
/gripper_ready
/robots_ready
/grasp_complete
/support_removed
/transport_ready
```

---

## 9.2 推奨状態遷移

```text
START
  ↓
controllers_ready
  ↓
gripper_open_ready
  ↓
approach_complete
  ↓
grasp_complete
  ↓
support_removed
  ↓
transport_start
```

---

## 9.3 CodeXへの指示

```text
integrated_transport_simulation.launch.pyのTimerAction依存箇所を一覧化してください。

各TimerActionについて、
何を待っているのかを特定し、
可能なものはROS topic / service / lifecycle状態 / action result等を使った
ready eventへ置き換えてください。

固定時間待ちを可能な限り減らしてください。
```

---

## 合格条件

- RTFが0.3程度でも正常起動可能
- headlessでも正常起動可能
- 起動ごとの差が小さくなる

---

# 10. 推奨実装順序

CodeXでは以下の順番で進める。

```text
STEP 0
ベースライン保存
    ↓
STEP 1
指collision 9→3
    ↓
性能測定
    ↓
STEP 2
指collision 3→1
    ↓
性能測定
    ↓
STEP 3
max_step_size 0.003
    ↓
性能測定
    ↓
STEP 4
max_step_size 0.004
    ↓
性能測定
    ↓
STEP 5
不要collision除外
    ↓
性能測定
    ↓
STEP 6
pose/info削減
    ↓
性能測定
    ↓
STEP 7
contact集約
    ↓
性能測定
    ↓
STEP 8
headless安定化
    ↓
性能測定
    ↓
STEP 9
制御周期調整
    ↓
性能測定
    ↓
STEP 10
TimerActionイベント化
```

---

# 11. 比較実験マトリクス

最低限、以下を比較する。

| Test | Finger Collision | max_step_size | Pose Filter | Contact Aggregate | GUI |
|---|---|---:|---|---|---|
| A | Original | 0.002 | OFF | OFF | ON |
| B | 3 box | 0.002 | OFF | OFF | ON |
| C | 1 box | 0.002 | OFF | OFF | ON |
| D | 採用案 | 0.003 | OFF | OFF | ON |
| E | 採用案 | 0.004 | OFF | OFF | ON |
| F | 採用案 | 採用値 | ON | OFF | ON |
| G | 採用案 | 採用値 | ON | ON | ON |
| H | 採用案 | 採用値 | ON | ON | OFF |

---

# 12. 各実験の測定方法

## 起動

```bash
cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 通常測定

```bash
/usr/bin/python3 simulation_performance/20260928/measure.py \
  simulation_performance/20260928/<条件名> \
  180 false
```

## 再集計

```bash
/usr/bin/python3 simulation_performance/20260928/analyze.py
/usr/bin/python3 simulation_performance/20260928/summarize_profiles.py
PYTHONNOUSERSITE=1 \
/usr/bin/python3 simulation_performance/20260928/plot_results.py
```

---

# 13. CodeXに毎回実行させる確認テンプレート

各変更後、CodeXへ以下を実行させる。

```text
今回変更した内容について以下を確認してください。

1. 変更したファイル一覧
2. git diff
3. ビルド成功
4. launch成功
5. Gazeboエラーの有無
6. ROS node一覧
7. ROS topic一覧
8. 把持動作
9. 荷物落下
10. 異常振動
11. RTF
12. Update ms/step
13. CPU使用率
14. 改善前との比較

最後に
- 採用
- 保留
- 差し戻し
のいずれかを判定してください。
```

---

# 14. 差し戻し条件

以下のいずれかが発生した場合は、その改善を一旦戻す。

```text
把持失敗
荷物落下
接触判定消失
異常振動増大
滑り量増加
指定軸回転失敗
搬送物姿勢誤差の大幅増加
ロボット同士の衝突
障害物衝突を検出できない
RTF悪化
```

Gitで戻す例:

```bash
git diff
git restore <file>
```

ブランチごと破棄する場合:

```bash
git checkout main
git branch -D <branch>
```

---

# 15. 最終採用条件

最終構成は単純にRTF最大のものではなく、

```text
物理妥当性
+
把持安定性
+
搬送性能
+
再現性
+
計算速度
```

で決定する。

特に本研究では、協調搬送中の把持・滑り・姿勢・回転動作を評価するため、
接触モデルを軽量化しすぎて研究対象の現象そのものを失わないこと。

---

# 16. 最終目標

現状:

```text
max_step_size = 2 ms
Update ≒ 7.13 ms
RTF ≒ 0.26
```

第一目標:

```text
RTF >= 0.5
```

第二目標:

```text
RTF >= 0.8
```

理想目標:

```text
RTF ≒ 1.0
```

ただし、RTF=1.0のために把持・摩擦・滑りの再現性を失う場合は、
研究目的を優先してRTF目標を下げる。

---

# 17. CodeX向け最初の実行指示

以下をCodeXへ最初に入力する。

```text
このリポジトリの協調搬送シミュレーションについて、
simulation performance optimizationを段階的に実施します。

最初にコード変更は行わず、
次の項目だけ調査してください。

1. AMIR左右グリッパ指のcollision定義ファイル
2. 各指のcollision数
3. collision geometryと寸法
4. contact sensorとの対応
5. max_step_sizeの設定箇所
6. /world/cooperative_transport_friction/pose/info の全subscriber
7. finger contact sensorの全topic
8. integrated_transport_simulation.launch.py のTimerAction一覧

結果をMarkdown表で出力してください。

まだコードは変更しないでください。
調査結果を確認した後、
まず「指collision 9分割相当 → 3分割」から実装します。
```

---

# 18. 実装上の重要事項

- 一度に複数項目を変更しない
- 変更前後を必ず同じシナリオで比較する
- GUI負荷と物理負荷を混同しない
- センサ周期低下と物理接触計算削減を混同しない
- 制御周期低下とphysics step変更を混同しない
- `real_time_factor` の設定値変更だけで高速化しようとしない
- 把持・摩擦・滑りを研究対象とするため、collision簡略化後は必ず物理妥当性を確認する
- 最終判断はRTFだけでなく把持・搬送性能を含めて行う

---

## 完了チェックリスト

- [ ] ベースライン保存
- [ ] Collision 3box評価
- [ ] Collision 1box評価
- [ ] max_step_size 3ms評価
- [ ] max_step_size 4ms評価
- [ ] 不要collision整理
- [ ] pose/info軽量化
- [ ] contact通知集約
- [ ] headless安定化
- [ ] 制御周期調整
- [ ] TimerActionイベント化
- [ ] 全工程を通した性能測定
- [ ] 同一条件で複数回試験
- [ ] 最終構成決定
