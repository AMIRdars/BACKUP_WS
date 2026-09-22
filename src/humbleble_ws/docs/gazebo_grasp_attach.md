# Gazebo Pick&Place：把持中に RTF が落ちる問題と修正方針

作成日: 2026-07-04
対象: ign-gazebo (Gazebo Sim) + ROS 2 Humble での AMIR740 + Mecanum Rover3 pick & place

---

## 1. 症状

`pick_place_10_launch.py`（`jointfix10` ノード）で 10 個連続 pick & place を実行すると、
物体を**把持している間だけ RTF（Real Time Factor）が大きく低下**する。

再現手順:
```bash
source install/setup.bash && ros2 launch amir_gazebo gazebo_bringup.launch.py
source install/setup.bash && ros2 launch mecanumrover3_gazebo spawn_10box.launch.py
source install/setup.bash && ros2 launch amir_moveit_config moveit_gazebo.launch.py
source install/setup.bash && ros2 launch amir_operation pick_place_10_launch.py
```

## 2. 根本原因（確定）

**MoveIt では attach しているが、Gazebo では物理的に attach していない。**

- MoveIt 側: `MoveGroupInterface::attachObject()` を呼んでいる
  （`amir_operation/src/jointfix_10_pick_place.cpp:334` の `attachToGripper()`）。
  ただしこれは **PlanningScene 上の論理的なアタッチ**であり、planning/衝突回避のためのもの。
  Gazebo の物理エンジンには何の拘束も生成しない。
- Gazebo 側: 物体はグリッパの指との**摩擦接触だけ**で保持されている。
  スポーン SDF の接触は硬め（`kp=1e6, kd=1e3, mu=3.0`）で、
  把持中は指‐物体間の接触拘束を**毎フレーム解き続ける**ため計算コストが高く、RTF が落ちる。

イメージ:
```
MoveIt          : attachObject()  → PlanningScene 上で剛体化（物理には無関係）
Gazebo(現状)    : 指でずっと押さえ続ける → 接触拘束を毎フレーム求解 → RTF ダウン
```

## 3. 調査で判明した事実（環境の実測値）

| 項目 | 値 | 確認元 |
|---|---|---|
| シミュレータ | Gazebo Sim **6.18.0**（ign-gazebo6 / Fortress） | `ign gazebo --version` |
| ワールド名 | `/world/default` | spawn launch / bringup |
| ロボットのモデル名 | `amir_mecanum3` | `amir_gazebo/launch/gazebo_bringup.launch.py:83`（`create -name`） |
| MoveIt の attach 先リンク | `gripper_base_1` | `jointfix_10_pick_place.cpp:163-165` |
| 物体モデル名 | `target_obj_0` .. `target_obj_9` | `spawn_10box.launch.py`（`entity` + `_{i}`） |
| 物体リンク名 | `cylinder_link`（既定）/ `box_link` | `spawn_10box.launch.py`（shape で分岐） |
| 物体スポーン方法 | `ros_gz_sim create` で個別 SDF を生成・スポーン | `spawn_10box.launch.py` |
| DetachableJoint プラグイン | **インストール済み** `libignition-gazebo6-detachable-joint-system.so` | `/usr/lib/x86_64-linux-gnu/...` |
| attach/detach 両対応 | **対応**（`attach_topic` / `detach_topic` / `output_topic` をサポート） | `.so` の strings 確認 |

## 4. 修正方針：DetachableJoint による動的な物理アタッチ

ign-gazebo6 標準の **DetachableJoint システムプラグイン**を使い、
グリッパが物体を掴んだ瞬間に `gripper_base_1` ↔ 物体リンク間へ**固定ジョイントを動的生成**する。

- 固定ジョイントができると物体はグリッパと**運動学的に一体化**し、
  指‐物体間の接触拘束を解く必要がなくなる → RTF が回復。
- 離すときはジョイントを破棄して物体を解放する。
- MoveIt の `attachObject`/`detachObject` は planning 用に**そのまま残す**。
  → 「MoveIt = 論理アタッチ / Gazebo = 物理アタッチ」を**同じタイミングで二重に**張る。

代替案の比較（いずれも非推奨）:
- 接触パラメータ緩和（`min_depth`↑ / ソルバ反復↓）… 対症療法で保持が不安定化。
- 把持中だけ物体を `<static>` 化 / 毎フレーム teleport … 実装が汚く落下表現もできない。
- `gazebo_grasp_fix` プラグイン … Gazebo **Classic** 用であり ign では使えない。

→ **DetachableJoint 一択**。

## 5. 具体的な変更箇所（実装時のTODO）

### ① `mecanumrover3_gazebo/launch/spawn_10box.launch.py`
物体 SDF 生成ループで、モデルごとにプラグインを1つ注入する:

```xml
<plugin filename="ignition-gazebo-detachable-joint-system"
        name="ignition::gazebo::systems::DetachableJoint">
  <parent_link>{link_name}</parent_link>          <!-- cylinder_link / box_link -->
  <child_model>amir_mecanum3</child_model>
  <child_link>gripper_base_1</child_link>
  <attach_topic>/attach/{entity_name}</attach_topic>
  <detach_topic>/detach/{entity_name}</detach_topic>
  <suppress_child_warning>true</suppress_child_warning>
</plugin>
```
- プラグインは**物体モデル側**に置く（`parent_link` はそのモデル内のリンク＝物体リンク、
  `child_model`/`child_link` が相手＝ロボット）。1物体につき1インスタンス。
- 固定ジョイントなので parent/child の向きは物理的に等価。

### ② ros_gz_bridge
各 `/attach/target_obj_i` `/detach/target_obj_i` を
`std_msgs/msg/Empty ↔ ignition.msgs.Empty` で橋渡し（物体数ぶんループ生成）。
spawn launch に相乗りさせるか、専用 bridge launch を作る。

### ③ `amir_operation/src/jointfix_10_pick_place.cpp`
`std_msgs::msg::Empty` の publisher を追加し:
- `attachToGripper(arm, id)` の**直後**（`:334`）→ `/attach/target_obj_i` へ publish（物理アタッチ）
- 置く際、`controlGripper(..., GRIPPER_OPEN)`（`:340`）で**開く前**に
  `/detach/target_obj_i` へ publish（物理デタッチ）→ その後 `detachOnly` / gripper open

## 6. 実装時の注意点（ハマりどころ）

1. **初期状態の掃除**: DetachableJoint は環境によっては起動時に自動アタッチする。
   スポーン直後に各物体へ一度 `/detach` を publish し、
   **必ず「外れた状態」から開始**させると安全・確実。
2. **アタッチのタイミング**: 必ず「グリッパが閉じて物体を挟んだ後」に attach を送る。
   空中で早すぎると、その瞬間の相対姿勢のまま固定されてズレる。
   現状の `closeGripperGradually()` → `attachToGripper()` の順序ならOK。
3. **id とエンティティ名の一致**: cpp の `objId(i)` が生成する ID と
   Gazebo の `target_obj_i` を対応付け、topic 名を厳密に揃える必要がある（実装前に `objId()` の実体を確認）。
4. アタッチ中に指‐物体の衝突を無効化する必要はない（固定ジョイントが支配的になる）。
   さらに RTF が欲しければアタッチ中だけ物体 collision を無効化する手もあるが、まずは不要。

## 7. 検証方法

1. **手動検証（cpp 改修前）**: ①②だけ入れ、`ign topic` で手動 publish して RTF 改善を確認。
   ```bash
   ign topic -t /attach/target_obj_0 -m ignition.msgs.Empty -p ' '   # 物理アタッチ
   ign topic -t /detach/target_obj_0 -m ignition.msgs.Empty -p ' '   # デタッチ
   ```
   把持中に固定ジョイントが張られ RTF が戻ることを確認する。
2. **本番検証（③ 組込み後）**: 10個連続 pick&place を流し、
   - 把持中の RTF が非把持時と同等に保たれるか
   - 物体がグリッパと一体で運ばれるか（滑り落ちない）
   - 置いた後にきちんと解放され自由落下するか
   を確認。
3. 監視:
   ```bash
   ign topic -e -t /attach_state/target_obj_0   # output_topic を付けた場合、アタッチ状態
   ```

## 8. 進め方（案）

- 段階的: **①②を先に入れて手動 publish で RTF 改善を検証** → 確認後に ③ を組込む。
- 一括: ①②③をまとめて実装（`objId()` 確認後に topic 名を確定）。

## 参考（コード位置）

- `amir_operation/src/jointfix_10_pick_place.cpp`
  - `attachToGripper()` … `:163`（`group.attachObject(id, "gripper_base_1", TOUCH_LINKS)`）
  - `detachOnly()` … `:170`
  - `pickAndPlace()` の把持〜アタッチ … `:327`–`:334`
  - 置く〜デタッチ … `:340`–`:344`
- `mecanumrover3_ros2/mecanumrover3_gazebo/launch/spawn_10box.launch.py` … 物体 SDF 生成・スポーン
- `amir740_ros/amir_gazebo/launch/gazebo_bringup.launch.py:83` … ロボット `create -name amir_mecanum3`
