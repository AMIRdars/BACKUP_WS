# AMIR740 協調搬送：台車で接近 → 両台車停止 → アームのみでリーダー・フォロワー追従

**対象環境**：ROS 2 Humble ＋ Gazebo、AMIR740（メカナム移動台車＋5自由度アーム＋グリッパ）×2台  
**用途**：既存の協調搬送リポジトリを Codex で改修するための実装仕様書  
**作成日**：2026-11-12  
**範囲**：接近・把持後の**アームのみの並進追従**。追従中の台車移動、台車の再追従、物体回転、持ち替え、障害物回避は今回実装しない。

> **最優先要件**：開始時は両ロボットの**台車で物体の両側の把持可能位置へ近づく**。把持が完了したら**両台車は位置を保持して移動しない**。リーダーの**アームだけ**を動かして物体を並進させ、フォロワーは**把持点に生じる力からアドミッタンスで目標を生成し、アームだけ**で追従する。この一連の動作を Gazebo 上で視覚・ログの双方から確認する。

---

## 0. Codex に最初に伝える実装原則

1. **既存リポジトリの構成を必ず調査**し、既存の launch、TF、URDF、`ros2_control`、グリッパ把持、アームIK、力センサを再利用する。未確認の ROS トピック・コントローラ名を存在すると決めつけない。
2. 現行の**台車追従P制御**（例：`v_b = sat(K_b e_arm)`）は、`ARM_ONLY` 中には呼び出さない。状態が `ARM_ONLY` に入る前に Nav2 等の台車指令元を停止・無効化し、速度指令をゼロにする。
3. **リーダーだけに運動目標**を与える。フォロワーの本制御入力にリーダーの位置・速度・目標手先位置・物体の真値を使用しない。フォロワーは自身の力覚・関節角・運動学で応答する。
4. 初期動作は、**両台車停止＋物体の低速短距離直線並進**のみ。アームの姿勢・可操作域と5自由度の限界を事前確認し、物体回転・任意6D姿勢制御は混ぜない。
5. 把持されている**同一剛体物体をアーム2本が動かす**。Gazebo上で物体の姿勢を毎フレーム直接テレポートさせたり、フォロワーの手先目標を物体真値から生成したりしない。
6. 停止指令 `cmd_vel=0` は物理的な台車固定を保証しない。**車輪側の保持能力とodom/TFの実移動量**を監視し、外力で台車が動く場合は駆動制御・保持方式を修正する。検証用の世界固定拘束を使用した場合は、通常の台車停止制御とは明確に区別する。
7. 固定把持・高ゲイン位置制御による**過拘束と内部力の増大**に注意する。アームの位置・速度・加速度・接触力を制限し、安全停止を優先する。
8. Gazebo の物理演算は既存のシミュレーション管理に任せ、制御ノードから独自に二重ステップを行わない。`/clock` と `use_sim_time` を使用し、制御更新は統一した周期で処理する。
9. センサが無い場合は架空の `/wrench` を仮定して完成扱いにしない。**運動学単体テスト → 擬似力入力テスト → Gazebo力覚計測による本実験**を分ける。
10. 実装後は**既存機能を破壊せず**、新しいモードとして切り替えられること。台車追従制御・回転制御はそのまま温存するが、今回の実験時は絶対に有効化しない。

---

## 1. 実現したい動作・完成条件

### 1.1 時系列

| 段階 | リーダー台車 | リーダーアーム | フォロワー台車 | フォロワーアーム | 物体 |
|---|---|---|---|---|---|
| ① 接近 | 物体へ接近 | 接近姿勢を取る | 物体へ接近 | 接近姿勢を取る | 初期位置 |
| ② 把持 | 停止 | 把持位置に到達し把持 | 停止 | 把持位置に到達し把持 | ほぼ静止 |
| ③ 保持確認 | **位置保持** | 初期姿勢を保持 | **位置保持** | 初期姿勢を保持 | 静止 |
| ④ リーダー始動 | **位置保持** | **手先を低速並進** | **位置保持** | 把持反力から追従を開始 | アームに引かれて移動 |
| ⑤ 追従継続 | **位置保持** | 目標手先軌道を追従 | **位置保持** | アドミッタンス＋IKで追従 | 並進 |
| ⑥ 停止 | **位置保持** | 到達位置を保持 | **位置保持** | 到達位置で安定保持 | 静止 |

**図解（側面の概念、実際の把持配置はURDFに依存）**

```text
〔接近〕
  Lベース --->     [ 搬 送 物 ]     <--- Fベース
       Lアーム                         Fアーム

〔把持・ベース停止〕
  [ Lベース ] -- Lアーム -- [ 物体 ] -- Fアーム -- [ Fベース ]
       |                                        |
    位置保持                                  位置保持

〔アームだけで協調並進〕
  [ Lベース ] -- Lアーム→ [ 物体→ ] ←Fアーム -- [ Fベース ]
       |                         ↑              |
    移動しない           外力に応じて追従       移動しない
```

### 1.2 完成判定（数値は**初期試験の暫定値**）

- 接近の完了・把持完了後、両台車の走行指令はゼロ、Nav2等の追従指令も停止している。
- 両台車の初期保持姿勢に対する移動量が試験中に**平面距離 0.01 m 未満、ヨー差 1°未満**を目標とする（物理的に固定できるか確認）。限界値の正式な安全基準ではない。
- リーダーアームが**0.03 m（初期テスト）**を低速で移動し、フォロワーアームの関節角・手先位置にも**同じ物体並進に対応する変化**が生じる。
- 物体および両把持点に大きな飛び・破綻がない。グリッパが開かず、物体を落とさない。
- フォロワーはリーダーの状態情報や物体真値を使わず、**自身の力覚**で応答している。
- フォロワーの外力偏差が異常に増大しない。`ARM_ONLY` 中に既存の台車P制御が走らないことをログで確認する。

### 1.3 スコープ外

- 物体中心まわりの旋回、アームと台車の協調Whole-body IK、台車追従への自動切替
- 未把持状態からの汎用自律把持計画、未知障害物の自律回避、リーダー・フォロワーの役割変更
- 実機の安全認証／衝突安全性評価

---

## 2. 既存プログラムを調査してから変更する

Codexは実装前に以下を抽出し、**実在する名称とファイル位置の対応表**をREADMEまたは報告に提示すること。

| 調査対象 | 確認内容 |
|---|---|
| シミュレータ | Gazebo Classic / Gazebo Sim（Ignition系）と使用launch |
| ロボット2台 | namespace・TF prefix・`base_link`、アーム基準座標系、手先リンク |
| 台車 | メカナムの速度指令入力、車輪コントローラ、停止保持の方法、odom |
| アーム | 5関節の名前・順序・URDF制限、IKとヤコビ行列、指令インタフェース |
| グリッパ | 開閉API、2台同時把持の完了条件、固定ジョイント／摩擦接触の方式 |
| 力センサ | F/Tセンサの設置、topic名、型、フレーム、更新周波数、飽和・ノイズ |
| 実験対象物 | 形状・寸法・質量・把持点・固定ジョイントのパラメータ |
| 競合ノード | Nav2、既存全身制御、手先追従、速度スムーザ等の二重指令 |
| 時刻・ログ | `/clock`、`joint_states`、TF、csv/loggerの既存仕組み |

例：以下は**探索コマンド**。実際の topic・controller は存在確認後に使用する。

```bash
ros2 node list
ros2 topic list -t
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action list
ros2 service list
ros2 param list
```

既存にアドミッタンス機能があるなら重複実装せず、**台車指令を遮断したモード**を設ける。既存には全身IKしかない場合は**アームの5関節のみのヤコビ行列**へ切り替える。

---

## 3. 状態遷移：接近とアーム追従を厳密に分離

### 3.1 ステートマシン

```text
INIT
  |
  v
BASE_APPROACH    : 両台車の移動許可。指定した事前把持姿勢へ接近。
  |
  | 位置・向き誤差が閾値内、障害物なし
  v
BASE_STOP        : Nav2などを停止、速度指令0、車輪の停止/位置保持。
  |
  | odom・wheel速度が許容値内で安定
  v
DUAL_GRASP       : アームで両側の把持姿勢へ移動し、両グリッパ把持。
  |
  | 両把持確認、関節限界・物体高さ・過大接触なし
  v
SETTLE_AND_BIAS  : 台車保持・アーム保持、静的荷重で力覚バイアス設定。
  |
  | センサ・TF・各状態が安定
  v
ARM_ONLY         : 両台車停止。リーダーアーム指令、フォロワーアーム力追従。
  |
  | リーダー目標到達
  v
HOLD             : 両アームで保持、台車停止、ログ保存。

どの状態からも異常 → ABORT（リーダー軌道停止、追従停止、安全側の保持）。
```

**順序の要点**：原則、台車接近を先に完了させ、その後で最終把持する。両把持を維持した状態で接近用の台車移動をしない。接近しながら把持する複合動作は今回対象外。

### 3.2 `BASE_APPROACH`

- 既存のNav2／台車移動機能で、各ロボットを**物体両側の把持可能な事前配置**へ移動。
- 目標はリーダー・フォロワーでそれぞれの台車目標姿勢を持つ。物体や相手ロボットへの衝突を避ける。
- この段階に限りベース運動を許可する。アームは事前把持用の安全姿勢を保持。
- 接近目標は**台車姿勢**であり、グリッパ先端の把持到達可否も事前IKで確認する。
- `BASE_APPROACH → BASE_STOP` の際に速度目標をゼロへ滑らかに下げ、ナビゲーションgoalを終了・キャンセルする。

### 3.3 `BASE_STOP` と固定保持

- 両ロボットについて既存の台車制御を**停止・競合排除**し、追従用コントローラの台車速度出力を恒常的にゼロにする。
- 車輪速度制御がある場合は0速度保持し、odomのドリフトを監視。停止指令のみでは外力で転がる場合、既存の車輪位置保持や停止サーボを利用してよい。**これは台車を移動させて物体に追従する制御ではなく、指定位置の保持である**。
- ホイールを強制的に無効化して自由回転にしない。`cmd_vel=0`の発行元は可能な限り1箇所にする。
- なお検証用にGazebo worldとの固定ジョイントを仮設する方法は**別途オプション**。デフォルトで用いず、結果を実機相当の台車位置保持と混同しない。

### 3.4 `DUAL_GRASP`

- 物体の左右把持位置まで各アームをゆっくり動かし、2台がグリッパで保持する。
- 既存実装が把持後の初期状態をロードできる場合はそれを再利用してよい。ただし**両台車がBASE_STOPを通ったこと**は記録。
- `fixed joint` 型把持では、同一剛体に2本の手先を強拘束するため、位置指令を一致させないと内部応力が急増する。初期位置・接触位置・手先目標を合わせ、二重拘束状態を検証する。
- グリッパの開閉・固定ジョイント生成・接触判定は既存処理に合わせ、勝手に新たな架空APIを作らない。

### 3.5 `SETTLE_AND_BIAS`

- 両アーム保持・両台車停止で物体が十分静止したことを確認。力覚センサが安定した区間の平均値を**初期負荷基準**として保存。
- `F_bias` はセンサ固有のオフセットに加え、姿勢一定のもとでの静的荷重を差し引くための基準とする。物体が動いて手先姿勢が変化する場合は重力成分が座標変換により変わるため、可能ならセンサバイアスと重力補償を区別して実装する。
- リーダー・フォロワー各アームの初期姿勢と基準手先位置、両台車のロック開始時の平面姿勢を記録。
- センサ欠落、NaN、時刻不整合、過大外力があれば `ARM_ONLY` に進まない。

### 3.6 `ARM_ONLY`

**この状態で必須の不変条件**

```text
leader_base_target_twist   = [0, 0, 0]
follower_base_target_twist = [0, 0, 0]
Nav2/既存base追従 = inactive
whole_body_base_assist = disabled
leader_arm = 時間軌道に基づく手先並進制御
follower_arm = 力覚アドミッタンス + アームのみ逆運動学
```

`ARM_ONLY` 中は台車が保持位置からずれたら**台車追従に切り替えず、異常として停止する**。上位制御側からの誤った速度コマンドは下流でも遮断してログに残す。

---

## 4. 制御座標系・剛体把持条件

| 記号 | 意味 |
|---|---|
| `W` | Gazeboワールド基準 |
| `B_L`, `B_F` | リーダー、フォロワー台車基準フレーム |
| `E_L`, `E_F` | 各手先・把持点フレーム |
| `S_F` | フォロワー F/T センサフレーム |
| `O` | 物体フレーム |
| `q_L,q_F∈R^5` | 各アームの関節角 |
| `p_{E,i}^{B_i}` | 各台車から見た手先位置 |
| `v_{E,i}^{B_i}` | 各台車から見た手先並進速度 |
| `F_{ext}^{B_F}` | 物体からフォロワー手先へ加わる力（変換・補正済み） |

台車保持下では `B_L`,`B_F` の世界姿勢は（許容範囲で）一定である。物体を剛体把持した場合、把持点の世界姿勢は次を満たす。

\[
{}^{W}T_{E,i} = {}^{W}T_O\,{}^{O}T_{E,i},\quad i\in\{L,F\}.
\]

両腕は同一物体と接続されるため、要求された物体並進は各把持点に反映される。しかしそれだけで**力の内部ループが安定するとは限らない**。物体姿勢・把持接触をログで監視する。

### 座標と符号の検証

- リーダー・フォロワーが向かい合うと、互いの台車ローカル`+X`の方向が逆になりうる。両ロボットに同じローカル速度をそのまま流さない。
- フォロワー力センサで読み出した力は、**センサ軸から`B_F`へ回転変換**して使う。トルクも使う場合は参照点移動時の `r×F` を含める。
- 力の符号が「手先が物体に及ぼす力」なら反作用の符号を反転する。
- **単体極性テスト**：既知のワールド方向に物体を数mm押したとき、フォロワー手先の目標が物体移動方向へ変化することを確認。逆向きなら符号・TFの扱いを修正。

---

## 5. リーダーアーム：事前軌道に従い物体を動かす

今回リーダー台車は動かさない。リーダーアームにのみ**低速の手先目標軌道**を与える。

初期テストは次を推奨する（未確認の実機限界ではなく**試験開始値**）：

```yaml
leader_arm_trial:
  translation_axis: "reachable_direction_in_leader_base"  # 運動学・把持配置から決定
  translation_m: 0.03       # 慎重に3cmから
  max_linear_speed_mps: 0.01
  max_linear_accel_mps2: 0.03
  hold_after_motion_s: 2.0
  return_to_start: false    # 初期試験では戻さず保持、別試験で往復
```

- 把持開始時の手先位置を `p_{L,0}` とし、各時刻の目標を滑らかな S 字補間（minimum-jerk等）で生成：

\[
\boldsymbol p_{L,des}(t)=\boldsymbol p_{L,0}
+\boldsymbol d\,s(u),\quad
s(u)=10u^3-15u^4+6u^5,
\]

\[
 u=\operatorname{clip}\!\left(\frac{t-t_0}{T},0,1\right),
\quad \|\boldsymbol d\|=0.03\,\mathrm{m}.
\]

- `T` は所定速度・加速度を超えないよう選択（例：`T >= 1.875 * distance / vmax` は上のminimum-jerk軌道の最大速度に基づく下限）。さらに加速度制約も確認する。
- リーダー側も**アームのみのIK**を使い、ベースの自由度を含めない。
- まず手先の**並進2自由度または3自由度**だけを主タスクとする。手先姿勢を完全固定できるかは5関節の運動学による。剛体把持での姿勢破綻を防ぐため、実現可能な姿勢制約を副次タスクとして扱うか、並進方向を制限する。
- 物体に直接速度・位置指令を与えない。リーダーが物体を動かす駆動源となること。

---

## 6. フォロワーアーム：力覚アドミッタンスによる追従

### 6.1 入力

- `WrenchStamped`等で取得した把持部の力・モーメントを使用。今回の主タスクは**水平XY並進**（実際の手先可動方向に合わせる）。
- `F_meas` はセンサ座標系から `B_F` へ変換、重力・オフセットを補正し、`F_ref` または静的把持負荷基準を引く。
- ノイズ対策にローパスフィルタ（例：時定数0.03–0.1秒）と小さなデッドバンドを設ける。**フィルタは制御遅れを生む**ため実機適用前に再調整する。

### 6.2 最初の実装は「速度型アドミッタンス」

図で使用していた二次系アドミッタンス

\[
M\Delta\ddot{\boldsymbol x}+D\Delta\dot{\boldsymbol x}+K\Delta\boldsymbol x
=\boldsymbol F_{ext}-\boldsymbol F_{ref}
\]

のうち、今回の**連続移動への追従検証では `K=0`** とし、速度型にする。

\[
\boxed{M_a\dot{\boldsymbol v}_{adm}+D_a\boldsymbol v_{adm}
=\boldsymbol F_{err}}
\]

- `M_a`: 仮想慣性 [kg]
- `D_a`: 仮想減衰 [N·s/m]
- `v_adm`: フォロワーの手先目標並進速度 [m/s]
- `F_err = F_ext - F_ref`: 把持力偏差 [N]。姿勢一定の初期把持荷重を基準として除去する。

**理由**：`K>0` かつ基準位置を固定すると、一定の物体変位を維持するために静的な復元力が残りやすい。今回は台車を動かせず、物体に追従した新しい手先位置で停止する必要があるため、まず `K=0`、速度制限・作業空間制限付きで検証する。バイアス誤差によるドリフトがあり得るため、その対策は必須。

離散時間 `dt` の例：

\[
\boldsymbol a_k = M_a^{-1}(\boldsymbol F_{err,k}-D_a\boldsymbol v_{adm,k}),
\]
\[
\boldsymbol v_{adm,k+1}=\operatorname{limit}_{v,a}
(\boldsymbol v_{adm,k}+dt\,\boldsymbol a_k),
\]
\[
\boldsymbol p_{F,ref,k+1}^{B_F}=
\operatorname{project}_{\mathcal W_{safe}}\left(
\boldsymbol p_{F,ref,k}^{B_F}+dt\,\boldsymbol v_{adm,k+1}
\right).
\]

- `p_F_ref` はフォロワーが**自身の力覚から積分する手先位置目標**。物体の真値・リーダー位置を参照して更新しない。
- `project` によって仮想作業空間、関節可動域へ入る目標に制限する。目標がクランプされたら積分を止め、内部速度状態を適切に再設定する（アンチワインドアップ）。
- 移動開始・停止の不連続や数値発散を防ぐため、`dt`の異常値／未更新センサ／NaNを拒否する。
- `F_err`のX/Y各軸の微小値はデッドバンドで除去し、停止中のドリフトを減らす。

### 6.3 アームのみの逆運動学（IK）

現在のフォロワーアーム関節角 `q_F∈R^5` から、URDFのFKと**台車固定時のアームヤコビ行列**を計算する。初期実装では水平面の2自由度タスクを使用する。

\[
J_{a,xy}(\boldsymbol q_F)\in\mathbb R^{2\times5}
\]

\[
\boldsymbol v_{E,cmd}^{B_F}=
\boldsymbol v_{adm}+K_p
\big(\boldsymbol p_{F,ref}^{B_F}-\boldsymbol p_{F,meas}^{B_F}\big)
\]

\[
\boxed{\dot{\boldsymbol q}_F=
J_{a,xy}^{\#}\boldsymbol v_{E,cmd}^{B_F}}
\]

\[
J_{a,xy}^{\#}=
J_{a,xy}^{T}\left(J_{a,xy}J_{a,xy}^{T}+\lambda^2 I\right)^{-1}.
\]

- DLS（減衰付き擬似逆）を使う。特異姿勢に近いときは減衰を増やす。
- `qdot`・`q_cmd` は**関節位置、速度、加速度、トルク（取得可能なら）**の限界内に制限。アーム姿勢の安全域が無くなったら、**台車を動かさずABORT**する。
- 関節位置指令しか無い場合は、毎周期の関節角度指令を**滑らかに積分**し、追従可能な周期・ゲインに調整する。ロボット側の位置サーボが過大な接触力を生じないか監視。
- 実際の把持拘束で手先姿勢固定が必要な場合は、制御できる姿勢自由度を検討し、拘束可能性と5自由度の限界を評価。**XYだけ追従して6自由度の剛体把持を必ず保てると仮定しない**。

### 6.4 フォロワーに入れてはいけない情報

`ARM_ONLY` の本制御ループでは、次のものを追従目標生成に使用してはならない。

- `/leader/odom`、リーダー関節角、リーダーの手先目標
- Gazeboの物体真値（`/model_states`相当）、物体リンクのworld pose
- 共通の世界座標目標を両ロボットへ直接配布する「2台同期の位置追従」方式

**例外**：観測用のログ、可視化、誤差評価には使用してよいが、追従計算には入力しない。力センサが存在しない場合のテストモードでは、真値・擬似力を使用したことを明示し、力覚ベースの協調追従ができたと判定しない。

---

## 7. 力覚が未実装の場合の進め方

**優先順を固定**する。

1. **単体IKテスト**：ベース停止、物体未把持でリーダー・フォロワーのアームそれぞれに微小XY目標を与え、IKと関節方向を確認。
2. **擬似力入力テスト**：フォロワー単体に既知の力（例：+Xへ数N）を注入し、力方向に手先が動き、力解除時に停止することを確認。この試験では物体真値を使用しない。
3. **F/Tセンサ接続**：使用中のGazeboのセンサ方式に合わせ、グリッパ／手首の力覚を取得。センサ軸の変換・反力符号・重力基準を検証。
4. **2台把持試験**：接近→停止→両把持→バイアス→リーダー短距離移動→フォロワー力追従。

`joint_states.effort` から直接 `F=J^{-T}τ` と決めつけない。推定時には重力、関節摩擦、慣性、軸数とヤコビ行列の階数を考慮し、推定精度を確認する必要がある。**センサの欠如を隠したまま本試験へ進まない。**

---

## 8. 初期試験パラメータ（未調整の参考値）

このYAMLは**提案するパラメータ構造の例**であり、既存コードに同名フィールドがあることは保証しない。必ずロボットのURDF／アクチュエータの限界、Gazeboの応答に合わせて調整する。

```yaml
arm_only_experiment:
  enabled: true
  controller_rate_hz: 100.0
  use_sim_time: true

  base:
    allow_motion_in_states: [BASE_APPROACH]
    command_zero_during: [BASE_STOP, DUAL_GRASP, SETTLE_AND_BIAS, ARM_ONLY, HOLD]
    max_position_drift_m: 0.010
    max_yaw_drift_deg: 1.0
    stable_time_s: 1.0
    disable_base_follow_controller: true
    world_fixed_constraint_debug_only: false

  grasp:
    settle_time_s: 2.0
    require_both_grasps: true
    require_sensors_before_arm_only: true

  leader_arm:
    motion: minimum_jerk_translation
    distance_m: 0.030
    max_speed_mps: 0.010
    max_accel_mps2: 0.030
    hold_after_s: 2.0

  follower_arm:
    task_axes: [x, y]
    admittance:
      mode: velocity
      M_kg: [1.5, 1.5]
      D_Ns_per_m: [120.0, 120.0]
      K_N_per_m: [0.0, 0.0]
      force_filter_tau_s: 0.05
      force_deadband_N: 0.8
      max_target_speed_mps: 0.035
      max_target_accel_mps2: 0.15
      max_relative_displacement_m: 0.06
    ik:
      method: damped_least_squares
      damping_lambda: 0.05
      cartesian_position_kp: 2.0
      max_joint_speed_radps: 0.4
      joint_limit_margin_deg: 10.0

  watchdog:
    max_sensor_age_s: 0.10
    excess_force_warning_N: 8.0
    excess_force_abort_N: 20.0
    max_joint_tracking_error_rad: 0.15
    stop_on_base_drift: true
    stop_on_grasp_loss: true
```

**注意**：`max_relative_displacement_m`はアドミッタンス目標の初期姿勢からの変位に対する暫定制限。過大な把持力の判定は**静荷重を取り除いた追従軸の力**について行い、荷重支持方向の絶対値と混同しないこと。安全閾値はハードウェア仕様ではなく初期シミュレーションの試験用数値である。

---

## 9. ROS 2 での実装構成（サンプル、名称は既存構成へ合わせる）

```text
arm_only_cooperative_transport/
├── launch/
│   └── two_amir_arm_only_test.launch.py
├── config/
│   └── arm_only_test.yaml
├── src/ または package_name/
│   ├── experiment_state_machine.*
│   ├── leader_cartesian_trajectory.*
│   ├── follower_wrench_admittance.*
│   ├── arm_kinematics_adapter.*
│   ├── base_stop_guard.*
│   └── experiment_logger.*
├── test/
│   ├── test_admittance_sign_and_stability.*
│   ├── test_arm_jacobian_and_limits.*
│   ├── test_base_motion_inhibit.*
│   └── test_state_transitions.*
└── README.md
```

新パッケージを強制する構成ではなく、既存のアーム・台車・グリッパ制御を使って**足りない処理のみ追加**する。controller競合を起こす独立ノードを増やさない。

### 9.1 内部コンポーネント境界

| 部品 | 入力 | 出力 | 注意 |
|---|---|---|---|
| Experiment FSM | 把持、台車停止、センサ健全性、実験状態 | 状態と実行許可 | 異常時停止への単一路 |
| Base Approach Adapter | 2台の接近目標 | Nav2等の既存台車指令 | **接近時のみ**動く |
| Base Stop Guard | FSM、odom、cmd_vel、車輪状態 | 走行指令遮断・位置保持異常判定 | 外部競合を検出 |
| Leader Arm Controller | リーダー初期手先、目標変位、joint states | リーダー関節指令 | 台車速度指令なし |
| Follower Wrench Admittance | フォロワー力覚、TF、joint states | フォロワー手先目標 | 相手状態を参照しない |
| Arm IK Adapter | 手先目標、関節角、URDF | 5関節指令 | ベース自由度を含めない |
| Logger | 両者の制御・状態・物体真値 | CSV等 | 真値は制御入力へ逆流させない |

### 9.2 制御更新擬似コード

```python
# 擬似コード：実トピック、controller API、関節名は調査して置換

def update(dt, now):
    state = fsm.state
    read_robot_states_and_tf()

    if state == "BASE_APPROACH":
        run_existing_base_approach_for_both_robots()
        hold_safe_arm_pregrasp_pose()
        if both_robots_at_pregrasp_goals():
            stop_nav_goals_and_base_commands()
            fsm.set_state("BASE_STOP")
        return

    # ここより先は常に両台車の移動を禁止
    base_guard.stop_or_hold_both_bases()
    base_guard.block_all_base_follow_commands()

    if base_guard.drift_exceeded() or unexpected_base_command_detected():
        safe_abort("base moved or competing base command")
        return

    if state == "BASE_STOP":
        if base_guard.both_bases_stationary_for_required_time():
            fsm.set_state("DUAL_GRASP")
        return

    if state == "DUAL_GRASP":
        operate_existing_arm_and_gripper_grasp_sequence()
        if both_grasps_verified():
            fsm.set_state("SETTLE_AND_BIAS")
        return

    if state == "SETTLE_AND_BIAS":
        hold_both_arms()
        if poses_and_wrench_are_stable():
            follower_calibrate_wrench_and_home_pose()
            leader_initialize_cartesian_trajectory()
            fsm.set_state("ARM_ONLY")
        return

    if state == "ARM_ONLY":
        if not all_safety_inputs_fresh_and_valid(now):
            safe_abort("sensor or control-state failure")
            return
        # リーダー：時間軌道→自分のアーム5関節だけ
        leader_target = leader_trajectory.evaluate(now)
        qd_L = leader_arm_only_ik(leader_target)

        # フォロワー：力覚→アドミッタンス→自分のアーム5関節だけ
        F_ext_F = follower_wrench_to_follower_base_frame()
        v_adm, p_ref_F = follower_admittance.step(F_ext_F, dt)
        qd_F = follower_arm_only_ik(v_adm, p_ref_F)

        if any_arm_or_grasp_safety_violation(qd_L, qd_F):
            safe_abort("arm limits or grasp force")
            return
        send_safe_joint_commands(qd_L, qd_F)
        assert base_guard.command_is_zero_for_both()
        log_state_and_robot_data()
        if leader_trajectory.finished_and_stable():
            fsm.set_state("HOLD")
        return

    if state == "HOLD":
        hold_both_arm_joint_targets()
        base_guard.stop_or_hold_both_bases()
        log_state_and_robot_data()
        return

    if state == "ABORT":
        base_guard.stop_or_hold_both_bases()
        stop_leader_motion_smoothly_if_safe()
        stop_follower_motion_smoothly_if_safe()
        keep_gripper_closed_when_safe()
        emit_failure_diagnostics()
```

**ABORTの安全動作に関する注意**：物体を剛体把持したまま一方のアームを急停止・無力化すると、物体落下や拘束力増加を引き起こし得る。まずリーダーの目標変位生成を止め、可能な範囲で双方のアームを協調的に減速し、グリッパを維持する。物理シミュレーションで異常時パターンを個別検証する。

---

## 10. 実装手順：小さな段階ごとに動作確認

### Step 0 — 現状把握

- `launch`、ロボット名前空間、IK、物体把持、力覚トピック、既存 base/arm の指令元を特定。
- 既存実装で、アーム追従よりも台車が優勢になっている原因（同時走行指令、全身IK重み、実際のアーム関節指令が更新されていない等）をログで確認。
- 既存モードは残し、専用パラメータ `arm_only_experiment.enabled` で切り替える。

### Step 1 — 物体に接近して両台車を停止

- 2台の台車で把持可能位置へ移動する既存経路を使う。
- 停止後に全ての走行指令がゼロ、車輪が保持、odomが安定することを確認。
- **合格条件**：把持・アーム追従に移行しても台車への新たな走行指令が一切出ない。

### Step 2 — 物体の両側を把持する

- 2台のアームで把持、物体の重力荷重で静止保持できることを確認。
- Gazeboのjointやcontact constraintが過剰拘束になっていないか、初期接触力・物体の姿勢跳躍を確認。
- **合格条件**：静止保持で把持が外れない、強制的な物体ジャンプや持続的な過大接触力がない。

### Step 3 — リーダーアーム単体の小変位移動

- まず**フォロワーを把持させない単体試験**を実施し、リーダー台車停止で物体を低速0.01〜0.03 m移動できるか検証。
- この試験はフォロワーとの拘束を切るため、協調搬送成功とは扱わない。
- **合格条件**：5自由度のIKが指示方向に正しく動き、物体運動の方向・距離が妥当。

### Step 4 — フォロワーアドミッタンス＋アームIK単体試験

- 擬似力で符号、フィルタ、停止応答、限界動作、速度制限を確認。
- `+X`に外力を与えると`+X`に手先が逃げるかを**同じ座標系**で確認。
- **合格条件**：台車停止、アーム関節・手先だけが動く。力を解除すると手先**速度が減衰して停止**する（`K=0`なので手先位置が自動的に初期位置へ戻るわけではない）。

### Step 5 — 両把持＋力覚追従

- `BASE_APPROACH→...→ARM_ONLY` の一連動作を実行。
- リーダーアーム0.03mの移動中、フォロワーの`F_ext`、アドミッタンス目標、関節角、台車姿勢を全て取得。
- 両台車が停止していること、フォロワーの手先追従が成立していることを確認。
- **合格条件**：両台車停止・両アーム移動・物体並進・把持維持が同時成立する。

### Step 6 — 微小往復と条件変更（Step 5 合格後のみ）

- まず停止→往復、次に移動距離0.03→0.05m、速度0.01→0.02m/sなどを**1条件ずつ**変更。
- 上記は試験例。前提の到達可能性や力制限が満たされない条件は実施しない。
- 物体の回転・台車追従の追加には進まない。

---

## 11. ログ・可視化・合否判定

### 11.1 CSV項目（最低限）

```text
timestamp, state, trial_id,
leader_base_x, leader_base_y, leader_base_yaw,
follower_base_x, follower_base_y, follower_base_yaw,
leader_cmd_vx, leader_cmd_vy, leader_cmd_wz,
follower_cmd_vx, follower_cmd_vy, follower_cmd_wz,
leader_arm_q1..q5, follower_arm_q1..q5,
leader_ee_x, leader_ee_y, leader_ee_z,
follower_ee_x, follower_ee_y, follower_ee_z,
leader_ee_target_x, leader_ee_target_y,
follower_ee_target_x, follower_ee_target_y,
follower_wrench_raw_fx, follower_wrench_raw_fy,
follower_force_filtered_fx, follower_force_filtered_fy,
follower_v_adm_x, follower_v_adm_y,
object_world_x, object_world_y, object_world_yaw,
grasp_state_leader, grasp_state_follower,
base_drift_leader_m, base_drift_follower_m,
failure_reason
```

`object_world_*`は**評価・描画専用**。センサ真値を本制御へ利用しない。

### 11.2 必須グラフ

1. 時間‐**両台車の並進移動量**（`ARM_ONLY`開始以降ほぼ0であること）。
2. 時間‐**リーダー・フォロワー各手先の変位量**（両方変化すること）。
3. 時間‐**フォロワーの把持部外力偏差とアドミッタンス目標速度**。
4. 時間‐**物体位置と両把持点の世界位置**（剛体把持が破綻していないこと）。
5. 時間‐各アーム関節角・速度と限界への余裕。

### 11.3 失敗を明確に判定するケース

- フォロワーが動き始める前に台車が動く／他ノードが`cmd_vel`を上書きする。
- 物体をリーダーのアームで動かさず、オブジェクト真値を直接更新する。
- フォロワーは力覚追従を名乗るが、実際にはリーダー目標・物体真値を利用している。
- 2台のアームがほぼ動かないのに物体だけ移動する。
- 力センサの極性が逆、センサ停止で最後の力を積分し続ける。
- 固定把持の力が発散、把持が外れる、関節限界・過大ドリフトが発生。

---

## 12. 安全性・よくある問題への対応

| 症状 | 主な原因候補 | 対応方針 |
|---|---|---|
| 台車が少し動く | ゼロ速度だけで車輪が外力に押される／競合走行指令 | odomを観測し、車輪位置保持・競合排除を確認 |
| リーダーアームが動かない | 全身IKや台車優先ゲインが残存 | 台車自由度をIKから除外、関節指令実送信を確認 |
| フォロワーアームが動かない | 力センサが未接続、符号・軸違い、死域過大、IK不成立 | 擬似力注入→座標変換→手先目標→関節目標の順に確認 |
| フォロワーが逆方向に動く | ロボットが対向、センサ符号の取り違え | `S_F→B_F`変換と反作用の向きを確認 |
| 物体が振動する | アドミッタンスの帯域過大、高ゲイン位置サーボ、硬い固定拘束 | 小速度・小ゲイン・減衰増加・joint constraintを見直し |
| 移動後もフォロワーが流れ続ける | `K=0`で力のオフセットが積分される | バイアス・重力補償、デッドバンド、速度停止条件、位置範囲制限 |
| 把持直後に力が大きい | 二重固定の初期位置が不整合 | 把持点・アーム姿勢・constraint初期化を修正 |
| アーム可動域が不足 | 5自由度・手先姿勢拘束・ベース停止 | 初期台車停止位置と物体移動方向・距離を縮小 |

**緊急停止**：機械的限界値を知らない状態で非安全な動きを許容しない。力・位置・速度・関節限界・台車ドリフト・センサ途絶がいずれか異常なら上位軌道を安全停止。把持解除を自動実行しない（落下リスク）。

---

## 13. Codex 実装依頼文（このままコピー可能）

```text
現在のROS 2 Humble＋GazeboのAMIR740 2台協調搬送コードを改修してください。
添付の「AMIR740 協調搬送：台車で接近 → 両台車停止 → アームのみでリーダー・フォロワー追従」仕様書を要求仕様として扱います。

今回実装するのは、ベースで把持位置まで接近した後、両ベースを停止・位置保持し、
リーダーのアームのみで物体を短距離並進させ、フォロワーのアームのみが
把持部の外力からアドミッタンス＋5関節アームIKで追従するモードです。
追従中のベース移動は許可しません。ベースP追従やWhole-bodyのベース成分は無効化してください。

作業の順序：
1. 既存のlaunch・2台のnamespace・TF・アーム制御・台車制御・把持・センサ実装を調査し、実在するトピックとコード位置を報告。
2. 接近→停止→両把持→静止・バイアス取得→アームのみ追従→停止のFSMを実装。
3. リーダーは時間軌道でアームだけを動かす。フォロワーは自身の力覚のみから目標を生成する。
4. アームのみのDLS IKを用い、両台車への移動指令を止めるガードを設ける。
5. 位置・速度・力の制限、台車ドリフト、把持喪失、センサ途絶を監視。
6. 単体試験とGazebo統合試験を実行し、両台車停止・両アーム動作・物体並進のログを保存。
7. 変更ファイル、起動コマンド、パラメータ、実行結果、未解決課題をREADMEに記す。

未実装の力センサや存在しないtopicを仮定した実装は禁止です。
力センサがない場合はテストモードまで完成させ、本実験の未実装部分を明示してください。
既存の他モードを破壊せず、コード改変は必要最小限に留めてください。
```

---

## 14. 今回の完成イメージを一文で

**「2台が台車で物体へ接近して把持し、両台車がその場に停止したまま、リーダーアームが物体を動かし、フォロワーアームが力覚アドミッタンスで追従する」ことを実現・検証する。**

その後の研究段階で、アームの関節限界・可操作度・把持力などを指標として**台車追従を追加**するが、それは**別仕様書**で扱う。
