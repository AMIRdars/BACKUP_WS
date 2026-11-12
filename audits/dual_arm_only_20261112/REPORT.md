# 接近後・両台車停止・アームのみ協調追従 実装報告

2026-11-12（Asia/Tokyo）。対象仕様書は `request_spec.md` に保存。最終実行は `verified_gui/trial/`。

## 結果と適用範囲

新しい既定モード `ARM_ONLY / dual_arm` を実装した。把持前のリーダーアーム単体確認後、両台車が車輪で接近し、停止を確認してから既存の摩擦接触把持を行う。リーダーのアームが時間軌道で物体を移動し、フォロワーは自己F/TだけからK=0の速度型アドミッタンス＋5関節DLSで追従する。世界固定治具と位置設定サービスは使用しない。

最終実測判定は **71/71項目合格**。リーダー手先は 29.999020 mm、物体は 29.821484 mm、フォロワー手先は 29.640707 mm移動。両指の接触・把持力帯を維持し、安全停止・把持保持を確認した。停止後の物理を一時停止してGUIを残している。

台車への指令ゼロと物理的な完全固定は同じではない。車輪odomはほぼゼロだが、把持時のなじみを含む物理位置変化は最大約1.11mmある。仕様1.2節の暫定10mm基準で評価し、ARM_ONLY開始時にラッチする既存1mm停止監視は変更していない。

| 機体 | 台車ローカル手先Δx [mm] | ARM_ONLY中GT最大ドリフト [mm] | BASE_STOP以降GT最大ドリフト [mm] | ARM_ONLY中odom最大速度成分 [m/s,rad/s] |
|---|---:|---:|---:|---:|
| leader | 29.999020 | 0.178256 | 1.105227 | 2.081e-12 |
| follower | -29.640707 | 0.179073 | 1.105991 | 2.703e-12 |

F/T増分ピーク 0.270983 N、フォロワー手先追従誤差最大 18.831 µm、物体姿勢変化最大 1.154893e-05 rad。制御dtはシミュレーション時刻で8〜12ms。world TCP変位と物体変位の差は両側とも最大約0.060mmで、既存2mm境界以内。これらはこの0.5kg物体・30mm条件のシミュレーション結果である。

## 既存調査と原因

Humble、Gazebo Fortress 6.16.0、既存JointTrajectoryController 2.53.1ワークスペース修正版を使用。今回ROS/Gazebo/URDF/controller YAML/ゲインを変更していない。既存修正版JTCの実ロード証拠は `controller_isolation.json`、参照混入検査は `jtc_isolation_audit.json`。

以前の既定 `CompensatingLeader` はリーダー台車の自己odom移動量からアームの半分の引き込み量を決めるため、台車走行を前提にしていた。従来 `ArmBase` は実測アーム偏差を開始閾値なしで再中心化へ渡していた。今回この両経路は起動せず、リーダーの時間ベースアーム軌道とStrictArmOnlyを使う。Nav2は今回のgraphにない。台車／アームの最終指令先は各一つで、新しい並行writerを残さない。

| 項目 | 実在確認済みの名称・実装 |
|---|---|
| namespace / TF | `/leader`, `/follower`。prefix `leader/`, `follower/`。実TF48フレームを `runtime_tf_audit.json` に保存 |
| 台車基準 / TCP | `<robot>/base_footprint`、`<robot>/base_link`、`<robot>/tcp_link` |
| アーム | `Joint_1`〜`Joint_5`、順序固定、全てrevolute。位置・速度・effort限界は実URDFから取得 |
| FK / J / IK | `wrench.py::LocalModel.frames/jacobian`、`arm_only_dls.py::xy_velocity_dls`、`stroke_kinematics.py::solve_stroke_pose` |
| 位置範囲 [rad] | J1[-2.96706,2.96706], J2[0,2.356194], J3[-2.792527,0], J4[-2.094395,1.308997], J5[-2.75762,2.75762] |
| URDF速度 / 実験速度 | [1,1,1,1,2]rad/s / 5関節とも既存0.03rad/s上限。0.15rad/s²は既存全身allocatorの実験値を流用し実機定格とは扱わない |
| 関節状態 | `/<robot>/joint_states`。controller_manager 250Hz、JTC state 100Hz、位置指令API |
| アーム要求 / 最終先 | `/<robot>/arm_joint_cmd` → 既存gate/interlock → `/<robot>/arm_controller/joint_trajectory`。型JointTrajectory、最終writerはactuator_interlock |
| 台車要求 / 最終先 | `/<robot>/base_cmd` (TwistStamped) → gate/interlock → `/<robot>/rover_twist` (Twist) → rover_twist_relay_ign → `/<robot>/mecanum_drive_controller/reference_unstamped` |
| wheel controller / odom | mecanum_drive_controller、車輪velocity API、`/<robot>/mecanum_drive_controller/odometry`。車輪半径0.076m、幾何パラメータ0.24mは既存YAML継承 |
| F/T | `/<robot>/ft_sensor` (WrenchStamped)、Joint_5のwrist_ft_sensor、100Hz、frame child / child_to_parent |
| フォロワー処理 | `/follower/wrench_at_grasp_raw`, `/follower/wrench_filtered`, `/follower/wrench_status`。実際の型・購読先一覧はgraph.json |
| F/T校正 | 既存6軸±入力検証、sign=+1、header=follower/Joint_5/wrist_ft_sensor。センサ極性検証済み、許容荷重校正は未完了 |
| 把持API | `/<robot>/gripper_controller/gripper_cmd` action、既存grasp/prepare・close・takeover等。左右contact＋finger wrenchを確認 |
| 物体 | cooperative_payload、既存box 1.20×0.03×0.08m、0.5kg。物体と両グリッパは摩擦接触で、固定ジョイント把持ではない |
| 時刻 / 評価GT | `/clock`、既存ROS/Gazebo bridge。制御ノードに独自step処理なし。world poseは準備・安全評価・描画専用 |

FK/Jの既存実モデル有限差分誤差は5.41e-11 m/rad以下。監査数値と証拠は `audit.json`、`verified_gui/arm_plan.json`、実controller/services/購読/最終writerは `verified_gui/trial/graph.json`。

## 数式・座標と状態遷移

B_L/B_Fを各base_footprint軸、TCPを手先位置とする。対向配置なので世界+Xへの並進はL側ローカル+x、F側ローカル-xになる。リーダーだけに相対x目標を与える。フォロワーの購読境界は自分のjoint_states・odom・F/T整形値・局所把持状態・共有安全信号に限定し、leader位置・速度・目標／world GTを購読しないことをgraphで検証した。

F_B=sign R_BS F_S、tau_TCP=sign R_BS tau_S+(p_S-p_TCP)×F_B。既存preprocessorがFKをF/T時刻に補間し、100サンプル静止荷重tare、8Hz低域フィルタを行う。grasp_alignedはTCP参照点で軸はBと平行。今回の静的基準除去は姿勢依存の完全な重力モデル補償とは区別する。

リーダーはs(u)=10u³−15u⁴+6u⁵、p_ref=p_home+[distance*s,0,0]。T=max(1.875distance/vmax, sqrt((10/√3)distance/amax))、amax=0.0012m/s²。両腕の実URDF経路を61点ずつ事前検査し、5自由度で到達できる姿勢枝だけを使う。既存位置サーボ実測応答による0.04秒補償と80msの位置軌道APIを継承する。

フォロワーはM v_dot+D v=F_err、M=2kg、D=34.6Ns/m、K=0。3mN連続死域は先行静止試験206サンプルのXY最大2.87245mNに基づく暫定値。p_ref=p_home+integral(v_adm)、e_track=p_ref−p_actual、e_arm=p_actual−p_homeを区別し、homeはenable時に一度だけラッチする。v_cmd=v_adm+4e_track、qdot=J_xy^T(J_xy J_xy^T+lambda²I)^−1 v_cmd。z/姿勢は既存nullspace副次制御。速度・位置飽和、関節余裕、特異点、F/T欠測、両台車ドリフトは保持停止へ進む。

実測FSM: INIT (0.000s) → BASE_APPROACH (40.696s) → BASE_STOP (56.600s) → DUAL_GRASP (58.188s) → SETTLE_AND_BIAS (80.284s) → ARM_ONLY (94.760s) → HOLD (111.392s)。

BASE_APPROACHだけ一時的なrover_twist writerを持ち、最大0.015m/s・0.005m/s²で既存の±1.064987m配置へ車輪走行する。両側停止を0.5秒確認し、ゼロ送信後にpublisherを破棄する。位置サービス呼び出しは0回。DUAL_GRASP以降は両機のgateとinterlockをbase_zero_onlyにし、非ゼロを拒否する。ARM_ONLY中のドリフト異常で台車追従へ移らず共有停止する。グリッパを自動で開かない。

## 実行手順

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select amir_compliance_control cooperative_transport_bringup
source install/setup.bash

ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true \
  forward_distance_m:=1.0 \
  translation_velocity_m_s:=0.1 record_data:=true
```

上の旧走行引数は新モードの台車走行目標に流用しない。既定は `arm_only_experiment.enabled:=true arm_only_trial:=dual_arm dual_arm_distance_m:=0.03 dual_arm_speed_m_s:=0.005`。初期試験は30mm以下・5mm/s以下。小試験は `dual_arm_distance_m:=0.005`。終了時には停止・把持確認後 `gui_review_mode:=paused` でGUIを保持する。GUIを閉じるかCtrl+Cで所有プロセスだけ終了する。

以前の分岐は `arm_only_trial:=native_stroke` / `arm_only_trial:=micro`、既存WHOLE_BODYは `control_mode:=WHOLE_BODY` で残す。今回その全身Gazebo動作は再認定していない。詳細手順は `src/humbleble_ws/cooperative_transport_bringup/README_dual_arm_only.md`。

評価: `PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/amir_dual_arm_evidence.py --case audits/dual_arm_only_20261112/verified_gui/trial --held`。GUI終了後は --held を外す。動画は `tools/amir_gazebo_window_video.py --case <case>/trial --output <新規AVI> --include-preparation --fps 2 --duration-s 360`。

## 動作試験・失敗の記録

| 試験 | 結果 |
|---|---|
| 単体・ROS回帰 | `regression.log`:61 passed、`dual_tests.log`:31 passed、`final_tests.log`:78 passed。重複を含むため合算しない |
| K=0新設定＋F/T欠測・共有停止 | `velocity_mode_tests.log`:7 passed。力解除で速度が減衰し変位が残る、死域の極性、両機非ゼロ台車指令拒否を確認 |
| 把持前リーダー手先単体 | 9.999987mm実測、homeへ復帰。把持なしでのアーム/controller試験である |
| 車輪接近 | 実走行約110mm、停止を確認して把持へ移行。テレポートなし |
| micro 5mm | Lアーム4.999mm、Fアーム4.843mm、物体4.876mm。初版評価が接近中の正当な非ゼロ指令を含めて失敗。失敗resultを保存し、BASE_STOP以降へ評価区間を修正 |
| translation_030_gui | 30mm協調動作・停止・把持合格、動画dual_arm_030.avi。最終版に追加した単体probe前の実行として保存 |
| final_gui | 単体probeの計算後時刻・観測順序でfreshness拒否。把持前に終了。新鮮な観測から軌道epochを確定し、50ms source/150ms wall条件で整合サンプルを待つよう修正 |
| release_gui | 単体probeの移動と復帰後、NumPy boolのJSON保存に失敗。Python boolへ正規化して修正。失敗ログを保持 |
| verified_gui | 最終版の単体probe、接近、両把持、30mm追従、停止・把持保持、独立判定71項目が合格 |

最初の独立評価ではBASE_STOP以降のドリフトへARM_ONLY用1mm境界を誤適用した。元JSONは `dual_arm_evidence_initial_stop_scope.json` に保存。仕様のBASE_STOP基準10mmと、実装のARM_ONLY開始基準1mmを別項目へ修正した。実測値・制御停止閾値は変更していない。

試験で使用した制御ソース79ファイルは現在のファイルとSHA一致（`tested_source_consistency.json`）。既存のtracked差分は作業開始時と同一（`tracked_preservation.json`）。既存修正版JTCの参照混入0件も記録した。

## 成果物

- `verified_gui/trial/held_result.json`:GUI保持時点の実行結果・両機停止・グリッパactive証拠。GUIを閉じるとresult.jsonが確定する。
- `verified_gui/trial/dual_arm_evidence.json`:71項目の個別合否、指令ゼロ、実GTドリフト、FK、接触、誤差。
- `verified_gui/trial/timeseries.csv`:全時系列。両台車cmd/odom/GT、両腕q/qdot/FK/目標、F/T、物体姿勢、接触、停止理由、元センサ時刻。
- `verified_gui/trial/dual_arm_timeseries.csv`:共有記録時刻の抜粋。元CSVは保持し、source/receipt時刻を同一視しない。
- `verified_gui/trial/wrench_samples.csv`, `control_samples.csv`, `events.jsonl`:元時刻を保持したF/T・DLS・最終指令の詳細。
- `verified_gui/trial/dual_arm_measured.png` / `.pdf`:両台車GT/指令/odom、両手先実測・目標、F/T＋アドミッタンス速度、物体・両TCP、全関節角・速度・余裕。
- `verified_gui/trial/dual_arm_fsm.png`, `experiment_fsm.json`:実測状態遷移。
- `dual_arm_verified.avi` / `.json`:この試験のGazeboウィンドウのみの動画とwall/simフレーム時刻。実時間動画であり、物理が実時間速度という主張はしない。
- `gui_final.png`, `gui_final_proof.json`:所有ウィンドウと一時停止状態の確認。
- `changed_files.json`, `changes.patch`, `source_snapshot/`:変更内容とソース保存。baseline未採取の既存untrackedファイルはpatch内で全体表示になるため、同JSONで区別する。

## 変更ファイル

| ファイル | 理由・役割 |
|---|---|
| `src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py` | 既定dual_armモード・専用有効フラグ・30mm/5mm毎秒の実験引数 |
| `src/humbleble_ws/amir_compliance_control/launch/arm_only_af.launch.py` | リーダーをアーム時間軌道ノードへ差し替え、フォロワーAPIを継承 |
| `src/humbleble_ws/amir_compliance_control/launch/guarded_safety.launch.py` | 両機のgate/interlockをbase_zero_onlyで固定、writer監査 |
| `src/humbleble_ws/amir_compliance_control/amir_compliance_control/transport.py` | 既存台車軌道を既定で継承し、アームモードでは無効化する入口 |
| `src/humbleble_ws/amir_compliance_control/amir_compliance_control/admittance.py` | 剛性の半正定値K>=0を許容、既存K>0動作を維持 |
| `src/humbleble_ws/amir_compliance_control/amir_compliance_control/strict_arm_only.py` | K=0利用、実測静止ログに基づく死域、既存実験値による関節速度ランプ |
| `src/humbleble_ws/amir_compliance_control/setup.py` | 既存packageにleader_arm_only実行入口を追加 |
| `tools/amir_af_trial.py` | 既存の計測・把持・停止手順にdual_arm分岐、FSMと両台車GTを記録 |
| `tools/cp9_transport_preparation.py` | 新モードだけ物理接近へ分岐、従来の準備・把持APIを流用 |
| `src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_only_leader.py` | 局所状態から五次補間アーム軌道、両下流ゼロロック確認、ドリフト・関節監視 |
| `src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_translation.py` | 速度・加速度を満たす五次補間関数 |
| `src/humbleble_ws/cooperative_transport_bringup/config/compliance_dual_arm_gui.config` | 両アームが見やすい近接側面視点（表示のみ） |
| `src/humbleble_ws/cooperative_transport_bringup/README_dual_arm_only.md` | 実行・停止・評価・既存モードの手順 |
| `tools/dual_arm_only_runtime.py` | 公開launchの実行方針、両腕IK経路の事前検査、実験プロセス管理 |
| `tools/dual_arm_preparation.py` | 接近専用の一時writer、車輪接近、停止確認後にwriter破棄 |
| `tools/dual_arm_single_probe.py` | 把持前10mmのリーダー手先単体移動・復帰・実測判定 |
| `tools/amir_dual_arm_evidence.py` | 指令・実測FK・odom・GT・接触から独立に合否判定、CSV/グラフ |
| `tools/test_dual_arm_only.py` | 新モードと既存経路の回帰・失敗時停止テスト |
| `tools/amir_p0_evidence.py` | 両台車停止の評価経路と接近区間の分離 |
| `tools/amir_gazebo_window_video.py` | 試験所有のGazeboウィンドウのみ動画保存 |
| `tools/test_native_arm_stroke.py` | 新モードと既存経路の回帰・失敗時停止テスト |
| `src/humbleble_ws/amir_compliance_control/test/test_strict_arm_only.py` | 新モードと既存経路の回帰・失敗時停止テスト |

## 残課題・未実施

両台車は車輪ゼロ速度保持であり、物理的ドリフトを完全に0へは抑えていない。把持時の約1.11mmのなじみと、追従中の約0.18mmを区別して記録した。より厳密な保持が必要な場合は既存の車輪保持性能・接地摩擦・荷重を検証する。世界固定治具で結果を置き換えてはいない。

Step3の「フォロワー非把持で、リーダー単独把持により物体を動かす」専用実験は未実施。実施したのは把持なし10mmアーム単体と、両把持30mm協調試験であり、単腕での物体保持成功と混同しない。Step6の往復、50mm拡張、速度拡張も未実施。台車再追従・回転は今回有効化していない。

力・トルク停止境界は既存native実験の50N/100N/15Nm、指の把持監視8〜26N等を継承した。実機の校正済み許容荷重ではない。完全な姿勢依存重力補償、ノイズ死域の条件別再校正、長時間K=0保持、故障注入を含む全条件のGazebo再認定、反復成功率は未完了。F/T欠測のROS結合停止は新K=0設定で通過したが、今回の30mm摩擦把持の全故障行列を実行したわけではない。

終了後GUIは物理一時停止による結果確認用であり、長時間の動的把持耐久試験ではない。有限条件の把持維持を確認した結果として扱う。
