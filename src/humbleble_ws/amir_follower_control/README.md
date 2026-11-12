# AMIR local wrist follower (Humble / Fortress)

既存の直進・指定軸回転・把持を保全する追加パッケージ。実験結果と未完のゲートはworkspaceの `GAZEBO_IMPLEMENTATION_REPORT.md` を参照。未検証のまま自動で軸を増やさない。

既定Q_HOMEはrank 3、8 kg固定把持はraw手首モーメント約14.64 Nmで安全制限を超える。追加の傾斜0.2 rad姿勢はrank 4。0.5 kg摩擦把持でFx/Fy/Fzの±0.5 N、τzの±0.1 Nmを順に実測した（各2秒＋1秒整定、試験間pause）。4軸とも方向・把持維持・安全条件を確認済み。この短時間の単軸試験と、4軸の連続搬送・台車回復の実証範囲は区別する。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  amir_follower_gazebo amir_follower_control coop_transport_bt
source install/setup.bash
ros2 launch amir_follower_control coop_transport.launch.py headless:=true \
  evidence_directory:=/tmp/amir_follower_evidence
ros2 run amir_follower_control local_evidence --seconds 10 \
  --output /tmp/amir_follower_evidence/sensors.jsonl
```

センサは既存Joint_5、既存bridgeのみ。専用worldにFortress ForceTorque systemを加え、controllerのactiveを待ってからhome・把持へ進む。Followerは初期DISABLED、指令なし。追加worldの把持adapterが実DetachableJoint constraintを10 sim Hzで照合するため、単独の `follower.launch.py` は把持後でも開始できる。自機heartbeatが0.35 sim s途絶えると停止する。

Followerの入力は `/amir2/ft_sensor`, `/amir2/joint_states`, `/amir2/mecanum_drive_controller/odometry`, `/amir2/grasp/state`, `/clock` のみ。`/amir2/odom` はGTなので禁止。ローカルのB/S/G変換はF/T stampを挟む関節サンプルの補間とURDF FKから得る（外挿なし）。baseは **base_footprint** （base_linkとの固定yaw=-π/2もFKで計算）、Gはモデルのtcp_link。CはBに追従する設定。台車移動中に固定校正biasの重力成分が変わる限界を考慮すること。

指令はアームJointTrajectoryと既存relayへの `/amir2/rover_twist` (Twist)。relayから既存mecanum controllerのreference_unstampedへ送る。READYでも運転権を取得するまで発行しない。approach/home/旧Follower/coordinatorとの同時発行は禁止。supervisor側で初期把持・支持台除去を確認し、既存approachへ `/cooperative_transport/active=true` を渡してhandoffする。新Followerはこの共有topicを購読しない。

## 段階ゲート

物理試験の証跡付きJSONを `calibration_file` に設定してから校正・enableする。雛形の数値を実測済みとして埋めない。

```json
{
  "robot": "amir2",
  "urdf_sha256": "展開したrobot_descriptionのsha256",
  "sensor_link": "gripper_base_1",
  "header_frame": "実測header.frame_id",
  "sign": 0,
  "phase0_passed": false,
  "time_alignment_passed": false,
  "verified_axes": [],
  "phase2_axes_passed": [],
  "base_axes_verified": false,
  "evidence_files": []
}
```

`sensor_sign` は既知方向の外力で確認した+1または-1、header_frameは実出力の文字列を設定。現モデルの無把持較正は+1、`amir2/Joint_5/wrist_ft_sensor`と確認済み。sign=0では駆動不可。まずaxes_enabled=[true,false,false,false]のFx。`verified_axes` はセンサの物理較正済み軸、`phase2_axes_passed` は負荷把持アドミッタンスの実証済み軸。FyにはFx、FzにはFx/Fy、tzにはFx/Fy/FzのGate 2通過が必要。モードA=位置保持、B=アームのみ、C=アーム＋低速回復。Cはbase_axes_verified、4軸すべてのphase2_axes_passed、recenter_enabledが必要。

```bash
ros2 service call /amir2/follower/calibrate std_srvs/srv/Trigger '{}'
ros2 topic echo /amir2/follower/status
ros2 service call /amir2/follower/enable std_srvs/srv/SetBool '{data: true}'
# 停止（アーム保持・グリッパは開かない）
ros2 service call /amir2/follower/enable std_srvs/srv/SetBool '{data: false}'
```

較正は静止把持でdistinct F/Tサンプルのみ蓄積。時刻・skew・古い状態・NaN・raw全6軸過大値・関節上限・controllerの型とactive・指令追従誤差を監視。FAULTは明示reset後に再較正が必要。pause中は演算しない。DLSは空間ωzを4行目とし、一般姿勢のEuler yaw微分と混同しない。並進残差[m/s]と回転残差[rad/s]は分離監視。

追加worldでは標準イベントに加えて実constraintの周期確認を用いる。`controlled_grasp:=true` は初期未接続から検証済みAPIで固定constraintを管理する。これは摩擦把持の性能実証ではない。`gentle_grasp:=true` は指荷重のLPFとraw peak監視で両側接触を確認し、確認後に支持台を撤去する。手首F/Tはbias差引前の全6軸と、URDF軸へ射影したセンサjointの伝達モーメントを監視する。JointState effortは実構成でNaNのため、他関節のトルク検証済みとは扱わない。

```bash
PYTHONPATH=src/humbleble_ws/amir_follower_control:$PYTHONPATH \
  python3 -m pytest -q src/humbleble_ws/amir_follower_control/test
```

ログJSONLは実シミュレーション時刻、raw wrench、G/B変換値、静止bias、誤差、admittance変位・速度、q/dq/q_des、base command/encoder odom、singular values、配分残差、joint margin、state age、ローカル時計からのRTF概算、FAULTを保存。Gazebo statsのRTFとworld真値は評価プロセス側で保存し、Followerプロセスへ渡さない。現時点の実証範囲を越えて成功率や力低減を主張しない。

既知外力の再較正は `with_payload:=false external_wrench_test:=true` でsceneを起動し、home完了後に `experiments/excite_wrist.py --output ...` を実行する。Fx/Fy/Fzの±0.5 N、τzの±0.05 Nmを各1.5 sim s、別プロセスから加える。`analyze_evidence.py` で生ログから較正CSV/グラフを生成できる。APT matplotlibを使う場合は環境で確認したABI不整合を避けるため `PYTHONNOUSERSITE=1` を指定する。ROS/Gazeboの並行試験は明示したROS_DOMAIN_IDとIGN_PARTITIONをCLI側にも揃える。


## 続行作業の再現プロファイル

`config/tilted_0p5kg_staged.yaml` は接触剛性を考慮してM=[50,50,50,20]、D=[1500,1500,1500,400]へ調整した段階試験用。既定の雛形ゲインでは把持付きFxが過大力停止した。元の8 kg物体・SDFは変更せず、派生SDFだけで質量と慣性を同倍率へ変更する。

```bash
ros2 launch amir_follower_control coop_transport.launch.py \
  headless:=true gentle_grasp:=true posture_pitch_rad:=0.2 \
  contact_compliance:=true controlled_grasp:=true payload_mass_kg:=0.5 \
  external_wrench_test:=true evaluation_logging:=true \
  config:=/absolute/path/to/certified_fx.yaml \
  evidence_directory:=/absolute/path/to/evidence
```

較正済み設定には、その実行で生成したamir2.urdfのhashに一致する証跡JSONを指定する。既存証跡のパスを実験なしで別モデルへ移さない。`run_local_trial.py` は初期化・把持・支持台撤去後にhandoffし、較正と運転を実行する。`--stimulus leader --delay 0.3 --speed 0.00025 --acceleration 0.001 --duration 4 --settle-seconds 3` は低速Fxの再現条件。失敗もsummary.jsonへ記録する。`analyze_control.py` と `analyze_trial.py` が別プロセスの実ログをCSV・PNG・単位別指標へ変換する。

連続搬送ではadmittanceのオフセット上限に達するため、現段階は短距離実験に限定する。base回復、全軸搬送、A/B/C比較、長時間安定性の完了は実測レポートで判断する。


`grasp_mode:=friction gentle_grasp:=true controlled_grasp:=false` は既存の0.5 kg接触・摩擦モデルを使う追加構成。固定constraintを生成せず、各ロボットの指接触・指F/Tに基づいて自機stateを周期確認する。相手側の接触が失われても、自機stateの判定条件には入らない。グリッパ初期化の両側完了だけは上位supervisorが確認する。把持方式を変えた場合はFxのGate 2から再検証する。


最新の単軸合格条件は、摩擦版の物体生成→接近という初期化順序、挿入25 mm、初期接触目標6 N、把持後の局所保持目標5 N、holding gain=.00015、max step=.0003 rad/.1秒。旧版のFy失敗や接近timeoutは証跡に残し、合格へ書き換えていない。`defer_grasp:=true` でhome後の無把持較正を実施できる。較正後のTrigger `/experiment/start_grasp` はstartup専用。較正・把持方式・挿入量を変えた場合は新しいURDFと実測証跡を使う。

## 2026-10-17の回復試験

0.5 kg摩擦版でCモードのFx/Fy/yaw回復を実測。Leader先行0.3秒（encoderからの実測差.296秒）のFxでは腕水平変位1.135→.227 mm、Fy=1 Nでは1.256→.242 mm、τz=.1 Nmでは相対yaw1.287→.031 mradへ回復した。yaw試験だけ回復閾値を.4/.16 mm、他は1/.4 mmとした。台車G点速度の差引きも実指令から検証済み。ただし接触滑りを含むTCP相対位置変化があり、無滑り長距離搬送や速度.02/.05/.1 m/sは未実証。8 kg摩擦版も支持台撤去後の指荷重条件を満たさず、追従前に停止した。

このworkspaceの実測証跡を使う再現例（初期状態はDISABLED）:

```bash
source install/setup.bash
RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS ROS_DOMAIN_ID=82 \
IGN_PARTITION=amir_local_demo ros2 launch amir_follower_control coop_transport.launch.py \
  headless:=true grasp_mode:=friction gentle_grasp:=true \
  contact_force_n:=6.0 holding_force_n:=5.0 \
  holding_position_gain:=0.00015 holding_max_step_rad:=0.0003 holding_interval_s:=0.1 \
  grasp_inset_m:=0.025 posture_pitch_rad:=0.2 contact_compliance:=true \
  payload_mass_kg:=0.5 external_wrench_test:=true evaluation_logging:=true \
  config:="$(pwd)/gazebo_evidence/20261016/continue/friction_hold_fast/c.yaml" \
  evidence_directory:=/tmp/amir_local_demo
# 別端末: 同じinstall、ROS_DOMAIN_ID、IGN_PARTITION、RMW設定を使用
RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS ROS_DOMAIN_ID=82 \
IGN_PARTITION=amir_local_demo python3 \
  src/humbleble_ws/amir_follower_control/experiments/run_local_trial.py \
  --output-directory /tmp/amir_local_demo/leader_delay03 --stimulus leader \
  --delay .3 --speed .001 --acceleration .001 --duration 6 --settle-seconds 10 \
  --wait-startup-s 360
```

BTは最初のsim時計受信を待ち、service発見→enable→状態監視→disable応答とDISABLED確認を行う。静止1秒の実行を検証済み。実験supervisorは較正前に自機関節/encoderの連続した静止区間を待つが、較正中の動きや荷重変化は引き続きFollowerが拒否する。

## 横方向把持・短リーチの追加設定

`pinch_orientation:=lateral` は傾斜・摩擦scene専用。Joint_5=0、物体roll=0にし、断面に合わせて支持台高さも計算する。既定verticalは元のまま。`arm_reach_m:=0.40` はG点高さと物体軸を保ちながら腕を短く使う追加設定で、0が元のリーチ約.475 m。`contact_cfm` は既定.05、試験範囲1e−4〜.1。これらを変更した際の荷重合格は、それぞれの実測条件に限定する。

横方向0.5 kg・元リーチ・CFM=.05ではFx/Fy/Fz正負、τz正負とCモードのFx Leader先行.3秒を検証した。τzは2秒加力＋3秒整定。他の3軸は2秒＋1秒。加力中の応答と除荷後の戻りを区別して解析する。C試験の水平変位は1.059→.243 mmへ回復したが、物体高さも2.513 mm低下しており、無滑り連続搬送とは扱わない。

横方向8 kg・元リーチでは指の初期過大荷重を回避したが、Joint_2のモデル荷重が22 Nmを超え、追従不良で停止した。短リーチ.40 mではFx正負/Fy正方向まで成立したものの、把持の瞬間荷重・滑りで次段階を停止。CFMと保持応答を変えた条件は再びFxから検証する。旧姿勢や異なる荷重条件のcertificateを流用して全身運転しない。

追加安全監視 `quasistatic_joint_load_Nm` は、測定手首wrenchにURDF上流リンク/固定取り付け質量の重力項を加えるモデル推定。下流のグリッパ・物体を二重加算せず、URDF effort超過は較正前から拒否する。監査済み平面台車・gravity_B=[0,0,-9.81]が前提で、上流慣性・摩擦や実モータートルクの計測ではない。指8 Nも暫定試験判定であり実機定格ではない。

実験supervisorだけが両機の局所grasp heartbeatを監視し、欠測・古さ・detachedで開始を待つ/試験を止める。Follower自身の購読は従来の自機情報だけ。最新の合否、設定、起動失敗を含む証跡はimplementation report末尾を参照。

摩擦保持の `holding_interval_s` は.02〜.2秒（既定.1秒）。独立timerと共通の間隔gateでstartup側timerとの二重補正を防ぎ、`holding_max_step_rad / holding_interval_s <= .004 rad/s` を強制する。関連値を運転中に変える場合はatomic parameter setで一括検証する。20 ms試験はgain=.000012、max step=.00004 radを使用し、常時50 Hzのaction送信を保証するものではない。自機heartbeatは直近.3秒の自機raw指荷重ピークも監視するため、一瞬の8 N超過後に値が戻っても停止を見逃さない。

8 kg/.40 m/CFM=.01の20 ms保持ではFx正負のみ通過し、Fy正方向は指raw約8.8 Nで停止した。`experiments/analyze_task_coupling.py` は実URDF・自機関節ログから、4Dタスクに伴う未指定ωx/ωyをオフライン診断する。今回の姿勢ではFyにωxが約12.286 rad/m結合し、実測の約41 µmのy変位と約.485 mradのωx積分が整合した。rank 4だけで把持姿勢の拘束を満たすとはせず、8 kgの残る軸・全身搬送は未合格。

## GUIで短い直進を確認する

workspace直下の端末で起動する。このworkspace内の検証済み証跡を参照するデモ。

```bash
source install/setup.bash
python3 src/humbleble_ws/amir_follower_control/experiments/gui_straight_demo.py
```

0.5 kg横方向摩擦把持・Cモードで、初期姿勢→接近→把持→支持台撤去→静止較正/保持を確認し、GUIをpauseして待機する。端末の **Enter** で最大1 mm/s・加速度.001 m/s²・直進6秒（Leader先行.3秒）＋姿勢回復10秒を実行し、再びpauseする。**q** またはGUI終了でそのセッションを閉じる。移動量は数mmの短距離確認であり、通常速度や長距離搬送の実証ではない。FAULT後の自動resetはしない。

GUI用のカメラ設定は各証跡ディレクトリへ生成し、ユーザーの既定GUI設定を上書きしない。launch追加引数 `gui_config` は既定空。セッションごとにIGN_PARTITIONと証跡を分離し、既定ROS_DOMAIN_ID=90（変更は `--domain 91` など）。同時に別sceneを同じROS domainで動かさない。最新の証跡パスは `gazebo_evidence/gui_straight_latest.txt`、起動コマンドとPIDは `session.json`、初期確認は `ready.json`、各直進は `straight_001/` 以降に保存する。試験後のGT評価やグラフ生成は別プロセスで行う。
