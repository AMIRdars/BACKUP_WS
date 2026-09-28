# 周期一覧と変更候補

| 対象 | 周期（sim 時間基準） | 判定 |
|---|---:|---|
| DART physics | 4 ms / 250 Hz（候補） | 3 ms と 4 ms を別測定、最終完走で採否判定 |
| controller_manager / 車輪・アーム・指 | 250 Hz | 維持。接触・低レベル制御に必要 |
| arm_controller 状態 | 100 Hz | 維持 |
| arm action 状態監視 | 20 Hz | 維持 |
| finger F/T | 100 Hz、4 センサ | 維持。把持力制御入力 |
| contact sensor | 30 Hz、36 センサ | 維持。物理接触計算は別 |
| C++ contact 集約 | 30 Hz、4 指状態 | candidate；力入力は集約しない |
| 真値 odometry | 50 Hz / robot | 維持 |
| coordinator | 50 Hz | 経路追従を維持 |
| rotation_controller | 100 Hz | 軌道生成を維持 |
| pivot_rotation_controller | 50 Hz | 指定軸回転を維持 |
| friction_grasp_manager | 10 Hz | 把持力制御を維持 |
| dual_base_approach | 30 Hz | 接近制御を維持 |
| arm_home_positioner | 25 Hz | 初期化時のみ |
| safety_monitor | 50 Hz | 緊急停止応答を維持 |
| slip_monitor の判定 / 状態通知 | 50 Hz | 30 Hz を単独評価する候補 |
| slip_monitor の実測 Pose / TF | pose 入力 callback ごと | 判定タイマとは独立。位置・姿勢入力頻度は下げない |
| rover_twist_relay | command callback ごと | 入力と同周期 |
| world pose/info | SceneBroadcaster のイベント | 設定した制御周期と同一とは限らない。必要 3 モデルへ内容を削減 |

設定根拠：`friction_transport.yaml`、`rotation_params.yaml`、各ノードの `create_timer`、`controllers.yaml`、`arm_controllers.yaml`、`amir_mecanum3_sim.xacro`、`mecanum3.gazebo`。slip 判定を 50→30 Hz にすると追加の検知待ちは最大約 13.3 sim ms。低レベル制御・F/T・安全監視は変更しない。
