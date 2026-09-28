# 次段階：Gazebo 接触通知の直接集約

変更前 checkpoint: 5826561。手順書の項目5（contact 通知集約）を追加検証する。

現在は36 Gazebo Contacts→36 ROS Contacts→C++ OR→4 Bool。候補は36 Gazebo Contacts→C++ OR→4 Boolで、raw ROS変換/配信を省く。接触センサ36個、9 box/指、4 ms physics、F/T 100 Hz、集約30 Hz、期限0.25 sim秒、把持力制御は維持する。Gazebo raw topic は引き続き取得できる。

## 比較手順

1. 現在の採用構成を headless 180 wall秒測定。
2. contact_input_transport:=gazebo のみを変更して同じ条件で測定。
3. 接続テストで OR・解除・期限・payload 限定・欠測深さを確認。
4. 品質と性能が成立すれば全工程を反復し、採用/保留/差し戻しを判断。

条件ごとに condition.json、world.sdf、samples.jsonl、quality.json、CPU、Update、ROS一覧、launchログを保存。比較は support_removed 後の区間を使用する。全工程と180秒試験は区間が異なるため区別する。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 simulation_performance/optimization/summarize.py simulation_performance/next_stage
ROS_DOMAIN_ID=116 IGN_PARTITION=contact-native-test-isolated /usr/bin/python3 simulation_performance/next_stage/test_native_contact.py
```

原則：最終判断まで既定は ros。aggregate_contacts:=false は従来の生 ROS Contacts を維持。物理拘束計算の削減とは扱わない。RTF=1.0 の達成を保証しない。
