# 次段階：Gazebo 接触通知の直接集約

変更前 checkpoint: 5826561。手順書の項目5（contact 通知集約）を追加検証する。

現在は36 Gazebo Contacts→36 ROS Contacts→C++ OR→4 Bool。候補は36 Gazebo Contacts→C++ OR→4 Boolで、raw ROS変換/配信を省く。接触センサ36個、9 box/指、4 ms physics、F/T 100 Hz、集約30 Hz、期限0.25 sim秒、把持力制御は維持する。Gazebo raw topic は引き続き取得できる。

## 比較手順

1. 現在の採用構成を headless 180 wall秒測定。
2. contact_input_transport:=gazebo のみを変更して同じ条件で測定。
3. 接続テストで OR・解除・期限・payload 限定・欠測深さを確認。
4. 品質と性能が成立すれば全工程を反復し、採用/保留/差し戻しを判断。
5. Oは候補全工程、Pは従来経路の全工程対照、Qは起動通信で停止、Q2は同条件再試行。失敗データは性能比較から除外する。

条件ごとに condition.json、world.sdf、samples.jsonl、quality.json、CPU、Update、ROS一覧、launchログを保存。比較は support_removed 後の区間を使用する。全工程と180秒試験は区間が異なるため区別する。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 simulation_performance/optimization/summarize.py simulation_performance/next_stage
ROS_DOMAIN_ID=116 IGN_PARTITION=contact-native-test-isolated /usr/bin/python3 simulation_performance/next_stage/test_native_contact.py
```

判定：保留。全工程でRTFが対照より低下したため既定は ros を維持。aggregate_contacts:=false は従来の生 ROS Contacts を維持。物理拘束計算の削減とは扱わない。RTF=1.0 の達成を保証しない。

## 利用するAPIの根拠

Fortress にインストール済みの ignition-transport11 `Node.hh` の Subscribe と ignition-msgs8 `contacts.proto` / `contact.proto` を確認して実装。collision1/2、depth の対応は従来の ROS Contacts と同じ。Transport callback と ROS timer は別スレッドなので共有状態を mutex で保護する。ROS clock を引き続き使用するため期限は sim 時間。

変更箇所：cooperative_transport_gazebo の finger_contact_aggregator.cpp / CMakeLists.txt / package.xml、robot_bringup / friction_simulation / integrated_transport_simulation の launch。単独 robot bringup と集約無効時の生ROS経路は維持する。

最終レビューで時計読み取りをmutex取得後に変更し、新旧接続テストを再実行。性能表はその修正前の測定値で、修正後の全工程性能は再測定していない。採用保留のため通常設定には直接入力を適用しない。
