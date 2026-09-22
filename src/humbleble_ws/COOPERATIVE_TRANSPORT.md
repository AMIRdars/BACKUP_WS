# AMIR協調搬送（ROS 2 Humble / Gazebo Fortress）

`協調搬送実装.md`の段階A（固定拘束）を、このPCの既存AMIR構成へ合わせて実装したものです。
Jazzy/Harmonic向けの例とは異なり、以下を使用します。

- Ubuntu 22.04 / ROS 2 Humble
- Gazebo Sim 6 (Fortress) の `ignition-gazebo-detachable-joint-system`
- 既存の `/amir1`、`/amir2` 名前空間
- 既存の `geometry_msgs/msg/Twist` 入力 `/amirN/rover_twist`
- TFフレーム名のprefixではなく、既存構成どおり `/amirN/tf` topicで分離

## 実装範囲

- 8 kg、長さ1.2 m、把持厚30 mmの長尺搬送物と左右把持リンク
- AMIR 2台と搬送物を固定するDetachableJoint
- 2 ms物理ステップの評価ワールド
- attach/detach/state用ROS–Gazebo bridge
- 搬送物目標から2台の速度を同時生成する集中型コーディネータ
- 速度・加速度制限、オドメトリタイムアウト、同時停止
- 把持状態、台車間距離、速度、アーム関節限界を監視する安全ノード

段階B（摩擦だけの把持）、MoveItによる自動接近、F/Tセンサ、アドミッタンス制御は、
固定拘束での直進・旋回を安定させた後に追加する前提です。
段階Aでは指との二重拘束によるRTF低下を避けるため、搬送物のcollisionだけを両端
150 mmずつ短くしています（visualと慣性は1.2 mのまま）。段階Bへ進む際はcollisionを
実寸へ戻し、DetachableJointを無効にして接触パラメータを再調整してください。

## ビルド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cooperative_transport_bringup
source install/setup.bash
```

## 起動

```bash
ros2 launch cooperative_transport_bringup simulation.launch.py
```

GUIを使わない場合:

```bash
ros2 launch cooperative_transport_bringup simulation.launch.py headless:=true
```

起動時はrobot1、robot2、搬送物の順にスポーンします。搬送物は両TCPと合う初期位置にあり、
`attach_manager`が固定拘束コマンドを送ります。コーディネータは安全のため停止状態で開始します。
FortressのDetachableJointは初期attach時に`output_topic`を通知しないため、制御・安全監視には
attach managerの論理状態 `/cooperative_transport/amirN/grasp_state` を使用します。物理側から
状態遷移通知が来た場合は、その実値で論理状態を上書きします。

## 確認と搬送

コントローラと把持状態を確認します。

```bash
ros2 control list_controllers -c /amir1/controller_manager
ros2 control list_controllers -c /amir2/controller_manager
ros2 service call /cooperative_transport/grasp_status std_srvs/srv/Trigger '{}'
ros2 topic echo /cooperative_transport/safety_status
```

搬送物中心の目標（world座標、ここではx方向へ1 m）を設定します。

```bash
ros2 topic pub --times 3 --rate 5 /cooperative_transport/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 0.443}, orientation: {z: 0.0, w: 1.0}}}"
```

両台車の同期搬送を開始します。

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool '{data: true}'
```

停止:

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool '{data: false}'
```

90度旋回を含む目標では、yawの半角をQuaternionへ設定します。

```bash
ros2 topic pub --times 3 --rate 5 /cooperative_transport/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.443}, orientation: {z: 0.7071068, w: 0.7071068}}}"
```

## 把持操作と安全停止

```bash
# leaderを先、followerを0.5秒後にattach
ros2 service call /cooperative_transport/attach_all std_srvs/srv/SetBool '{data: true}'

# followerを先、leaderを0.5秒後にdetach
ros2 service call /cooperative_transport/attach_all std_srvs/srv/SetBool '{data: false}'
```

安全停止はラッチします。原因を解消し、搬送を無効にしてから解除します。

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /cooperative_transport/reset_safety std_srvs/srv/Trigger '{}'
```

主な監視topic:

```text
/cooperative_transport/state
/cooperative_transport/active
/cooperative_transport/payload_pose
/cooperative_transport/grasp_status
/cooperative_transport/amir1/grasp_state
/cooperative_transport/amir2/grasp_state
/cooperative_transport/safety_status
/cooperative_transport/emergency_stop
```

## 調整

速度・ゲイン・安全しきい値は
`cooperative_transport_control/config/cooperative_transport.yaml`で変更します。
最初は既定値の直進0.08 m/s以下、旋回0.10 rad/s以下で確認してください。
既存AMIRの初期姿勢はJoint_2/Joint_3がURDF境界上にあるため、初期設定では関節余裕を
0 rad（範囲外のみ停止）にしています。搬送用の非境界アーム姿勢を決めた後で5度
（0.0872665 rad）へ上げてください。0 rad設定時はシミュレーション数値誤差を許容する
ため、ハードリミット外1 mradまでは停止しません。

搬送を有効にできない場合は、順に次を確認します。

1. `/amir1/odom`と`/amir2/odom`が更新されていること
2. 両方の`grasp/state`が`true`であること
3. `/cooperative_transport/safety_status`が`OK`であること
4. 両controller managerの`mecanum_drive_controller`が`active`であること
