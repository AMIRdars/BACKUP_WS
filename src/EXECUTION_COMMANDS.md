# ROS 2 Humble 実行コマンド集

`src/MULTI_ROBOT_NAVIGATION.md`と`src/humbleble_ws`直下のMarkdown 8ファイルに記載された実行コマンドを、用途ごとに整理した一覧です。各コードブロックはコピーしやすいように1コマンド1行で記載しています。複数端末と書かれている項目は、それぞれ新しい端末で共通setupを実行してから使用してください。

## 1. 共通setup・インストール・ビルド

ワークスペースへ移動してROS 2とローカル環境を読み込む。

```bash
cd /home/dars5070/ros2_humble_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

リポジトリを取得してrosdep依存関係をインストールする。

```bash
mkdir -p ~/ros2_humble_ws/src && cd ~/ros2_humble_ws/src && git clone https://github.com/danbo-rusenki/humbleble_ws.git -b light-sim-ign && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
```

README記載のROS 2/Gazebo/controller依存パッケージをインストールする。

```bash
sudo apt update && sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-velocity-controllers ros-humble-effort-controllers ros-humble-joint-trajectory-controller ros-humble-position-controllers ros-humble-robot-state-publisher ros-humble-xacro ros-humble-gz-ros2-control
```

ワークスペース全体をビルドする。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
```

マルチロボットNav2関係だけをビルドする。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select amir_gazebo mecanum_navigation2 && source install/setup.bash
```

協調搬送関係を依存パッケージまでビルドする。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to cooperative_transport_bringup && source install/setup.bash
```

協調搬送の主要7パッケージを指定してビルドする。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select mecanumrover_description amir_description amir_gazebo cooperative_transport_description cooperative_transport_gazebo cooperative_transport_control cooperative_transport_bringup && source install/setup.bash
```

力覚監視関係の5パッケージをビルドする。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select amir_description amir_gazebo cooperative_transport_gazebo cooperative_transport_control cooperative_transport_bringup && source install/setup.bash
```

制御とbringupだけを再ビルドする。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select cooperative_transport_control cooperative_transport_bringup && source install/setup.bash
```

必要なPythonスクリプトへ実行権限を付ける。

```bash
chmod +x rover_twist_relay.py joint_state_filter.py
```

## 2. Gazeboと基本ロボット起動

標準Gazebo構成を起動する。

```bash
ros2 launch amir_gazebo gazebo_bringup.launch.py
```

Gazebo構成2を起動する。

```bash
ros2 launch amir_gazebo gazebo_bringup2.launch.py
```

Gazebo構成3を起動する。

```bash
ros2 launch amir_gazebo gazebo_bringup3.launch.py
```

amir1とamir2を同じGazebo worldへGUI付きで起動する。

```bash
ros2 launch amir_gazebo multi_robot.launch.py headless:=false
```

指定したwarehouse worldで2台を起動する。

```bash
ros2 launch amir_gazebo multi_robot.launch.py world:=warehouse_world.sdf world_name:=warehouse_world
```

amir1だけをheadless Gazeboへ起動する。

```bash
ros2 launch amir_gazebo robot_bringup.launch.py namespace:=amir1 launch_sim:=true headless:=true
```

amir2だけを初期位置`(0,1)`でheadless Gazeboへ起動する。

```bash
ros2 launch amir_gazebo robot_bringup.launch.py namespace:=amir2 x:=0.0 y:=1.0 launch_sim:=true headless:=true
```

## 3. マルチロボットSLAM・Nav2

amir1のSLAM/Nav2だけを名前空間付きで起動する。

```bash
ros2 launch mecanum_navigation2 bringup_launch.py namespace:=amir1 use_namespace:=true mode:=slam use_sim_time:=true use_rviz:=false
```

amir2のSLAM/Nav2だけを名前空間付きで起動する。

```bash
ros2 launch mecanum_navigation2 bringup_launch.py namespace:=amir2 use_namespace:=true mode:=slam use_sim_time:=true use_rviz:=false
```

amir1の地図情報を1回表示する。

```bash
ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info
```

amir2の地図情報を1回表示する。

```bash
ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info
```

両ロボットのNavigateToPose Actionを確認する。

```bash
ros2 action list | grep -E '^/amir[12]/navigate_to_pose$'
```

amir1へ`(0,-1)`のNav2ゴールを送る。

```bash
ros2 action send_goal /amir1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: -1.0}, orientation: {w: 1.0}}}}"
```

amir2へ`(0,0)`のNav2ゴールを送る。

```bash
ros2 action send_goal /amir2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

amir1の速度指令を監視する。

```bash
ros2 topic echo /amir1/cmd_vel geometry_msgs/msg/Twist
```

amir2の速度指令を監視する。

```bash
ros2 topic echo /amir2/cmd_vel geometry_msgs/msg/Twist
```

amir2の主要Nav2 lifecycle状態をまとめて確認する。

```bash
ros2 lifecycle get /amir2/controller_server && ros2 lifecycle get /amir2/planner_server && ros2 lifecycle get /amir2/bt_navigator
```

amir2のglobal costmap情報を1回表示する。

```bash
ros2 topic echo /amir2/global_costmap/costmap nav_msgs/msg/OccupancyGrid --once --field info
```

## 4. ロボット別RViz

amir1専用RVizを名前空間付きTF・scan・map・costmapへ接続して起動する。

```bash
ros2 run rviz2 rviz2 -d /home/dars5070/ros2_humble_ws/src/humbleble_ws/mecanum_navigation2/rviz/navigation.rviz --ros-args -r __ns:=/amir1 -p use_sim_time:=true -r /tf:=/amir1/tf -r /tf_static:=/amir1/tf_static -r /robot_description:=/amir1/robot_description -r /scan:=/amir1/scan -r /map:=/amir1/map -r /map_updates:=/amir1/map_updates -r /particle_cloud:=/amir1/particle_cloud -r /global_costmap/costmap:=/amir1/global_costmap/costmap -r /global_costmap/costmap_updates:=/amir1/global_costmap/costmap_updates -r /global_costmap/published_footprint:=/amir1/global_costmap/published_footprint -r /local_costmap/costmap:=/amir1/local_costmap/costmap -r /local_costmap/costmap_updates:=/amir1/local_costmap/costmap_updates -r /local_costmap/published_footprint:=/amir1/local_costmap/published_footprint -r /plan:=/amir1/plan -r /local_plan:=/amir1/local_plan -r /goal_pose:=/amir1/goal_pose -r /initialpose:=/amir1/initialpose -r /clicked_point:=/amir1/clicked_point
```

amir2専用RVizを名前空間付きTF・scan・map・costmapへ接続して起動する。

```bash
ros2 run rviz2 rviz2 -d /home/dars5070/ros2_humble_ws/src/humbleble_ws/mecanum_navigation2/rviz/navigation.rviz --ros-args -r __ns:=/amir2 -p use_sim_time:=true -r /tf:=/amir2/tf -r /tf_static:=/amir2/tf_static -r /robot_description:=/amir2/robot_description -r /scan:=/amir2/scan -r /map:=/amir2/map -r /map_updates:=/amir2/map_updates -r /particle_cloud:=/amir2/particle_cloud -r /global_costmap/costmap:=/amir2/global_costmap/costmap -r /global_costmap/costmap_updates:=/amir2/global_costmap/costmap_updates -r /global_costmap/published_footprint:=/amir2/global_costmap/published_footprint -r /local_costmap/costmap:=/amir2/local_costmap/costmap -r /local_costmap/costmap_updates:=/amir2/local_costmap/costmap_updates -r /local_costmap/published_footprint:=/amir2/local_costmap/published_footprint -r /plan:=/amir2/plan -r /local_plan:=/amir2/local_plan -r /goal_pose:=/amir2/goal_pose -r /initialpose:=/amir2/initialpose -r /clicked_point:=/amir2/clicked_point
```

## 5. 地図保存・AMCL

amir1のSLAM地図を`amir_world.yaml/.pgm`として保存する。

```bash
ros2 run nav2_map_server map_saver_cli -f /home/dars5070/ros2_humble_ws/src/humbleble_ws/mecanum_navigation2/map/amir_world --ros-args -r map:=/amir1/map
```

amir1を保存地図＋AMCL localizationモードで起動する。

```bash
ros2 launch mecanum_navigation2 bringup_launch.py namespace:=amir1 use_namespace:=true mode:=localization use_sim_time:=true use_rviz:=false
```

amir2を保存地図＋AMCL localizationモードで起動する。

```bash
ros2 launch mecanum_navigation2 bringup_launch.py namespace:=amir2 use_namespace:=true mode:=localization use_sim_time:=true use_rviz:=false
```

amir1のAMCL初期姿勢を`(0,0)`に設定する。

```bash
ros2 topic pub -r 1 --times 3 /amir1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

amir2のAMCL初期姿勢を`(0,1)`に設定する。

```bash
ros2 topic pub -r 1 --times 3 /amir2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

amir1のAMCL推定姿勢を1回表示する。

```bash
ros2 topic echo /amir1/amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once
```

amir2のAMCL推定姿勢を1回表示する。

```bash
ros2 topic echo /amir2/amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once
```

## 6. Fleet manager・同時ナビゲーション

2台用fleet managerを起動する。

```bash
ros2 launch mecanum_navigation2 fleet_manager.launch.py
```

amir1のfleet状態を監視する。

```bash
ros2 topic echo /fleet/amir1/status std_msgs/msg/String
```

fleet manager経由でamir1へ`(0,-1.1)`のゴールを送る。

```bash
ros2 topic pub --once /fleet/amir1/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 0.0, y: -1.1}, orientation: {w: 1.0}}}"
```

fleet managerで実行中の全ゴールをキャンセルする。

```bash
ros2 service call /fleet/cancel_all std_srvs/srv/Trigger "{}"
```

既定目標で2台同時ナビゲーションを実行する。

```bash
ros2 run mecanum_navigation2 simultaneous_navigation.py
```

amir1を`(0,-1)`、amir2を`(0,2)`へ同時ナビゲーションする。

```bash
ros2 run mecanum_navigation2 simultaneous_navigation.py --ros-args -p amir1_x:=0.0 -p amir1_y:=-1.0 -p amir1_yaw:=0.0 -p amir2_x:=0.0 -p amir2_y:=2.0 -p amir2_yaw:=0.0
```

## 7. Nav2スラロームステージ

スラロームworldで2台のロボットをGUI付き起動する。

```bash
ros2 launch amir_gazebo multi_robot.launch.py world:=slalom_world.sdf headless:=false
```

既存worldへ円柱スラロームステージをspawnする。

```bash
ros2 launch amir_gazebo spawn_slalom_stage.launch.py world:=default entity_name:=slalom_stage x:=0.0 y:=0.0 z:=0.0 yaw:=0.0
```

Ignition Gazeboのentity生成serviceを確認する。

```bash
ign service -l | grep -E '/world/.*/create'
```

両ロボットのLaserScan、地図、Nav2 Actionを確認する。

```bash
ros2 topic echo /amir1/scan sensor_msgs/msg/LaserScan --once && ros2 topic echo /amir2/scan sensor_msgs/msg/LaserScan --once && ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info && ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info && ros2 action list | grep -E '/amir[12]/(navigate_to_pose|follow_waypoints)'
```

amir1へS字状の9点waypointを送る。

```bash
ros2 action send_goal /amir1/follow_waypoints nav2_msgs/action/FollowWaypoints "{poses: [{header: {frame_id: map}, pose: {position: {x: 3.0, y: -0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 4.6, y: 0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 6.2, y: -0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 7.8, y: 0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 9.4, y: -0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 11.0, y: 0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 12.6, y: -0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 14.2, y: 0.5}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 16.0, y: 0.0}, orientation: {w: 1.0}}}]}"
```

## 8. 協調搬送の基本操作

標準協調搬送シミュレーションをGUI付きで起動する。

```bash
ros2 launch cooperative_transport_bringup simulation.launch.py
```

標準協調搬送シミュレーションをheadlessで起動する。

```bash
ros2 launch cooperative_transport_bringup simulation.launch.py headless:=true
```

両ロボットのcontroller一覧を確認する。

```bash
ros2 control list_controllers -c /amir1/controller_manager && ros2 control list_controllers -c /amir2/controller_manager
```

協調搬送の把持状態を確認する。

```bash
ros2 service call /cooperative_transport/grasp_status std_srvs/srv/Trigger '{}'
```

協調搬送の安全状態を監視する。

```bash
ros2 topic echo /cooperative_transport/safety_status
```

搬送物の目標を`world`座標`(1,0,0.443)`へ設定する。

```bash
ros2 topic pub --times 3 --rate 5 /cooperative_transport/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 0.443}, orientation: {z: 0.0, w: 1.0}}}"
```

搬送物の目標を原点で90度回転した姿勢へ設定する。

```bash
ros2 topic pub --times 3 --rate 5 /cooperative_transport/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.443}, orientation: {z: 0.7071068, w: 0.7071068}}}"
```

協調搬送制御を有効にする。

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool '{data: true}'
```

協調搬送制御を停止する。

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool '{data: false}'
```

leader、followerの順で搬送物をattachする。

```bash
ros2 service call /cooperative_transport/attach_all std_srvs/srv/SetBool '{data: true}'
```

follower、leaderの順で搬送物をdetachする。

```bash
ros2 service call /cooperative_transport/attach_all std_srvs/srv/SetBool '{data: false}'
```

安全停止を解除する前に制御を停止して安全状態をリセットする。

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool '{data: false}' && ros2 service call /cooperative_transport/reset_safety std_srvs/srv/Trigger '{}'
```

## 9. 摩擦把持・滑り検知

摩擦把持シミュレーションをGUI付きで起動する。

```bash
ros2 launch cooperative_transport_bringup friction_simulation.launch.py
```

摩擦把持シミュレーションをheadlessで起動する。

```bash
ros2 launch cooperative_transport_bringup friction_simulation.launch.py headless:=true
```

自動把持を無効にして摩擦把持シミュレーションを起動する。

```bash
ros2 launch cooperative_transport_bringup friction_simulation.launch.py auto_grasp:=false
```

摩擦把持状態を監視する。

```bash
ros2 topic echo /cooperative_transport/friction_grasp_status
```

滑り・安全・制御状態を各1回表示する。

```bash
ros2 topic echo --once /cooperative_transport/slip_status && ros2 topic echo --once /cooperative_transport/safety_status && ros2 topic echo --once /cooperative_transport/state
```

低速直進目標を送信する。

```bash
ros2 topic pub -r 2 -t 3 /cooperative_transport/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 0.10, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

摩擦把持を開始する。

```bash
ros2 service call /cooperative_transport/friction_grasp std_srvs/srv/SetBool "{data: true}"
```

摩擦把持を解除する。

```bash
ros2 service call /cooperative_transport/friction_grasp std_srvs/srv/SetBool "{data: false}"
```

滑り状態を監視する。

```bash
ros2 topic echo /cooperative_transport/slip_status
```

搬送停止後に滑りと安全状態をリセットする。

```bash
ros2 service call /cooperative_transport/set_enabled std_srvs/srv/SetBool "{data: false}" && ros2 service call /cooperative_transport/reset_slip std_srvs/srv/Trigger "{}" && ros2 service call /cooperative_transport/reset_safety std_srvs/srv/Trigger "{}"
```

## 10. 軌道評価・障害物スラローム・開空間スラローム

30度旋回シナリオの軌道評価をGUI付きで起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=turn_30
```

短距離スラロームの軌道評価をheadlessで起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py headless:=true scenario:=slalom_short
```

障害物スラロームの軌道評価をGUI付きで起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_obstacles
```

障害物スラロームの軌道評価をheadlessで起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py headless:=true scenario:=slalom_obstacles
```

開空間の大振幅スラロームをGUI付きで起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_open
```

開空間の大振幅スラロームをheadlessで起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_open headless:=true
```

手動開始モードで30度旋回評価を起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=turn_30 auto_start:=false
```

軌道評価を手動開始する。

```bash
ros2 service call /cooperative_transport/start_evaluation std_srvs/srv/Trigger '{}'
```

軌道評価を中止する。

```bash
ros2 service call /cooperative_transport/cancel_evaluation std_srvs/srv/Trigger '{}'
```

軌道評価の進行状態を監視する。

```bash
ros2 topic echo /cooperative_transport/evaluation_status
```

軌道評価の最終結果をTransient Local QoSで1回表示する。

```bash
ros2 topic echo --once /cooperative_transport/evaluation_result std_msgs/msg/String --qos-durability transient_local
```

計測した搬送物姿勢を監視する。

```bash
ros2 topic echo /cooperative_transport/measured_payload_pose
```

障害物監視状態を表示する。

```bash
ros2 topic echo /cooperative_transport/obstacle_status
```

障害物との最小クリアランスを表示する。

```bash
ros2 topic echo /cooperative_transport/minimum_obstacle_clearance
```

支持物が取り除かれたかをTransient Local QoSで確認する。

```bash
ros2 topic echo --once /cooperative_transport/support_removed std_msgs/msg/Bool --qos-durability transient_local
```

CSV保存先を作り、結果を指定ディレクトリへ保存するheadless評価を起動する。

```bash
mkdir -p ~/cooperative_transport_results && ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py headless:=true scenario:=slalom_short results_directory:=$HOME/cooperative_transport_results
```

## 11. 横方向アドミッタンス

アドミッタンス付き障害物スラロームを起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_obstacles
```

横方向アドミッタンスを無効にした比較走行を起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_obstacles enable_lateral_admittance:=false
```

横方向アドミッタンスと力覚安全停止を有効にする。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_obstacles enable_lateral_admittance:=true enable_wrench_safety:=true
```

アドミッタンス状態を監視する。

```bash
ros2 topic echo /cooperative_transport/admittance_status
```

アドミッタンス補正量を監視する。

```bash
ros2 topic echo /cooperative_transport/admittance_correction
```

## 12. 力覚監視・安全停止

両ロボットの力覚センサーを個別に監視する。

```bash
ros2 topic echo /amir1/ft_sensor
```

```bash
ros2 topic echo /amir2/ft_sensor
```

協調搬送の力覚状態を監視する。

```bash
ros2 topic echo /cooperative_transport/wrench_status
```

両力覚センサーの配信周波数を確認する。

```bash
ros2 topic hz /amir1/ft_sensor
```

```bash
ros2 topic hz /amir2/ft_sensor
```

力覚安全停止を有効にした障害物スラロームを起動する。

```bash
ros2 launch cooperative_transport_bringup trajectory_evaluation.launch.py scenario:=slalom_obstacles enable_wrench_safety:=true
```

力覚異常状態を監視する。

```bash
ros2 topic echo /cooperative_transport/wrench_fault
```

安全停止状態をリセットする。

```bash
ros2 service call /cooperative_transport/reset_safety std_srvs/srv/Trigger '{}'
```

## 13. テスト

協調搬送controlパッケージをテストして詳細結果を表示する。

```bash
cd ~/ros2_humble_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select cooperative_transport_control && colcon test-result --test-result-base build/cooperative_transport_control --verbose
```

## 14. MoveIt・ピック＆プレース・Behavior Tree・VR/MR

amir1のMoveIt/Gazeboを起動する。

```bash
ros2 launch amir_moveit_config moveit_gazebo.launch.py namespace:=amir1
```

amir1のMoveIt/GazeboとNav2を一緒に起動する。

```bash
ros2 launch amir_moveit_config moveit_gazebo.launch.py namespace:=amir1 nav2:=true
```

amir1のpick/place/move_meca Action Serverを起動する。

```bash
ros2 launch amir_operation pick_and_place_launch.py namespace:=amir1
```

固定箱をspawnする。

```bash
ros2 launch mecanumrover3_gazebo spawn_koteibox.launch.py
```

10個の箱をspawnする。

```bash
ros2 launch mecanumrover3_gazebo spawn_10box.launch.py
```

複数箱をspawnする。

```bash
ros2 launch mecanumrover3_gazebo spawn_multibox.launch.py
```

指定スケールでworld objectをspawnする。

```bash
ros2 launch mecanumrover3_gazebo spawn_wor.launch.py scale:=0.001
```

Gazebo用初期姿勢処理を実行する。

```bash
ros2 run amir_operation initial_posi_gz
```

MoveIt/Gazeboを既定名前空間で起動する。

```bash
ros2 launch amir_moveit_config moveit_gazebo.launch.py
```

固定物用pick/placeを起動する。

```bash
ros2 launch amir_operation pick_place_fix_launch.py
```

10個箱用pick/placeを起動する。

```bash
ros2 launch amir_operation pick_place_10_launch.py
```

既定設定のpick and placeを起動する。

```bash
ros2 launch amir_operation pick_and_place_launch.py
```

Behavior Tree XML送信ノードを起動する。

```bash
ros2 launch bt_generator bt_send_xml_launch.py
```

Behavior Tree executorを起動する。

```bash
ros2 launch ros2_behavior_tree bt_executor_launch.py
```

Gazebo pose filter utilityを実行する。

```bash
ros2 run my_utility gz_pose_filter
```

VR/MR用MoveIt Servoを起動する。

```bash
ros2 launch amir_operation vr_servo_launch.py
```

arm controllerからforward position controllerへ切り替える。

```bash
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
```

forward position controllerからarm controllerへ戻す。

```bash
ros2 control switch_controllers --deactivate forward_position_controller --activate arm_controller
```

controller一覧を表示する。

```bash
ros2 control list_controllers
```

MoveIt Servo状態を監視する。

```bash
ros2 topic echo /servo_node/status
```

forward position controllerへの関節角指令を監視する。

```bash
ros2 topic echo /forward_position_controller/commands
```

