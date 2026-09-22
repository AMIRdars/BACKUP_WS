# amir1 / amir2 マルチロボット Nav2 起動手順

この手順は ROS 2 Humble、Gazebo Sim、Nav2 を使用する。各コマンドを実行する前に、端末ごとに次を実行する。

```bash
cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

初回またはソース変更後はビルドする。

```bash
colcon build --symlink-install --packages-select amir_gazebo mecanum_navigation2
source install/setup.bash
```

画面表示が不要な場合は `headless:=true` を使う。GUIを表示する場合は `headless:=false` にする。次の段階へ進む前に、前段階のlaunchを `Ctrl-C` で終了する。

## 構成

各ロボットは同じframe ID（`map`、`odom`、`base_footprint`）を使うが、TFトピック自体を名前空間で分離している。

| 用途 | amir1 | amir2 |
|---|---|---|
| LaserScan | `/amir1/scan` | `/amir2/scan` |
| SLAM/保存地図 | `/amir1/map` | `/amir2/map` |
| TF | `/amir1/tf` | `/amir2/tf` |
| Nav2速度指令 | `/amir1/cmd_vel` | `/amir2/cmd_vel` |
| 車体への中継 | `/amir1/rover_twist` | `/amir2/rover_twist` |
| Nav2 Action | `/amir1/navigate_to_pose` | `/amir2/navigate_to_pose` |

`/local_costmap/scan` や `/global_costmap/map` は使用しない。costmapは実センサー・地図である `/<robot>/scan` と `/<robot>/map` を購読する。Nav2が配信するcostmapの確認先は `/<robot>/global_costmap/costmap` である。

## 1. amir1だけをnamespace付きNav2で確認

端末1:

```bash
ros2 launch amir_gazebo robot_bringup.launch.py \
  namespace:=amir1 launch_sim:=true headless:=true
```

端末2（controllerの起動後）:

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir1 use_namespace:=true mode:=slam use_sim_time:=true use_rviz:=false
```

RVizも確認する場合は、Nav2起動後に別端末から段階3「端末5」のamir1用RVizコマンドを実行する。

確認:

```bash
ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 action send_goal /amir1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: -1.0}, orientation: {w: 1.0}}}}"
ros2 topic echo /amir1/cmd_vel geometry_msgs/msg/Twist
```

地図の `width` と `height` が0でなく、ゴール結果が `SUCCEEDED`、走行中の速度が非ゼロなら正常である。

## 2. amir2だけをnamespace付きNav2で確認

端末1:

```bash
ros2 launch amir_gazebo robot_bringup.launch.py \
  namespace:=amir2 x:=0.0 y:=1.0 launch_sim:=true headless:=true
```

端末2:

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir2 use_namespace:=true mode:=slam use_sim_time:=true use_rviz:=false
```

RVizも確認する場合は、Nav2起動後に別端末から段階3「端末6」のamir2用RVizコマンドを実行する。

確認:

```bash
ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 action send_goal /amir2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
ros2 topic echo /amir2/cmd_vel geometry_msgs/msg/Twist
```

## 3. Gazebo、Nav2、RVizを別々に起動して2台同時SLAM

以下はすべて別端末で実行する。各端末で、冒頭に記載したworkspaceのsetupをsourceする。

### 端末1: Gazeboと2台のロボットだけを起動

```bash
ros2 launch amir_gazebo multi_robot.launch.py headless:=false
```

Gazebo画面が不要な場合だけ `headless:=true` に変更する。amir1のcontroller起動後、約9秒でamir2が同じworldへspawnする。

### 端末2: amir1のSLAM/Nav2だけを起動

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir1 use_namespace:=true \
  mode:=slam use_sim_time:=true use_rviz:=false
```

### 端末3: amir2のSLAM/Nav2だけを起動

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir2 use_namespace:=true \
  mode:=slam use_sim_time:=true use_rviz:=false
```

### 端末4: fleet managerだけを起動

```bash
ros2 launch mecanum_navigation2 fleet_manager.launch.py
```

fleet managerを使わず、それぞれの `navigate_to_pose` Actionへ直接ゴールを送る場合はこの端末を省略できる。

### 端末5: amir1用RVizだけを起動

```bash
export ROBOT_NS=amir1

ros2 run rviz2 rviz2 \
  -d /home/dars5070/ros2_humble_ws/src/humbleble_ws/mecanum_navigation2/rviz/navigation.rviz \
  --ros-args \
  -r __ns:=/${ROBOT_NS} \
  -p use_sim_time:=true \
  -r /tf:=/${ROBOT_NS}/tf \
  -r /tf_static:=/${ROBOT_NS}/tf_static \
  -r /robot_description:=/${ROBOT_NS}/robot_description \
  -r /scan:=/${ROBOT_NS}/scan \
  -r /map:=/${ROBOT_NS}/map \
  -r /map_updates:=/${ROBOT_NS}/map_updates \
  -r /particle_cloud:=/${ROBOT_NS}/particle_cloud \
  -r /global_costmap/costmap:=/${ROBOT_NS}/global_costmap/costmap \
  -r /global_costmap/costmap_updates:=/${ROBOT_NS}/global_costmap/costmap_updates \
  -r /global_costmap/published_footprint:=/${ROBOT_NS}/global_costmap/published_footprint \
  -r /local_costmap/costmap:=/${ROBOT_NS}/local_costmap/costmap \
  -r /local_costmap/costmap_updates:=/${ROBOT_NS}/local_costmap/costmap_updates \
  -r /local_costmap/published_footprint:=/${ROBOT_NS}/local_costmap/published_footprint \
  -r /plan:=/${ROBOT_NS}/plan \
  -r /local_plan:=/${ROBOT_NS}/local_plan \
  -r /goal_pose:=/${ROBOT_NS}/goal_pose \
  -r /initialpose:=/${ROBOT_NS}/initialpose \
  -r /clicked_point:=/${ROBOT_NS}/clicked_point
```

### 端末6: amir2用RVizだけを起動

端末5と同じコマンドを使い、変数だけamir2へ変更する。

```bash
export ROBOT_NS=amir2

ros2 run rviz2 rviz2 \
  -d /home/dars5070/ros2_humble_ws/src/humbleble_ws/mecanum_navigation2/rviz/navigation.rviz \
  --ros-args \
  -r __ns:=/${ROBOT_NS} \
  -p use_sim_time:=true \
  -r /tf:=/${ROBOT_NS}/tf \
  -r /tf_static:=/${ROBOT_NS}/tf_static \
  -r /robot_description:=/${ROBOT_NS}/robot_description \
  -r /scan:=/${ROBOT_NS}/scan \
  -r /map:=/${ROBOT_NS}/map \
  -r /map_updates:=/${ROBOT_NS}/map_updates \
  -r /particle_cloud:=/${ROBOT_NS}/particle_cloud \
  -r /global_costmap/costmap:=/${ROBOT_NS}/global_costmap/costmap \
  -r /global_costmap/costmap_updates:=/${ROBOT_NS}/global_costmap/costmap_updates \
  -r /global_costmap/published_footprint:=/${ROBOT_NS}/global_costmap/published_footprint \
  -r /local_costmap/costmap:=/${ROBOT_NS}/local_costmap/costmap \
  -r /local_costmap/costmap_updates:=/${ROBOT_NS}/local_costmap/costmap_updates \
  -r /local_costmap/published_footprint:=/${ROBOT_NS}/local_costmap/published_footprint \
  -r /plan:=/${ROBOT_NS}/plan \
  -r /local_plan:=/${ROBOT_NS}/local_plan \
  -r /goal_pose:=/${ROBOT_NS}/goal_pose \
  -r /initialpose:=/${ROBOT_NS}/initialpose \
  -r /clicked_point:=/${ROBOT_NS}/clicked_point
```

各RVizは自分の `/<robot>/tf` だけを使うため、同じframe IDを持つ2台が混線しない。RVizを終了してもGazeboとNav2は動作を継続する。

```bash
ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 action list | grep navigate_to_pose
```

`/amir1/map` と `/amir2/map` は別々のSLAM toolboxから配信される独立地図である。

## 4. cmd_velの独立性を確認

端末を2つ用意し、それぞれを観測する。

```bash
ros2 topic echo /amir1/cmd_vel geometry_msgs/msg/Twist
```

```bash
ros2 topic echo /amir2/cmd_vel geometry_msgs/msg/Twist
```

さらに別々の端末から異なるゴールを送る。

```bash
ros2 action send_goal /amir1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: -0.8}, orientation: {w: 1.0}}}}"
```

```bash
ros2 action send_goal /amir2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

経路全体も次のように分離される。

```text
/amir1/cmd_vel -> /amir1/rover_twist -> /amir1/mecanum_drive_controller/reference_unstamped
/amir2/cmd_vel -> /amir2/rover_twist -> /amir2/mecanum_drive_controller/reference_unstamped
```

## 5. 保存地図と各ロボットのAMCLへ移行

### 地図保存

独立SLAM起動中に、代表としてamir1の地図を保存する。次のコマンドは既存の `amir_world.yaml/.pgm` を更新するため、必要なときだけ実行する。

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/dars5070/ros2_humble_ws/src/humbleble_ws/mecanum_navigation2/map/amir_world \
  --ros-args -r map:=/amir1/map
```

保存後に端末2と端末3のSLAM/Nav2だけを終了する。Gazeboはそのまま動作させてもよい。次に、別端末で各AMCL/Nav2を起動する。

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir1 use_namespace:=true \
  mode:=localization use_sim_time:=true use_rviz:=false
```

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir2 use_namespace:=true \
  mode:=localization use_sim_time:=true use_rviz:=false
```

Gazeboまで終了した場合は、最初に段階3の端末1のコマンドでGazeboを再起動する。RVizとfleet managerは段階3の端末4〜6のコマンドで、それぞれ独立して起動できる。

### AMCL初期姿勢

`/amir1/initialpose` と `/amir2/initialpose` が見えるまで待ち、Gazeboのspawn位置を設定する。起動直後のDDS discovery競合を避けるため3回送信する。

```bash
ros2 topic pub -r 1 --times 3 /amir1/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

```bash
ros2 topic pub -r 1 --times 3 /amir2/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

実機やspawn位置を変更した場合は、既知の実位置に合わせて `x`、`y`、姿勢を変更する。RVizの「2D Pose Estimate」でも設定できる。

確認:

```bash
ros2 topic echo /amir1/amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once
ros2 topic echo /amir2/amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once
ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info
```

初期姿勢設定後、両方の `navigate_to_pose` でゴールが成功すれば移行完了である。

## 6. fleet manager

Gazebo、Nav2、RVizと分離して、専用端末で次を実行する。

```bash
ros2 launch mecanum_navigation2 fleet_manager.launch.py
```

ロボットごとのゴール入力と状態出力:

```text
/fleet/amir1/goal_pose  -> /amir1/navigate_to_pose
/fleet/amir2/goal_pose  -> /amir2/navigate_to_pose
/fleet/amir1/status
/fleet/amir2/status
```

先に状態を監視する。

```bash
ros2 topic echo /fleet/amir1/status std_msgs/msg/String
```

別端末からゴールを送る。

```bash
ros2 topic pub --once /fleet/amir1/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: -1.1}, orientation: {w: 1.0}}}"
```

正常時は `goal sent`、`goal accepted`、`goal finished with status 4` の順に通知される。ROS 2 Actionのstatus `4` は成功である。全ロボットの実行中ゴールをキャンセルするには次を使う。

```bash
ros2 service call /fleet/cancel_all std_srvs/srv/Trigger "{}"
```

このfleet managerは名前空間付きNav2へのゴール振り分けと状態集約を行う。複数ロボット間の衝突回避、交通整理、タスク割り当て最適化は行わない。

## 7. スラロール

通常の `amir_world.sdf` は外周壁を持つ約6 m四方の確認用worldであり、全長約18 mのスラロールコースには使用しない。スラロール実行時は、外周壁のない `slalom_world.sdf` を指定し、Gazebo起動後に円柱ステージを別エンティティとしてスポーンする。

今回追加したファイル:

```text
amir_gazebo/
├── launch/spawn_slalom_stage.launch.py
├── models/slalom_stage/model.config
├── models/slalom_stage/model.sdf
└── worlds/slalom_world.sdf
```

ステージには緑色のスタートゲート、左右交互に並ぶオレンジ色の円柱障害物8本、青色のフィニッシュゲートがある。円柱の半径は0.28 m、高さは1.0 mである。

### 初回またはステージ変更後のビルド

```bash
cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amir_gazebo mecanum_navigation2
source install/setup.bash
```

### 端末1: スラロール用worldと2台のロボット

```bash
ros2 launch amir_gazebo multi_robot.launch.py \
  world:=slalom_world.sdf headless:=false
```

画面が不要な場合は `headless:=true` にする。`amir1` のcontrollerが起動し、その後 `amir2` がスポーンするまで待つ。

### 端末2: 円柱ステージをスポーン

Gazebo起動後、`/world/default/create` が利用可能になってから実行する。

```bash
ros2 launch amir_gazebo spawn_slalom_stage.launch.py \
  world:=default entity_name:=slalom_stage \
  x:=0.0 y:=0.0 z:=0.0 yaw:=0.0
```

world名を確認する場合:

```bash
ign service -l | grep -E '/world/.*/create'
```

`Requested entity name already exists` と表示された場合は、Gazeboを再起動するか `entity_name:=slalom_stage_2` のように別名を指定する。

### 端末3: amir1のSLAM/Nav2

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir1 use_namespace:=true \
  mode:=slam use_sim_time:=true use_rviz:=false
```

### 端末4: amir2のSLAM/Nav2 trueでRviz2表示

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir2 use_namespace:=true \
  mode:=slam use_sim_time:=true use_rviz:=true
```

RVizとfleet managerは、段階3の端末4〜6と同じ方法で必要なものだけ別端末から起動する。

### ステージ認識の確認

```bash
ros2 topic echo /amir1/scan sensor_msgs/msg/LaserScan --once
ros2 topic echo /amir2/scan sensor_msgs/msg/LaserScan --once
ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 action list | grep -E '/amir[12]/(navigate_to_pose|follow_waypoints)'
```

LaserScanの `ranges` に有限値が入り、両方の地図の `width` と `height` が0でなければステージを認識している。SLAM中は走行に合わせて未観測部分が順次地図へ追加される。

### amir1でスラロール走行

次のwaypoint列は、`y=1.25` の円柱を下側、`y=-1.25` の円柱を上側から交互に通過し、フィニッシュゲートへ向かう。最初はamir1だけで実行する。

```bash
ros2 action send_goal /amir1/follow_waypoints \
  nav2_msgs/action/FollowWaypoints \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 3.0, y: -0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 4.6, y: 0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 6.2, y: -0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 7.8, y: 0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 9.4, y: -0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 11.0, y: 0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 12.6, y: -0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 14.2, y: 0.5}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 16.0, y: 0.0}, orientation: {w: 1.0}}}
  ]}"
```

途中の地図がまだ生成されておらず経路作成に失敗する場合は、RVizで地図を確認しながら近いwaypointへ1つずつ `navigate_to_pose` を送ってコースを探索する。

amir2でも同じ形式で `/amir2/follow_waypoints` を使用できる。ただし、このfleet managerはロボット間の衝突回避を行わないため、最初から2台を同時走行させない。1台ずつ完走を確認した後、十分な開始時間差を設ける。

終了時はNav2、ステージ、Gazeboの順で各端末を `Ctrl-C` する。ステージだけを終了しても、スポーン済みエンティティはGazeboを終了するまで残る。

## RVizとログの注意点

- amir1用とamir2用のRVizは別プロセスとして起動する。1つのRVizへ2台の同名frameを混ぜない。
- RVizだけを再起動しても、Gazebo、SLAM、AMCL、Nav2を再起動する必要はない。
- RVizの `Downsampled Costmap: No map received` は、現在使用するNavFn plannerが `/downsampled_costmap` を配信しないためで、通常のglobal costmapやNavigation失敗を意味しない。このDisplayは無効化してよい。
- 起動直後の `timestamp ... earlier than all the data in the transform cache` が一時的に出る場合がある。継続する場合は、全Nav2/SLAM/AMCLノードで `use_sim_time:=true` になっていることと、`/clock` のPublisherが1つだけであることを確認する。
- SLAMの地図が `width: 0, height: 0` にならないよう、Gazebo worldにはLiDARで観測できる外周壁と障害物を配置してある。

## 同時ナビゲーション

スラロール用ではない通常の `amir_world.sdf` を使用し、2台がそれぞれ生成する通常のSLAM地図上で異なる目標位置をほぼ同時に送信する。専用コマンドは両方のNav2 Actionへゴールを送り、次の3条件を自動判定する。

1. `/amir1/navigate_to_pose` と `/amir2/navigate_to_pose` がそれぞれゴールを受理する。
2. `/amir1/cmd_vel` と `/amir2/cmd_vel` に同時期の非ゼロ速度が出る。
3. 2台がそれぞれ異なる目標位置へ到着する。

既定の目標は、開始位置から経路が分かれる次の位置である。

| ロボット | 開始位置 | 目標位置 |
|---|---:|---:|
| amir1 | `(0.0, 0.0)` | `(0.0, -0.8)` |
| amir2 | `(0.0, 1.0)` | `(0.0, 1.8)` |

### 初回またはソース変更後のビルド

```bash
cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amir_gazebo mecanum_navigation2
source install/setup.bash
```

### 端末1: 通常worldと2台のロボットを起動

`world:=amir_world.sdf` を明示しているため、スラロールステージは使用しない。

```bash
ros2 launch amir_gazebo multi_robot.launch.py \
  world:=amir_world.sdf headless:=false
```

Gazebo画面が不要な場合は `headless:=true` に変更できる。`amir1` と `amir2` の両方が表示されるまで待つ。

### 端末2: amir1の通常SLAM/Nav2とRVizを起動

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir1 use_namespace:=true \
  mode:=slam \
  use_sim_time:=true use_rviz:=true
```

### 端末3: amir2の通常SLAM/Nav2とRVizを起動

```bash
ros2 launch mecanum_navigation2 bringup_launch.py \
  namespace:=amir2 use_namespace:=true \
  mode:=slam \
  use_sim_time:=true use_rviz:=true
```

起動直後のDDS discovery競合を避けるため、Nav2は内部で3秒待ってから自動的にactivateされる。それぞれのRVizのNavigation 2パネルが `active` になるまで待つ。amir1用RVizのGlobal Costmapは `/amir1/global_costmap/costmap`、amir2用RVizは `/amir2/global_costmap/costmap` を購読する。

`use_rviz` に指定できる値は `true` または `false` である。スペルを間違えた場合は、RVizを黙って無効化せずlaunchエラーとして通知される。

### 端末4: 地図とActionサーバーを確認

両方の地図が生成され、Actionサーバーが表示されるまで待つ。SLAMモードなのでAMCL初期姿勢の送信は不要である。

```bash
ros2 topic echo /amir1/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 topic echo /amir2/map nav_msgs/msg/OccupancyGrid --once --field info
ros2 action list | grep -E '^/amir[12]/navigate_to_pose$'
```

両方の地図の `width` と `height` が0でなく、2つのAction名が表示されれば準備完了である。

amir2のNav2状態とGlobal Costmapを個別に確認する場合:

```bash
ros2 lifecycle get /amir2/controller_server
ros2 lifecycle get /amir2/planner_server
ros2 lifecycle get /amir2/bt_navigator
ros2 topic echo /amir2/global_costmap/costmap \
  nav_msgs/msg/OccupancyGrid --once --field info
```

3つのlifecycle状態がすべて `active [3]` となり、Global Costmapの `width` と `height` が0でなければ正常である。現在のNavFn plannerはDownsampled Costmapを配信しないため、そのRViz表示は既定で無効にしている。

### 端末4: 異なる目標を同時送信して自動確認

```bash
ros2 run mecanum_navigation2 simultaneous_navigation.py
```

走行中はGazebo上で2台が同時に異なる方向へ動く。成功時は最後に次のメッセージが表示され、コマンドは終了コード0で終了する。

```text
SIMULTANEOUS MOTION CONFIRMED: /amir1/cmd_vel and /amir2/cmd_vel are both active
amir1: goal succeeded
amir2: goal succeeded
SIMULTANEOUS NAVIGATION PASSED: both robots moved concurrently and reached different goals
```

目標を変更する場合は、通常マップの壁と中央障害物を避けた座標を指定する。

```bash
ros2 run mecanum_navigation2 simultaneous_navigation.py --ros-args \
  -p amir1_x:=0.0 -p amir1_y:=-1.0 -p amir1_yaw:=0.0 \
  -p amir2_x:=0.0 -p amir2_y:=2.0 -p amir2_yaw:=0.0
```

途中で中止した場合は、各Nav2端末とGazebo端末を `Ctrl-C` で終了する。
