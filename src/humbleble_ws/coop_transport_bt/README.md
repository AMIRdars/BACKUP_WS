# Cooperative transport BT

This package implements the guide's separation of responsibilities:

```text
BehaviorTree.CPP -> /cooperative_motion Action -> cooperative_coordinator
                                      -> /amir1/rover_twist, /amir2/rover_twist
```

`CooperativeMotion` is a `StatefulActionNode`. Its ports are read on each
`onStart`, its result is polled in `onRunning`, and halting it cancels its
active Action goal. `EmergencyStop` disables the coordinator and publishes a
zero `Twist` to both bases.

The simulation launch starts the separate `arm_home_positioner` executable
after all configured robots are spawned. It sends the common `Q_HOME` posture
to every configured AMIR arm controller while tracking each `gripper_base_1`
point and compensating its horizontal displacement with the bases. The
executable exits after all arms reach the target; only then is the payload
spawned.

## Build and run

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to coop_transport_bt cooperative_transport_bringup
source install/setup.bash

# Start the existing simulation plus Action server and example BT.
ros2 launch cooperative_transport_bringup cooperative_transport_bt.launch.py \
  start_simulation:=true
```

For an already-running simulator, omit `start_simulation:=true` and reduce
`bt_start_delay` if appropriate. The first physical trial should use the
existing low velocity limits in `coop_transport_controller/config/controller.yaml`.

## Phase-0 integration facts

The workspace uses ROS 2 Humble and BehaviorTree.CPP 3.8.7. The coordinator's
base outputs are `/amir1/rover_twist` and `/amir2/rover_twist`; it receives
`/amir1/odom` and `/amir2/odom`. `world` is its configured world frame, and
the AMIR configuration uses `base_footprint` as its base frame.

There is not yet a confirmed Gazebo-to-ROS TF for the physical payload model.
Consequently the Action server uses `/cooperative_transport/payload_pose` when
available and otherwise the midpoint/axis estimate derived from both odometry
messages. For friction-grasp evaluation, connect a measured payload pose to
the existing `/cooperative_transport/measured_payload_pose` path and enable
the coordinator's measured-payload parameters before treating the results as
physical-object ground truth.
