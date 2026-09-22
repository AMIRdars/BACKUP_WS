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

The default tree begins with `REPOSITION_ARMS` (`motion_type="5"`). It sends
the requested Joint_1..Joint_5 targets to both AMIR arm controllers while the
Action server tracks each `gripper_base_1` grasp point and commands the two
bases to compensate its horizontal displacement. Both grasp-state topics and
joint-state topics must be available before this step starts.

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
