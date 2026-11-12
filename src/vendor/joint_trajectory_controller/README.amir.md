# AMIR: controller instance isolation

This is upstream `joint_trajectory_controller` **2.53.1**, the same version as
the installed ROS 2 Humble package. Source and archive digest are recorded in
`AMIR_PATCH.json`; upstream copyright and Apache-2.0 licensing are retained.

The only runtime change replaces the function-static scratch vector in
`sort_to_local_joint_order()` with a call-local vector. Two controller managers
in the same Gazebo server update concurrently. Their controller instances must
not share mutable remapping storage. The old DSO fails
`JointOrderIsolation.ConcurrentRobotCommandsStayLocal`; the patched DSO passes.
Controller type, public headers, joint names, topics, parameters and hardware
interfaces are unchanged. No gain or model change is included.

Build in the existing workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select joint_trajectory_controller \
  --allow-overriding joint_trajectory_controller --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select joint_trajectory_controller
colcon test-result --test-result-base build/joint_trajectory_controller
```

The native ARM_ONLY trial requires this overlay marker and records the selected
library SHA-256 and the Gazebo server's actual loaded library from `/proc`.
The system package under `/opt/ros/humble` is unchanged. This preserves a clear
negative-control binary for reproducing the race.

Test-only adaptation: the upstream plugin-load test now uses an empty resource
manager, since it only loads a controller and never configures or claims joints.
The installed test-assets URDF refers to absent `test_actuator` / `test_sensor`
aliases. No AMIR model or runtime hardware plugin is substituted.
