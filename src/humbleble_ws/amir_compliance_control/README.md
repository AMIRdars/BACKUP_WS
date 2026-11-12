# 摩擦把持後のARM_ONLY・リーダー台車0.5m（2026-11-10）

公開launchは既定で `arm_only_trial:=native_stroke` を選ぶ。旧CP13の
ネイティブ摩擦把持準備（配置調整、home、閉じ、監視引き継ぎ）を使い、
把持後はフォロワ台車の最終速度入力を単一writerからゼロに保つ。
通常の床で台車へゼロ指令を出すだけでは、長ストローク中に1mmの受動滑りを
検出した。このため既定のnative_strokeは、把持完了後にGazebo試験用の
**ベース速度拘束治具（kinematic velocity clamp）**を有効にする。
アーム関節・車輪の構造を変えず、対象はフォロワの基部だけで、把持後にモデル位置やアーム関節をリセットしない。搬送物との
把持は摩擦接触のままである。基部の積分ずれは、把持後の姿勢誤差/dtを
root速度へ入れて戻す。これは外部の理想拘束で、ROSのbase_cmdは全周期ゼロ。これは床上の通常ブレーキ性能の実証とは別条件。

旧準備の11cm配置調整はSetEntityPoseによる初期配置処理であり、
走行制御とは区別する。把持後にこのRPCを使わない。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true \
  forward_distance_m:=1.0 translation_velocity_m_s:=0.1 record_data:=true
```

上記の旧搬送引数1m/0.1m/sは把持後のARM_ONLYには使用しない。
把持後は `leader_base_distance_m:=0.5 arm_only_speed_m_s:=0.005` が既定。
同一モデルの完了した2試験の生odom/評価GTから距離換算を校正し、
リーダー内部の自己odom推定だけに適用する。生odomのROS topicは変更せず、
フォロワへ係数やGTを渡さない。URDF/controller/world/fixture証明の一致を検証し、
実GTの移動量も別に評価する。速度・加速度profileを換算し、raw base_cmdの
既存上限0.005m/sを維持する。校正は同速度条件のnative前進に限定。
他速度の未補正試験には `calibrate_leader_distance:=false` を指定する。
リーダーは自己odom/FKからアームを台車移動の半分だけ引き、
フォロワは自己F/T→xyアドミッタンス→FK誤差→DLSで追従する。
**台車0.5mに対し、物体移動と各アーム姿勢変化は約0.25m**。
フォロワへリーダー情報を送らない。ベース再中心化・回転は無効。

両アームのposition-servo応答を把持前に実測し、証明とJTCログを保存する。
既存URDF、controller/API、ネイティブ接触の物理パラメータは維持する。
長ストローク用の可動域は実URDFの連続IK分岐を先に確認する。

ROS 2 Humbleの既存JTC 2.53.1に、並行動作するcontroller間で関節値が
混入する共有作業配列が見つかった。同じ版・APIのまま配列を呼び出し毎に
独立させた修正版を `src/vendor/joint_trajectory_controller` に置く。
`colcon build --packages-select joint_trajectory_controller --allow-overriding joint_trajectory_controller`
後に `source install/setup.bash` を実行する。システム側のパッケージは上書きしない。
native試験は修正版が未導入なら開始せず、実際にGazeboがロードしたライブラリを
`trial/controller_isolation.json` に保存する。

native実験の安全ゲートは、100 Hz反力と250 Hz関節状態の非同期到着に対し、
短い時刻履歴から同時期の観測が存在することを確認する。
source age 50 ms・相互skew 20 msを維持する。nativeだけは
simulation_source_watchdogを有効にし、センサの鮮度を進行中のGazebo時刻で
検査する。時計と安全bus/adapter/permitの実時間150ms監視は維持する。
低いreal-time factorで、12msのセンサ周期に実時間160msかかっただけの
誤停止を避ける。時計停止、source欠測、非有限値、逆行時刻は停止する。
Bool接触はsource stampを持たず、既存の受信時sim時刻の年齢で監視する。
最新の実関節角/速度と各反力は従来どおり検査し、反力超過は受信時にも停止する。
欠測を履歴で無期限に補間しない。旧経路はこの選択を既定falseに保つ。

F/Tには既存 `friction_transport.yaml` の50N/100N/15Nmと0.1秒継続判定、
各指8〜26N、URDF由来の関節反力監視を使う。固定拘束微動の0.5N/0.05Nm
とは別の条件であり、どちらも実機の荷重定格を校正した値ではない。
今回の経路は明示的なシミュレーション実験専用。

native実験だけ、指力が既存8〜26N内で左右に偏ったときは追い締めを保留し、
保持中の角度目標を維持する。8/26Nと接触の独立監視は維持する。
旧経路の保守的な停止方針は既定値falseで残す。

終了後の把持は、停止でFAULTにラッチするattached状態だけで判断せず、
新鮮な両指接触・実測把持力・グリッパACTIVEで検証する。
nativeのGUIは停止・実接触を確認した後、既定で物理を一時停止して最終姿勢を表示する。
これは長時間の動的把持耐久の証明とは区別する。実Gazebo statsのpaused=trueと
停止したclockを確認し、paused_review_proof.jsonを保存する。
ライブ監視は `gui_review_mode:=live` で選べる。長時間GUI監視に未解決の
grasp missing判定があるため、失敗時のmonitor内容と受信ageを保存する。
Ctrl+CまたはGUIを閉じると終了する。
自動終了には `headless:=true keep_gui_open:=false start_delay_s:=0`。

従来の固定拘束9.3mm試験は `arm_only_trial:=micro` で維持する。

# Local compliance processing and simulation safety gates

The control nodes depend on ROS messages, rclpy and numpy only. They have no
import or package dependency on the leader, simulator, experiment tools, or
the existing follower's evaluation infrastructure. The wrench preprocessor
issues no motion commands. The separately launched safety gates forward
approved local commands and stop both robots through an independent supervisor.

## One-command ARM_ONLY simulation trial

From this existing workspace, after sourcing ROS 2 Humble and `install/setup.bash`:

```bash
ros2 launch amir_compliance_control arm_only_af_trial.launch.py
```

To display the same simulation in the Gazebo GUI:

```bash
ros2 launch amir_compliance_control arm_only_af_trial.launch.py gui:=true
```

The default `gui:=false` preserves headless operation. GUI mode uses the same
Gazebo server, robots, grasp, controller settings and automatic trial. The GUI
closes when the trial finishes; Ctrl+C stops/holds and cleans up both GUI and
server. Run the command from the graphical desktop terminal (DISPLAY or
WAYLAND_DISPLAY and its display permissions are inherited).

This reproduces the AF-2 command with fixed-constraint grasp, 3 mm/s, 9 mm,
virtual M/D/K=40/80/5, 80 ms servo trajectory horizon, F/T-driven updates,
8 Hz filtering, measured servo response and 0.0012 m/s² leader reference
acceleration. It runs the existing evaluator to start Gazebo,
prepare/home/grasp/tare, move only the leader, record CSVs, stop/hold both
robots, and clean up. It does not launch a second controller stack.

The output defaults to `audits/arm_only_rerun_<time>_<unique id>`; `result.json`,
`timeseries.csv`, `wrench_samples.csv`, `control_samples.csv` and node logs
are saved there. A completed trial exits the launch automatically. Ctrl+C
requests a stop and allows the evaluator to verify hold and clean up.

Optional arguments are `ros_domain_id:=70`, `output_directory:=<new path>`
and `workspace:=<workspace root>` (normally detected through the installed
launch symlink). A unique Gazebo partition and localhost-only ROS environment
are set on the trial child. The existing workspace tools, CP6 fixture and
`audits/arm_only_speed_20261109/servo_response.json` remain required.

The existing `arm_only_af.launch.py` is the control-only launch used inside
the evaluator; it is unchanged.

`local_grip_maintenance` is a separately enabled, simulation-only gripper
action client. It uses its own `native_grasp_monitor` measurements and shared
boolean permit/stop signals. No leader state, trajectory, TF or ground truth
enters this node. Preparation must retire before enabling it, and every
runtime participant must finish unarmed discovery before the first permit.
The existing 17--23 N preparation operating band and .0002 rad correction
are numerical trial settings. Goals advance only toward closure, by at most
.0002 rad per .5 simulated second relative to both the measured position and
last goal. Unachieved goals do not accumulate. Invalid measurements, lost
contact, force/joint limits, action rejection or permit loss latch a stop;
the gripper retains its last bounded goal. Independent grasp monitoring keeps
the original 8/26 N limits and watches the enabled keeper's .15 s heartbeat.
During the monitor's original .3 s low-force/contact-loss confirmation,
maintenance withholds corrections; it neither regrips nor bypasses the stop.
The handoff's measured grip position seeds the first held reference. A
correction also requires a sustained .3 simulated-second operating-band
deficit, so brief force fluctuations do not excite the whole-body loop.
These values and the inherited action `max_effort=1.2` are not hardware ratings;
the installed position-interface adapter forwards positions and does not
implement a force cap from `max_effort`.

The public `compliance_contact_transport.launch.py` accepts the explicit
`grip_maintenance:=true` option. Its default remains false. This does not
raise the launch's distance/speed envelope or qualify the 1 m/s trial.

`wrench_preprocessor` requires `use_sim_time=true`, an explicit robot namespace,
`urdf_file`, and a matching measured six-axis `calibration_file` generated by
the CP-2 probe. Invalid or mismatched calibration fails at startup. No measured
force limit is implied by a sensor calibration certificate.

Inputs: local `ft_sensor`, `joint_states`, encoder
`mecanum_drive_controller/odometry`, and `/clock`. The node rejects remaps outside
these inputs. It computes FK from joints interpolated at the sensor stamp;
it does not subscribe to TF. Raw shifted wrench is published before bias
subtraction on `wrench_at_grasp_raw`. `tare_wrench` requires a window of fresh
stationary samples. `wrench_filtered` starts only after that explicit tare.

Both output wrenches are expressed in `<robot>/grasp_aligned`, whose origin is
the model TCP and whose axes are parallel to `base_footprint`. Its TF is
published locally at the same sensor stamp. This avoids labeling a moment
about the TCP as though it were about the base origin. The model TCP has not
yet been certified as the actual payload contact center.

LPF cutoff 8 Hz, 100 bias samples, age .05 s, skew .02 s, wall timeout .5 s,
and stationary velocity .001 are explicit initial sensor-processing settings,
not validated load limits. Missing, nonfinite, stale, reordered or out-of-limit
joint/sensor data invalidates the bias and suppresses filtered output. Sensor
recovery requires another explicit tare. This is a diagnostic pipeline;
grasp validation remains a separate fixture dependency.

## Simulation command interlock

`safety.launch.py run_directory:=<absolute measured fixture directory>` starts
`/<robot>/safety_gate` for leader and follower, and `/safety_supervisor`.
Each gate requires `use_sim_time=true`, its generated `urdf_file`, and a
matching `profile_file`. Generate profiles with the evaluation-only
`tools/amir_make_safety_profiles.py` after measuring all five joint reaction
sensors in that exact scene. URDF and controller-file hashes are checked;
the experiment runner also checks the world hash.

Gate inputs are local `joint_states`, encoder odometry, `ft_sensor`,
`joint_load/Joint_1` through `Joint_4`, `grasp/state`, `base_cmd` (TwistStamped),
`arm_joint_cmd` (complete positions-only JointTrajectory), `/clock`, and two
shared boolean stop/permit topics. Remaps outside this boundary are rejected.
No leader state, goals, TF, or evaluation data enters the follower gate.
The supervisor receives readiness/fault/sequence heartbeats, not robot poses.

Commands use `<robot>/base_footprint` and fresh stamps after arming. Gate outputs
are `rover_twist` and `arm_controller/joint_trajectory`. The existing relay is
the sole publisher to `mecanum_drive_controller/reference_unstamped`.
Exclusive output ownership is checked before arming. This check is not a ROS
security boundary: direct JTC action calls and new publishers after arming
must remain excluded by bringup.

The steady-clock gate and supervisor run at 50 Hz. Initial policies are
sensor age .05 simulation seconds, skew .02 seconds, command and heartbeat
timeouts .15 seconds (wall time also checked), and .01 rad software margin.
These are explicit simulation policies, not certified hardware deadlines or
manufacturer margins. Base velocity envelopes come from passing CP-1 tests.
Raw joint-axis reaction torque is checked against the selected URDF effort
limits before filtering. This does not establish structural, bearing, wrist
force, or gripper load limits. No authoritative hardware load limits exist
in this experiment; profiles are explicitly simulation-only.

Missing/stale/nonfinite sensors, clock stall/reversal, grasp loss, joint
position/velocity limits, reaction overload, stale commands, or shared permit
loss latch a stop. Both base commands become zero and arms hold the last
valid measured positions. Grippers are not opened. Recovery requires fresh
stationary state, explicit local `/<robot>/safety/reset`, shared
`/transport/safety/reset`, and `/transport/safety/arm` Trigger calls. Check
`/transport/safety_status` for `armed=true` after the arm handshake; service
acceptance alone is not an armed state. `/transport/safety/stop` latches a
shared stop. Old commands are discarded during rearming.

This is not full CP-7 completion. Gate-process death, downstream controller
death, complete action preemption, hardware emergency stops, payload attitude
failure, and certified force/grip thresholds still require work. The existing
mecanum controller timeout is unchanged at 1 simulation second. CP-3 tests
use fixed arms and comparison-only independent position references. CP-4 adds
leader-only goals below; autonomous force following and arm/base bandwidth
separation are not implemented.

## CP-4 leader-only goals

`leader_only.launch.py run_directory:=<measured fixture directory>` starts
`/leader/leader_trajectory_node` and `/follower/follower_hold`. This launch has
no simulator, evaluation bridge, or comparison runner. The safety launch must
also be running. It introduces no package dependencies on those components.

Before arming, call each `/<robot>/transport/prepare` Trigger service. This
requires fresh local state and exactly one publisher with the expected
namespace and name for encoder odometry and joint states. It also checks sole
ownership of each local command-request topic. Middleware discovery queries
run in this explicit disarmed preparation step. These are startup checks,
not authenticated identities or continuous enforcement against new publishers.

After shared safety arming, each endpoint independently sends one three-second
home request using initial joint positions from its own generated URDF. Base
requests remain zero. The fixture must remain supported and unattached during
this preparation. `READY` requires local home convergence. This local homing
does not use a leader reference. It is not a transport trajectory for the
follower. The evaluator attaches the fixture only after both endpoints report
`READY`.

The sole transport goal input is **`/leader/transport_goal`**, type
`geometry_msgs/msg/Vector3Stamped`. This is a new package API: `vector.x` is a
signed relative displacement in metres along the leader base x axis captured
at goal acceptance; `vector.y` and `vector.z` must be zero. `header.frame_id`
must be `leader/base_footprint`, and the simulation timestamp must be fresh
and newer than the last accepted goal. The local odometry origin and yaw are
captured once; no TF lookup is used. Default limits are distance .001 m,
speed .00025 m/s, acceleration .001 m/s², gain 1/s and encoder tolerance
.000002 m. These are initial simulation policies based on CP-3's tested
envelope, not tuned hardware specifications. The attached CP-4 experiment
explicitly reduces distance and speed to .000025 m and .000025 m/s.

Only the leader subscribes to a goal. It follows a triangular/trapezoidal
local reference using its own encoder and publishes through `base_cmd` to the
safety gate. Busy, stale, nonfinite, wrong-frame, lateral, or over-envelope
goals are rejected without replacing an active goal. After reaching the
encoder tolerance, state returns from `TRANSPORT` to `READY` and output is
zero. Encoder arrival is not ground-truth payload arrival.

The follower issues **zero base requests only**. It consumes its own encoder,
joints, local gate heartbeat and grasp state, plus `/clock` and shared
boolean safety permit/stop. It has no subscription to goals, leader topics,
TF, global transport state, or GT. It is a stationary CP-4 endpoint, not the
future force-admittance follower. Its state remains `READY` during leader
motion; it does not need to know that a leader transport goal exists.

Both publish JSON on `/<robot>/transport/state`, with `SAFE_STOP`, `READY` or
`TRANSPORT`, local readiness, home convergence and goal counters. Startup is
`SAFE_STOP` pending explicit prepare/arm. After an armed fault, `SAFE_STOP`
latches, drops the goal and requests a shared stop. Sensor/clock/permit loss
cannot auto-resume the goal. Recovery requires gate and shared safety reset,
then local `transport/reset` while stopped/disarmed, new `transport/prepare`,
and explicit shared arm. New goals are required. A successful service response
does not replace checking the published state.

CP-4 validates information boundaries and lifecycle only. It does not establish
continued cooperative transport, force compliance, safe absolute wrist loads,
or complete CP-7 process-failure coverage. Do not enlarge the attached test's
distance or speed before CP-5/6 validation.

## CP-5 ARM_ONLY admittance

`arm_only.launch.py run_directory:=<measured directory>
calibration_file:=<matching six-axis certificate>` replaces `follower_hold`
with `follower_arm_admittance` and starts the local wrench preprocessor. It
does not launch the simulator, apply external forces, or grasp a payload.
Never start this with `leader_only.launch.py`, which would duplicate command
publishers. The same shared safety launch is required.

The preparation/home sequence remains local and explicit. The arm endpoint
also verifies the preprocessor as the sole publisher of `wrench_filtered`
and `wrench_status` during disarmed preparation. After home convergence,
explicitly call `/follower/tare_wrench` while stationary, then
`/follower/admittance/enable`. The latter requires a fresh READY filter,
local state and permission. It captures the measured TCP as the zero-offset
reference; enabling does not move to an assumed origin.

Only the follower arm moves. Both base requests stay zero. The follower has
the CP-4 local inputs plus local filtered wrench/status; no goal, leader,
TF, ground truth or experiment-force topic enters the controller. The
preprocessor uses its measured sign, interpolated joints and TCP moment
transfer, explicit stationary tare, and 8 Hz LPF. An external-force generator
exists only in the opt-in CP-5 experiment scene and evaluator.

`admittance.py` integrates independent x/y virtual second-order systems with
implicit Euler substeps, velocity/acceleration bounds and a hard displacement
limit. The virtual state does not integrate outside the boundary; outward
velocity is reset at saturation. The hard bound takes priority at that event.
Initial **virtual**, not physical, settings per axis are mass 1 kg, damping
40 Ns/m, stiffness 400 N/m, displacement .0005 m, velocity .001 m/s and
acceleration .02 m/s². They are declared simulation tuning parameters, not
manufacturer ratings or inferred payload masses. No new force safety limit
is claimed.

The analytic geometric TCP Jacobian is checked against finite differences.
Position-only xyz DLS maintains the captured height and uses .001 m/rad
damping, a .001 m/rad minimum singular value, 1e-7 m convergence tolerance,
and the existing .01 rad joint margin. Joint requests are limited to .03 rad/s
and sent as complete positions-only trajectories through the safety gate
with a .04 s horizon. These are explicit software policies.

Orientation is free in CP-5. The audited home has xyz rank 3 but xyz+yaw rank
3; simultaneously imposing all four would be incorrect. This implementation
does not certify arm compliance while grasping an object or maintaining a
fixed gripper attitude. Experiments use the ungrasped CP-5 scene; attitude
changes and base motion are recorded separately. Successful local joint FK
displacement is not an independent world-frame TCP measurement.

On filtered-data age >.05 simulation seconds, clock/local-state/permission
loss, invalid filter bias, nonfinite wrench, frame/reordering errors, loss of
IK rank, or unreachable bounded IK, the endpoint latches SAFE_STOP, clears
the virtual state and requests shared stop. The raw joint-reaction gate
continues independently. Filter recovery, re-tare and shared rearm do not
re-enable the old arm offset. Explicit transport reset/prepare, safety arm,
fresh tare and a new `admittance/enable` are needed. Debug information is
published locally on `/follower/admittance/debug`.

This completes the limited CP-5 arm-only experiment. Its default settings and
stationary base behavior are preserved by the separate CP-6 mode below.

## CP-6 ARM_BASE local recentering

`arm_base.launch.py run_directory:=<measured directory>
calibration_file:=<matching six-axis certificate>` starts the leader endpoint,
`/follower/follower_arm_base` and the local preprocessor. Start the shared safety
launch separately. Use one transport launch at a time. This package does not
depend on the simulator, experiment input or ground-truth evaluator.

After source/ownership preparation, explicit safety arm and local home, confirm
both grasps and remove the experimental support. Tare while stationary, then
explicitly enable `/follower/admittance/enable`. The enable service captures the
measured local TCP and gripper rotation and verifies the straight-axis pose IK
envelope before enabling base recentering. Only `/leader/transport_goal` accepts
a transport goal. The follower subscriptions are the same local inputs as CP-5,
including its own wheel odometry and shared boolean safety permission/stop.
There are no leader-state, goal, TF or ground-truth subscriptions.

The arm virtual dynamics use mass `[20,20]`, damping `[1200,1200]` and stiffness
`[400,400]` in SI units. These are simulation tuning values, not physical masses
or verified safe load limits. CP-5 keeps its original mass/damping defaults.
The .5 mm displacement, 1 mm/s arm virtual speed, .02 m/s² virtual acceleration,
.03 rad/s joint request limit and .04 s trajectory horizon remain unchanged.

The base uses **measured joint FK deflection**, not the commanded virtual offset
or a force-to-base shortcut. The error consists of local TCP x/y displacement
and the z component of the relative SO(3) rotation log. A .2 Hz first-order LPF
and gains `[.5,.5,1]` produce task velocity. The inverse planar base Jacobian
includes the measured TCP lever arm: `vx=ux+p_y*wz`, `vy=uy-p_x*wz`. Base speed
limits are `[.0005,.0005,.001]` (m/s, m/s, rad/s) and acceleration limits are
`[.001,.001,.002]` (m/s², m/s², rad/s²). These are explicit software policies.
The base responds to persistent error while the arm absorbs the initial error.
No new data for .05 wall seconds causes zero base request; inherited local
sensor, permission, filter and IK faults latch SAFE_STOP and request both stops.
Local diagnostics are `/follower/recentering/debug` and `admittance/debug`.

Normal grasped transport is restricted to straight x. The pose DLS verifies
both position and rotation residuals and rejects unreachable targets; five arm
joints are never assumed to provide six independent freedoms. The orientation
uses a rotation log because Euler yaw is singular at the audited home. The
`.1 m` angular task weighting is a numerical metric, not a robot dimension.
Lateral/yaw sign checks use `ungrasped_test:=true`, free arm orientation and
explicitly generated single-base test safety profiles. They do not certify
grasped lateral/yaw transport. Restore normal grasp-required profiles for carry.

The isolated CP-6 fixture uses .52 kg, an opt-in DART weld CFM `.1` and fixed
constraint attachment. This is numerical compliance, not measured physical
gripper stiffness or contact gripping. Existing scenes keep their parameters.
Validated CP-6 carry is a repeated 1 mm relative goal at .05 mm/s. Guarded
execution and delay evaluation follow below. General whole-body allocation and
certified hardware load/grip limits remain subsequent checkpoints.

## CP-7 guarded execution and delay evaluation

Use `guarded_safety.launch.py run_directory:=<measured directory>` together with
`guarded_transport.launch.py run_directory:=<same directory>
calibration_file:=<matching certificate> mode:=ARM_BASE delay_s:=0`. These replace
the earlier safety/transport launches for new guarded experiments. They do not
launch Gazebo or ground-truth bridges. Keep exactly one safety/transport set.

The local safety gate now publishes candidate outputs to `interlock/base_request`
and `interlock/arm_request`. A separate `actuator_interlock` exclusively publishes
the real base/arm topics. It checks local gate/state/clock/relay/permit freshness
using a steady wall timer, publishes zero base and a measured arm hold on faults,
and retains the last valid pose if incoming joints are invalid. `controller_audit`
runs discovery and asynchronous controller queries in another process; its RPC
wait is bounded and a lost reply is retried. It checks controller active states,
sole writers, unexpected stamped base publishers and external JTC action status.

The shared supervisor requires both gates and both final interlocks. A fault
also requests action cancellation and deactivates both robots' mecanum and arm
controllers through independent manager services. It never activates controllers
or opens a gripper. A dead final publisher is checked through controller state
and measured wheel joints, since it cannot publish a new zero. The action server
can initially accept an external goal; detection/stopping is a functional guard,
not ROS access control. Use an isolated experiment domain.

Home starts after the local gate's own arm epoch. Guarded transport additionally
requires the local final actuator acknowledgement. Live trajectories wait for
gate active status rather than silently disappearing if status arrives later.
Measured convergence acknowledges home; bounded fresh requests recover lost
initial messages, reference the original three-second end, use a minimum .1 s
horizon, and stop if home has not converged in five seconds. Stop/reset/rearm
discard pending requests. No leader state or TF is added to follower inputs.

Only the guarded launch enables local pose monitoring: XY within the existing
.5 mm displacement envelope (+IK tolerance), height error .1 mm and rotation-log
norm .01 rad. These are software experiment limits, not physical safety ratings.
Raw model-derived torque limits and existing freshness/velocity checks remain.

`mode:=ARM_ONLY` in this launch is the same pose-preserving virtual controller
with base gains set to zero, for a controlled comparison. Its executable/debug
role remains ARM_BASE. It differs from the original CP-5 ungrasped ARM_ONLY,
whose free orientation and default M=1,D=40 behavior remain available.

`command_delay_s` defaults to zero. Test delay requires an explicit local call
to `transport/enable_test_delay` after preparation; home is not delayed. Original
message stamps are retained. Expired requests, including a trajectory whose
40 ms horizon ended before receipt, are rejected. Stop output bypasses the delay
and clears the queue. This does not promise successful following above 40 ms.
Guarded processes limit native numerical computation to one thread; global
environment and earlier launch defaults are preserved.

Recovery is explicit: restore data/processes, keep disarmed, reactivate the
controllers manually, reset each local gate/audit/interlock, reset shared safety
and local transport, prepare/arm, re-tare/re-enable and give a fresh leader goal.
Cold process restarts reset sequence counters and require a consistent restart
of the related control set. No old goal is replayed. Hardware load/grip ratings,
contact gripping and physical emergency stops remain unverified.

## CP-8 opt-in whole body allocation (simulation only)

`whole_body_transport.launch.py` selects `WHOLE_BODY` (default) or the
preserved `ARM_BASE` comparison. Start the measured fixture and
`guarded_safety.launch.py` separately. No evaluator, Gazebo process, world
pose or TF subscriber is launched by the control launch file.

The launch requires an explicit `yaw_profile:=initial|stiff|filtered`;
there is no implicit profile selection. The final CP-8 comparison uses
`filtered`. Initial yaw virtual inertia/damping/stiffness are 0.2/12/4;
stiff and filtered use 20/1200/400. These are virtual controller parameters.
Only filtered adds the 0.2 Hz filter on the allocated base velocity.

The CP-8 leader is `leader_planar_trajectory`. It accepts **only on the
leader** `/leader/transport_pose_goal` (`geometry_msgs/msg/PoseStamped`):

- `header.frame_id: leader/base_footprint`, with a fresh simulation stamp.
- Position x/y: relative metres in the leader's axes at goal acceptance;
  position z must be zero. Quaternion: normalized yaw rotation; roll/pitch
  must be zero. Translation <= 1 mm, yaw <= 1 mrad and the combined software
  metric `norm([x,y,0.1*yaw]) <= 1 mm`.
- The path interpolates local encoder x/y/yaw; actuator twist is converted
  from the start axes to the current leader body axes. The 0.1 m metric is
  a numerical task weight, not a robot dimension or load rating.
- Goals require measured local readiness, home completion, confirmed grasp
  and explicit shared safety arm. A stop clears the reference. Reset requires
  all three planar encoder velocity components to be below the stop policy.

The original `leader_trajectory_node` keeps its `Vector3Stamped`
`/leader/transport_goal` API and signed straight-x behavior.

`follower_whole_body` receives only its own F/T preprocessing, joint state,
encoder, grasp and local safety/interlock status, plus boolean shared safety
permission/emergency stop and clock. It never receives the leader's goal,
state, frame or rotation centre. It uses the full local TCP wrench (already
transferred from the sensor origin) for planar force/yaw compliance. The
actual URDF supplies both the TCP and gripper anchor geometry.

The 6x8 geometric Jacobian orders columns as Joint_1..5, body vx/vy/wz and
rows as local linear xyz/angular xyz. Angular rows have a numerical 0.1 m
metric. A bounded weighted least-squares active-set solve enforces software
joint margin, URDF joint velocity bounds, stricter request speed, request
acceleration and base speed/acceleration. Joint weights increase near their
limits and as the arm's smallest metric singular value decreases. The whole
Jacobian must retain task rank. The solver checks KKT residuals, rather than
clipping a coupled unconstrained inverse.

Fast compliance is handled by the arm. The 0.2 Hz low-pass of **measured**
local arm deflection grants slow base motion authority; a force sample alone
cannot move the base. Braking remains acceleration bounded when that
permission shrinks. An unavailable instantaneous arm direction may retain a
pose/twist residual until the base is allowed to contribute. The filtered
profile also filters the candidate base velocity within the hard bounds,
then solves again with the actual base velocity fixed so the arm allocation
matches the emitted base command. Debug outputs
are `/follower/allocation/debug`, `admittance/debug` and `recentering/debug`;
none is consumed by the other robot's controller.

All new virtual mechanics, numerical weights and pose/motion limits are
software policies for the measured simulation fixture. They do not certify
hardware inertia, load capacity or grip. `allow_ungrasped_test:=true` is an
explicit standalone experiment option and also needs the measured ungrasped
safety profile. Normal transport requires confirmed grasp on both robots.
The CP-7 freshness checks, 40 ms arm trajectory horizon, final exclusive
writer and independent dual-controller deactivation remain active. A healthy
active final writer forwards an accepted arm request in its callback, with
the same expiry check; pre-ack one-shot commands stay pending until safe.

Final measured cooperative forward (+1 mm), backward (-0.5 mm) and lateral
(+0.1 mm) trials passed the 12-second settling criterion. Cooperative yaw
and rotation about the specified leader gripper anchor overrotated and did
not settle; these motions remain unaccepted. Passing standalone signed yaw
tests does not establish cooperative yaw or arbitrary rotation centres.
The completed CP-8 comparison records both improvement and failure conditions.
Hardware transport remains disabled pending load/grip/sensor/emergency-stop
evidence and the separate CP-9 assessment.

Reproduce actual-model checks and isolated physics experiments with
`tools/amir_cp8_model_audit.py`, `amir_cp8_standalone.py`,
`amir_cp8_trial.py` and `amir_cp8_batch.py`. Evaluation/GT imports stay in
these external tools. `amir_cp8_analyze.py` only reads saved evidence.
See `audits/whole_body_compliance_20261019/cp8/README.md` for outcomes,
limitations, exact commands and the gate for the hardware assessment phase.
# CP-0〜CP-4: ARM_ONLY微小直進とGUI確認（2026-11-09）

`arm_only_trial:=micro` の経路は、リーダーだけが
9.3 mm前進する固定拘束把持の微小試験に変更した。フォロワは自己F/T、
自己関節/FK、自己odomだけを使用する。ベース再中心化・WHOLE_BODY・
回転追従はこの経路に含まない。既存のnamespace、URDF、position-only
JTC、Gazebo Fortressを維持する。

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  control_mode:=ARM_ONLY arm_only_trial:=micro headless:=false gui_nvidia_offload:=true \
  arm_only_distance_m:=0.0093 arm_only_direction:=forward \
  translation_velocity_m_s:=0.005 record_data:=true \
  start_delay_s:=10 keep_gui_open:=true
```

把持・tare・FK基準ラッチ後、10秒静止してから一度だけ目標を発行する。
完了後は安全停止と把持保持を確認して画面を残す。Ctrl+CまたはGUIを閉じて終了する。
`trial/held_result.json` と `trial/gui_hold_status.json` はGUI保持中の結果、
`trial/result.json` はプロセス終了・cleanup確認後の最終結果。
保持中は新しい目標を発行せず、把持・実関節・車輪・実位置を監視する。
`keep_gui_open:=false start_delay_s:=0` で通常の自動終了試験になる。

従来の `forward_distance_m:=1.0 translation_velocity_m_s:=0.1` は
ARM_ONLYの微小試験条件ではない。速度を黙って切り詰めず、起動前に拒否する。
前回の0.5 m後退も初期姿勢からの可動域上限を超えるため拒否する。
旧CP13経路は明示 `control_mode:=WHOLE_BODY` で分離されるが、現在は
既存の証明ハッシュ不一致によりfail-closed。ハッシュを書換えて迂回しない。

試験は既存の保護境界0.5 N/0.05 Nmと実測アーム偏差10 mmを維持する。
これらは微動シミュレーションの実験値であり、許容荷重の校正値ではない。
実機・摩擦把持・自律運用の許可を意味しない。CP5以降は今回有効化しない。

実最終台車入力は
`/follower/mecanum_drive_controller/reference_unstamped`（Twist）、
アーム入力は `/follower/arm_controller/joint_trajectory`（JointTrajectory）。
CSVには前段base_cmd、interlock出力、最終台車入力と実odomを別々に残す。
制御フレームは既存 `follower/base_footprint`。仕様のbase_linkとの固定yaw変換を
URDFから含めており、CSVのframe表記を変更して偽装しない。

CP4の追加検証入力（joint_margin/singularity/base_drift/dt）は
`simulation_fault_injection_enabled=true` の試験時だけサービスを公開する。
検査用コピーへ異常を与え、実joint_states/odomや幾何形状を変更しない。
通常ARM_ONLYにはこの試験APIは存在しない。

グラフ出力:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/amir_p0_evidence.py --case <出力先>/trial
# GUIを開いたまま評価する場合は --held を追加
```


Native runnerのnative_cpu_policy:=smtはLinux topologyで観測したSMT群を
runnerと子processだけに適用する。見つからなければ元の許可CPUを使い、
他アプリやwatchdog閾値は変更しない。従来の割り当てはnative_cpu_policy:=inherit。
trial/cpu_affinity.jsonに実測topologyと前後のCPU集合を保存する。

距離補正の最終係数は、3回の安全な搬送完了区間から得た換算比の最小値から、
観測した比の範囲をさらに差し引く経験的な余裕を用いる。係数と算出方法を
JSONへ保存し、再計算して検証する。統計的な保証区間ではなく、実GTが指定距離
以上かを毎回オフライン実測判定する。距離判定に未達の試験も計測として使えるが、
把持・停止・その他の検証に失敗した試験は校正へ採用しない。
