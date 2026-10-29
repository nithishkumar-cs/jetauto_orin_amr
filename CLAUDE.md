# Claude Handoff — Clean-Slate Rebuild

This repo is a ROS 2 Humble AMR stack for JetAuto on Jetson Orin (4-wheel mecanum).
On 2026-06-21 it was deliberately reset to a clean slate to clear old-version junk
and confusion. We now rebuild nodes and topics one by one against the design
decisions below.

## Historical reset snapshot (2026-06-21)

The following bullets describe the original clean-slate reset, **not** the
current repository. For current measured milestones and remaining gates, use
`docs/engineering_log.md` and the linked validation reports.

- **Reset done.** Every ROS package under `src/` was nuked **except `robot_bringup`**.
- The layer folders are **kept but empty**: `bridges drivers localization navigation
  perception platform safety tools`. (Empty dirs are not tracked by git — add a
  `.gitkeep` per folder if you want the skeleton versioned.)
- **`robot_bringup` is kept but is BROKEN-ON-LAUNCH by design** — see *Deferred* below.
- **`amr_interfaces` rebuilt (DONE, standards-first).** Lives at
  `src/amr_interfaces`. Defines only the 2 custom contracts ROS has no
  standard for — `TrackedObstacle`, `TrackedObstacleArray`.
  Everything else uses standards: detection → `vision_msgs/Detection2DArray` +
  `Detection3DArray`; health → `diagnostic_msgs`; e-stop → `std_msgs/Bool`;
  odometry → `nav_msgs`. Builds + message generation verified (needs
  `ros-humble-vision-msgs` installed). `TrackedObstacle` has no per-obstacle
  header — frame/stamp come from the array header (base_link).
- **`system_modes.yaml` was moved** out of the package to
  `configs/runtime_modes/system_modes.yaml` (sibling of `runtime_modes.yaml`).

## Layer purpose

- `amr_interfaces` (top-level, `src/amr_interfaces`) — shared message contracts. Source-agnostic; no algorithms. The `platform` layer was dissolved 2026-10-14; a future TF/calibration package would also sit top-level.
- `perception` — camera → detection → depth-projection → tracking → fusion → `/perception/fused_obstacles`.
- `safety` — collision avoidance plus an operator stop. **Obstacle zones are `nav2_collision_monitor` (upstream), not our code**; `scan_health_gate` closes the Humble scan-loss gap; `estop_gate` is the final latching software operator-stop. Chain: `/cmd_vel` → collision monitor → `/cmd_vel/collision_limited` → stop-zone recovery (bounded reverse-only escape) → `/cmd_vel/recovery_limited` → scan-health gate → `/cmd_vel/scan_health_limited` → e-stop latch → `/cmd_vel/safety_limited`.
- `localization` — SLAM / localization & mapping.
- `navigation` — nav tasks + teleop relay.
- `drivers` — real hardware IO. Consumes `/cmd_vel/safety_limited`, publishes `/odom/wheel`.
- `bridges` — sim (Isaac) / replay adapters that present the *same* contracts as hardware.
- `tools` — diagnostics, benchmarks, evaluation.
- `orchestration` — launch orchestration only, no business logic. Holds `robot_bringup`.
- `rosbag/` (outside `src/`) — replay scripts; not a ROS package.

## Settled design decisions

- **Source-agnostic core.** Hardware/sim/bag detail only at the edges (`drivers` / `bridges` / `rosbag`). Core packages never know the data source.
- **`base_interface` retired.** Motion contract: safety → `/cmd_vel/safety_limited` → driver/bridge → `/odom/wheel`. Any shared base math (mecanum IK / odometry / watchdog) that ends up duplicated goes in a **library**, never a resurrected node.
- **Safety is independent of navigation.** Separate, simple, trustworthy last-resort. It subscribes directly to obstacle sources, NOT to the map/planner.
- **Safety is multi-input, worst-case combined** ("anything screaming danger wins"). Detection-only obstacles are fail-open (blind to untrained objects) — a class-agnostic geometric source (lidar/sonar) must be on the safety path; do not let a depth-camera `pointcloud` source replace it.
- **Use `nav2_collision_monitor` for obstacle zones; do not write our own.** *(Decided 2026-07-26, replacing the hand-written radial policy.)* It is maintained upstream and strictly better: arbitrary polygons instead of circles, `scan`/`pointcloud`/`range` sources, built-in `source_timeout`, `base_shift_correction`, `stop_pub_timeout`, and it publishes `nav2_msgs/CollisionMonitorState`. It runs standalone, so this does not couple safety to navigation. **Two traps:** it has *no e-stop input whatsoever* (only observation sources, `cmd_vel`, footprint), and it is a **lifecycle node** — launched alone it logs "Waiting on external lifecycle transitions" and gates nothing while still looking healthy in `ros2 node list`. It needs `nav2_lifecycle_manager` with `autostart: true`.
- **NOT a safety-rated system, and must never be described as one.** ISO 3691-4 requires functional-safety-rated area protection: a scanner certified to IEC 61496 Type 3 / SIL 2 / PL d Cat 3 whose OSSD outputs drive a safety relay or safety PLC that cuts motor power **in hardware**, with no computer in the path. Navigation data off that scanner is informational only. Everything we run is a supervisory software layer on Linux over DDS. JetAuto has no safety-rated hardware, so the honest label is "collision avoidance and an operator stop". Adding a real safety layer is a hardware purchase, not a software task.
- **Component tiers.** REQUIRED core (sensors / perception / safety / motion) is locked in production. OPTIONAL instrumentation (benchmarks, diagnostics, calibration_validator) is a free toggle. VARIANT selectors (teleop / localization / navigation, and the sensor suite) are chosen among *validated* combinations, not free toggles. A specific sensor (e.g. lidar) is required only if it's needed to meet the minimum sensing contract; otherwise it's a variant choice (camera-only vs camera+lidar), each with its own validated speed/stop envelope.
- **Modes** (kept: `debug` + `profile` + `production`). `debug`/`profile` have per-node toggles where `false` ⇒ synthetic substitution; `production` is locked (no toggles). Mode *behavior* (log/assert/timing/metrics) lives in `runtime_modes.yaml`. **Constraint: `safety` may be `false` only when `drivers` is `false`.**
- **Config homes.** Per-package `config/` = node default params; top-level `configs/` = deployment/robot config + mode config. Everything is read from the *installed* share path, so `--symlink-install` (or overriding the path launch-arg) is what makes edits live without a rebuild — location does not change that.
- **`src/<layer>/<pkg>/src` nesting is correct.** Outer `src` = colcon source space; inner `src` = ament C++ sources. colcon discovers packages by `package.xml` recursively, by name not path. **Every** package sits at `src/<layer>/<pkg>/` — `robot_bringup` was the last exception and moved to `src/orchestration/robot_bringup/` on 2026-07-26.
- **Package names must stand alone.** ROS package names are a flat global namespace, so the layer folder provides no scoping: `src/perception/detection_depth_projection/` holds a package *named* `perception_detection_depth_projection`. Do not name a package after its folder alone (`safety`, `geometry`) — and avoid stutter (`safety/safety_layer`), which is why the operator-stop component is `estop_gate` and the orchestrator layer is `orchestration`, not `bringup`.
- **Launch files live in `<pkg>/launch/`.** `launch_lib/` installs as a *sibling*, so `robot_stack.launch.py` puts its parent directory on `sys.path`, not its own.

## robot_bringup (historical July rebuild; subsequently extended)

Rewritten 2026-07-26. The description below is the initial rebuild state;
the current bringup now launches rebuilt components and upstream SLAM/Nav2
as described in `src/orchestration/robot_bringup/README.md`.

- **Done.** Dangling package references removed (`package.xml` exec_depends
  trimmed to what exists; `NODE_SPECS` in `actions.py` is the registry to grow).
  `system_modes.yaml` install fixed — it is no longer installed from the package,
  it arrives via the `configs/` → `project_configs/` install. `config.py` rewritten
  for the current `system_modes.yaml` schema (`default_mode` + flat `toggles`);
  the old code expected `defaults`/`config_sets` and could not have worked.
  Production lock + tiers and the safety↔drivers constraint are implemented in
  `config.py`, tiers declared in `launch_lib/tiers.py`.
- **Adding a package back:** add a `NODE_SPECS` entry in `actions.py` and an
  `exec_depend` in `package.xml`. Unregistered or unbuilt packages are reported
  and skipped, never fatal.
- **Current.** Disabled required perception stages in debug/profile publish
  empty arrays from input stamps. Disabled safety with no driver publishes zero
  commands. A disabled driver starts no actuator process. Variant selections
  reject unimplemented teleop, mismatched navigation toggles, hardware drivers
  without odometry localization, and camera-only perception. Production locks
  required components on.

## Pending design decisions (not yet made)

- **Build and instrumentation modes.** `scripts/dev_helpers/build_mode.sh` maps
  debug/profile/production to Debug/RelWithDebInfo/Release. Assertions follow
  the build type. Runtime profile mode enables detector CUDA event timing and
  per-frame metrics on `/diagnostics`; debug and production leave them off.

## Rebuild approach

Fill nodes and topics one by one. Per package: extract a pure core library + gtest
(unit-level), then wire the thin ROS node, then integrate. Keep core packages
source-agnostic. Build/test per package with `colcon build|test --packages-up-to <pkg>`.
Use ROS Humble: `source /opt/ros/humble/setup.bash`.

**Progress:**

- `amr_interfaces` DONE. It contains only tracked-obstacle contracts; collision
  state comes from `nav2_msgs/CollisionMonitorState`. The unproduced custom
  `SafetyState` contract was removed on 2026-07-27.
- `estop_gate` (renamed from `safety_gate` on 2026-07-27) — **software operator-stop latch
  only**. The radial-zone policy that used to live here was deleted in favour of
  `nav2_collision_monitor`; the latch survives because collision_monitor has no e-stop input.
  A controller e-stop message latches it and a controller reset clears it.
  `estop_gate_node` subscribes to `/cmd_vel/scan_health_limited`,
  `/estop/engaged`, and `/estop/reset`.
  While latched it publishes zero velocity for each received command; otherwise it publishes commands
  to `/cmd_vel/safety_limited` after scan-health limiting; it is the final
  software command authority.
- `stop_zone_recovery` — bounded reverse-only escape from a collision-monitor stop zone (user chose design A, 2026-10-14). If the monitor zeroed a command whose raw `/cmd_vel` is a fresh pure reverse (no strafe/rotation), it emits that reverse capped at 0.05 m/s iff a fresh scan shows the rear box (1.0 m behind, ±0.65 m) empty; otherwise the monitor output passes unchanged. Component tests, offline replay, and a 2026-10-28 live Isaac head-on run passed. Nav2's stop-output window is 24 h of simulated time so delayed recovery can receive monitor zeros; after that recovery fails closed. No hardware test. Not safety-rated.
- `scan_health_gate` — separate C++ scan/TF-health interlock. It consumes
  `/cmd_vel/recovery_limited` and `/scan`, publishes
  `/cmd_vel/scan_health_limited`, and emits zero at 20 Hz while scans have
  never arrived or are stale/unusable. It checks both ROS header age and a
  steady-clock receipt timeout, and requires the scan-to-base transform through
  `odom` at the current command time. A missing moving TF holds zero even if
  scans remain fresh. This is not safety-rated and a separate TF listener
  cannot prove the collision monitor processed the same scan. See
  `docs/validation/offline_isaac_safety_replay_2026-10-13.md`.
- `collision_monitor` — **WIRED AND VERIFIED ACTIVE.** Config at
  `configs/safety/collision_monitor.yaml`, launched by bringup together with its
  lifecycle manager. Publishes `/cmd_vel/collision_limited`, `/safety/zone_stop`,
  `/safety/zone_slow`; consumes `/cmd_vel` and `/scan`. Isaac and rosbag can
  provide `/scan`; the hardware LiDAR driver is still absent. A missing or
  stale scan is ignored by Nav2 Humble, so its own `source_timeout` is
  **fail-open**. The separate scan-health gate prevents nonzero commands
  from reaching e-stop during tested missing/stale-scan cases.
- `robot_bringup` rebuilt; see above.
- `robot_bringup` selects a backend: `hardware`, `isaac`, or `rosbag`.
  `backend:=isaac` launches no adapter: Isaac's ROS 2 graph must publish and
  consume the canonical AMR topics directly. `base_driver` and replay support
  remain to be rebuilt.
- `navigation_mode:=follow_path_test` is a guarded Isaac-only, no-driver
  Nav2 controller bringup. It activates the saved-map localizer, planner,
  Regulated Pure Pursuit controller, and full software safety chain, but does
  not send a path goal or start BT Navigator. Its no-goal lifecycle test passed
  over live Ethernet. Isaac's opt-in command-drive scene now consumes
  `/cmd_vel/safety_limited`; direct and safety-chain synthetic motion over
  Ethernet passed. The camera-enabled automated Ethernet suite also exited 0
  with synchronized RGB/depth, scan, odometry, and TF; camera load reduced
  Isaac to about 0.59× wall time. **Nav2 `FollowPath` now closes the loop in
  Isaac:** three consecutive bounded 0.40 m goals succeeded through the full
  safety chain at a regulated 0.0667 m/s, and the latched e-stop afterwards
  held 18 zero commands with 0.000 m drift. Each run stopped about 0.148 m
  short at the 0.15 m `xy_goal_tolerance`, and the measured post-run map
  residual was 0.177 m — larger than the tolerance Nav2 accepted on. Straight
  routes only; no turning, obstacle, recovery, BT Navigator, or scan-loss
  interaction during motion, and 0.59× wall time means no real-time or
  stopping-distance claim. The bounded runner is
  `tests/integration_tests/run_isaac_follow_path.py`; it latches the e-stop on
  exit, so publish `/estop/reset` before the next run. See
  `docs/validation/isaac_follow_path_drive_2026-10-13.md`,
  `docs/validation/isaac_follow_path_bringup_2026-10-13.md`, and
  `docs/validation/isaac_command_drive_wired_2026-10-13.md`.
- `perception_detection_depth_projection` rebuilt. It synchronizes standard 2D detections,
  aligned depth, and CameraInfo within a configurable bounded interval; takes
  a median depth from each box's center ROI; and publishes standard
  `vision_msgs/Detection3DArray` in the shared RGB/depth camera optical frame.
  Its projection and depth-decoding logic have ROS-independent unit tests, and
  its topics, QoS, synchronization, validation, and publishing path have ROS
  component tests.
- `perception_detector` rebuilt. It publishes standard `vision_msgs/Detection2DArray`
  from `/camera/rgb/image_rect_color` through a C++ `DetectorBackend` boundary.
  The first production backend is YOLO26/TensorRT. One internal preprocessing
  interface selects `cuda` (default) or `cpu`; both write FP32/FP16 into the same
  TensorRT input destination and return the same letterbox transform. CUDA fuses
  RGB/BGR, resize, letterbox, normalization, and CHW conversion in one kernel;
  CPU is the portable correctness/performance baseline. A reusable private
  TensorRT session now owns engine/context setup, tensor discovery and shapes,
  device buffers, the CUDA stream, enqueueing, and downloads so later detector
  architectures do not duplicate runtime plumbing. Postprocessing supports raw
  or native end-to-end output and original-image coordinate recovery. Raw
  output gets class-aware NMS; native YOLO26 output is never suppressed twice.
  There is deliberately no deployed Python/PyTorch or ONNX Runtime backend.
  Core and ROS component tests need no GPU; conditional device tests verify both
  preprocessing implementations and CUDA parity with the CPU reference. A
  conditional node integration test now runs the production node, factory,
  CUDA, TensorRT, installed engine, postprocessing, and ROS publishing from a
  generated RGB input. **Real YOLO26n inference is validated on this Orin:** both
  selections produced the same five bus/person detections through real ROS
  topics. Ten steady 810×1080 measurements averaged 19.00 ms topic-to-topic with
  CUDA and 113.84 ms with CPU. Bringup still checks the required engine and
  labels before launching the node.
- `perception_lidar_clustering` rebuilt. It converts valid `/scan` ranges to
  Cartesian points without copying the scan buffer, segments adjacent returns
  by a configurable Euclidean gap, joins clusters across a full-circle scan
  seam, filters small candidates, and publishes unclassified standard
  `vision_msgs/Detection3DArray` boxes in the scan frame. The planar boxes have
  zero Z extent because a 2D LiDAR cannot measure height. ROS-independent unit
  tests cover validation, projection, clustering, filtering, boxes, and seam
  handling; ROS component tests cover real topics, QoS, parameters, output
  mapping, empty scans, and malformed input. Bringup launches it as the
  `lidar_clustering` sensor-suite variant.
- `perception_tracking` rebuilt. Bringup launches the same executable twice:
  camera detections become `/perception/camera/tracks`, and unclassified LiDAR
  clusters become `/perception/lidar/tracks`. Each instance uses a constant-
  velocity XY Kalman filter, gated deterministic one-to-one association,
  confirmation, short-dropout prediction, expiry, persistent IDs, covariance,
  and dynamic/static classification. Tracking is internal to `odom` to remove
  robot ego motion, then outputs `amr_interfaces/TrackedObstacleArray` in
  `base_link`. It requires time-aligned sensor-to-odom and odom-to-base_link TF;
  localization will generate that moving TF from wheel odometry and IMU rather
  than feeding raw IMU into tracking. Eight ROS-independent core tests and six
  ROS node-component tests pass.
- `perception_fusion` rebuilt. It bounded-approximately synchronizes camera and
  LiDAR `TrackedObstacleArray` streams in `base_link`, performs deterministic
  gated one-to-one association, covariance-weights matched XY position and
  velocity, preserves unmatched evidence, and publishes the authoritative
  `/perception/fused_obstacles`. A source-ID registry provides globally unique
  fused IDs that remain stable across camera-only, LiDAR-only, and paired
  cycles. Eight ROS-independent core tests and eight ROS node-component tests
  pass, including 25 skewed and out-of-order synchronization cycles. Both input
  nodes must keep publishing arrays; empty arrays are supported, but an absent
  stream cannot form a synchronized output cycle.
- `localization_odometry_fusion` added. It wraps upstream `robot_localization`
  EKF, publishing `/odometry/filtered` from `/odom/wheel` (and `/imu/data` yaw
  rate on hardware). On hardware the EKF owns `odom -> base_link`; in Isaac the
  simulator owns TF. Its Isaac profile was replayed against the 2026-09-27 bag
  (251 finite filtered messages), but the bag lacks a recorded command and IMU.
  Isaac odometry covariance is all zero, so filtered covariance is not credible.
  The Jetson still needs `ros-humble-robot-localization` installed with sudo;
  the dependency is now in `scripts/setup/install_jetson_dependencies.sh`.
- `localization_slam_mapping` added. Bringup starts it only with
  `slam_mode:=mapping` (default `off`, including production). It runs upstream
  SLAM Toolbox against `/scan` and `odom -> base_link -> lidar_link`, publishes
  `/map`, and owns `map -> odom`. A replay of the 2026-09-27 Isaac bag produced
  occupancy maps and `map -> odom` TF; both map image and pose-graph export
  services returned success. A later two-lap Isaac run saved a map and
  validated repeat-route localization. Later diagnostics distinguish
  fine-accepted candidates, gate passes, and newly inserted graph edges.

**Localization validation through 2026-10-12:** the earlier two-lap Isaac map and repeat-route saved-map localization
are recorded in `docs/validation/isaac_mapping_localization_2026-09-28.md`.
A Jetson-only replay with gradual odometry TF drift is in
`docs/validation/isaac_odometry_drift_2026-09-30.md`; it maintained roughly
the same pose error despite up to 0.67 m raw input drift. The
`docs/validation/isaac_full_route_1m_map_and_localization_2026-10-12.md`
report covers the full two-lap bag at a **test-only** 1.0 m loop search radius:
1,328 mapped poses had 0.092 m median / 0.445 m p95 position error, nine
explicit loop candidates inserted graph edges with at most 0.123 m relative
translation error, and the graph gained inter-lap links through other Karto
paths. The resulting saved map localized on the existing diagonal route
with 0.031 m median / 0.055 m p95 error. Production SLAM parameters are
unchanged. The same diagonal bag with synthetic odometry TF drift up to
0.671 m still localized at 0.029 m median / 0.056 m p95 error. A
new pillar-cluster scene and independent localization route are in
`docs/validation/isaac_distinct_geometry_slam_2026-10-12.md`: the test-only
1.0 m radius mapped at 0.045 m median / 0.234 m p95 position error and
localized at 0.043 m median / 0.160 m p95, better than the 3.0 m replay.
The 1.0 m run inserted one explicitly audited loop edge with 0.016 m relative
translation error. Production SLAM settings remain unchanged. Realistic
odometry/noise and Nav2 controlled-motion validation remain before claiming
navigation readiness.

The new scene's map also passed the
`docs/validation/isaac_geometry_planner_only_2026-10-12.md` path-only replay:
five successful Nav2 plans (including a pillar detour), two rejected invalid
goals, and bounded saved-map localization. No `/cmd_vel` or controller ran;
the costmap emitted four throttled scan-drop notices, but the actual drop
count is unknown. A temporary 0.8 s TF wait eliminated those notices in one
replay; see `docs/validation/isaac_planner_tf_filter_2026-10-12.md`.
The 2026-10-13 1× follow-up directly counted the global costmap's scan TF
filter events: checked-in 0.2 s wait passed 95/532 scans, while a temporary
0.8 s wait passed 529/532. Three live-start Nav2 plans succeeded, but that
does not make the default costmap safe for motion. On the distinct geometry,
0.583 m injected wheel-TF translation drift plus 0.08 rad yaw drift left
saved-map localization at 0.028 m median / 0.122 m p95 position error.
See `docs/validation/isaac_live_start_planner_tf_2026-10-13.md`; at that
stage no timing or motion setting was changed.
The subsequent planner-only timing profile is documented in
`docs/validation/isaac_localizer_tf_timing_2026-10-13.md`. It changes only
the Isaac/rosbag planner-only SLAM Toolbox `transform_timeout` to 0.8 s;
Nav2's costmap wait stays 0.2 s and no motion control is enabled. Its clean
replay passed 514/520 received costmap scans; a synthetic obstacle marked
and cleared, and a separate smooth odometry-drift replay passed 514/522.
This future-dates map correction rather than making scan matching faster.
Record `/cmd_vel/safety_limited` in a future live run to verify
the command-to-motion path. The hardware `base_driver` remains to be rebuilt
under `src/drivers/`.

Bringup supports opt-in `slam_mode:=localization` with a required absolute
`pose_graph_prefix` pointing to paired `.posegraph`/`.data` files and a required
map-frame `map_start_pose:='[x, y, yaw]'`. It uses
upstream SLAM Toolbox and does not launch bag playback or motion control.
The 1× Jetson replay through that bringup mode matched the earlier 158-pose
localization result; see
`docs/validation/isaac_bringup_localization_2026-09-30.md`.

**Partly validated:** the zones in `collision_monitor.yaml` (stop 0.55 m, slow
1.25 m, 35% throttle) were measured against a moving simulated base on
2026-10-13. Both trigger at their configured geometry — slowdown at 1.288 m
nearest return at an exact 0.350 ratio, stop at 0.619 m — and the monitor, not
the controller, is what stops the base. **The original gate is still open:**
that was 0.10 m/s in Isaac, and the 7.4 mm of post-trigger travel is a
near-massless simulator response, not braking. Still measure real stopping
distance at max speed on hardware and size the stop zone from that, with
margin. Two findings came out of it: the stop action has no reverse exemption,
so a robot stopped with an obstacle in its stop zone **cannot drive itself
out** without the later bounded reverse-only recovery; and
the former `stop_pub_timeout: 2.0` made the monitor publish *nothing* after
the first two seconds of a stop, blocking delayed recovery. The current
86,400 s simulated window let a live Isaac robot reverse 0.0245 m out of an
occupied head-on stop zone with the rear clear; after the window it fails
closed. An actuator-side watchdog is still **mandatory** in the unwritten
`base_driver`. See `docs/validation/isaac_collision_zone_measurement_2026-10-13.md`
and `docs/validation/isaac_head_on_zone_recovery_2026-10-28.md`. The LiDAR
default is a 0.20 m adjacent-point gap and three-point minimum. A two-point
minimum retained more narrow returns in one static Isaac scene but increased
confirmed tracker output in paired live replay without improving four-object
label matches. Tracking and fusion timing/noise/association/expiry thresholds
still need representative Isaac and physical-sensor labels.
