# Claude Handoff — Clean-Slate Rebuild

This repo is a ROS 2 Humble AMR stack for JetAuto on Jetson Orin (4-wheel mecanum).
On 2026-06-21 it was deliberately reset to a clean slate to clear old-version junk
and confusion. We now rebuild nodes and topics one by one against the design
decisions below.

## Current state

- **Reset done.** Every ROS package under `src/` was nuked **except `robot_bringup`**.
- The layer folders are **kept but empty**: `bridges drivers localization navigation
  perception platform safety tools`. (Empty dirs are not tracked by git — add a
  `.gitkeep` per folder if you want the skeleton versioned.)
- **`robot_bringup` is kept but is BROKEN-ON-LAUNCH by design** — see *Deferred* below.
- **`amr_interfaces` rebuilt (DONE, standards-first).** Lives at
  `src/platform/amr_interfaces`. Defines only the 2 custom contracts ROS has no
  standard for — `TrackedObstacle`, `TrackedObstacleArray`.
  Everything else uses standards: detection → `vision_msgs/Detection2DArray` +
  `Detection3DArray`; health → `diagnostic_msgs`; e-stop → `std_msgs/Bool`;
  odometry → `nav_msgs`. Builds + message generation verified (needs
  `ros-humble-vision-msgs` installed). `TrackedObstacle` has no per-obstacle
  header — frame/stamp come from the array header (base_link).
- **`system_modes.yaml` was moved** out of the package to
  `configs/runtime_modes/system_modes.yaml` (sibling of `runtime_modes.yaml`).

## Layer purpose

- `platform` — shared message contracts (`amr_interfaces`) + TF/calibration foundation. Source-agnostic; no algorithms. *(Dissolution pending — see Pending decisions.)*
- `perception` — camera → detection → depth-projection → tracking → fusion → `/perception/fused_obstacles`.
- `safety` — collision avoidance plus an operator stop. **Obstacle zones are `nav2_collision_monitor` (upstream), not our code**; `estop_gate` is the final latching software operator-stop, so Nav2 cannot override it. Chain: `/cmd_vel` → collision_monitor → `/cmd_vel/collision_limited` → e-stop latch → `/cmd_vel/safety_limited`.
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

## robot_bringup (REBUILT — launches clean, launches nothing)

Rewritten 2026-07-26. It builds, launches in all three modes, and enforces the
tier rules. It starts **no nodes**, because no package ships one yet.

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
- **Still open.** *Synthetic substitution* — a `false` toggle currently logs its
  intent instead of spawning a stand-in publisher; it needs the output contracts,
  so it waits on the packages. *Variant validated-sets* — `variant` components are
  currently free toggles; the "validated combinations only" rule is not enforced.

## Pending design decisions (not yet made)

- **`platform` dissolution** — move `sensor_drivers` → `drivers/`, promote `amr_interfaces` + `tf_and_calibration` to top-level, delete `platform/`. Discussed, not decided.
- **`preproc`** — was a dead orphan (its output was consumed by nobody; the detector did its own preprocessing). On rebuild, wire it into the detector or drop it.
- **Safety `frame_id` validation** — safety trusted the obstacle frame blindly (fail-open). Decide skip-vs-degrade on a frame mismatch.
- **debug vs profile divergence** — the `runtime_modes.yaml` knobs (`enable_assertions`, `enable_cuda_timing`, `export_metrics`) are declared but inert (only `log_level` is wired). Wire them or remove them. Add a build-type axis (Release/RelWithDebInfo for profiling vs debug build) — the modes do not model compiler flags today.

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
  `estop_gate_node` subscribes to `/cmd_vel`, `/estop/engaged`, and `/estop/reset`.
  While latched it publishes zero velocity for each received command; otherwise it publishes commands
  to `/cmd_vel/safety_limited` after collision_monitor publishes its result to
  `/cmd_vel/collision_limited`; it is the final software command authority.
- `collision_monitor` — **WIRED AND VERIFIED ACTIVE.** Config at
  `configs/safety/collision_monitor.yaml`, launched by bringup together with its
  lifecycle manager. Publishes `/cmd_vel/collision_limited`, `/safety/zone_stop`,
  `/safety/zone_slow`; consumes `/cmd_vel`. It has **no observation source
  publishing yet** — `/scan` is declared but no driver exists — so it currently
  gates nothing.
- `robot_bringup` rebuilt; see above.
- `robot_bringup` selects a backend: `hardware`, `isaac`, or `rosbag`.
  `backend:=isaac` launches no adapter: Isaac's ROS 2 graph must publish and
  consume the canonical AMR topics directly. `base_driver` and replay support
  remain to be rebuilt.
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
  validated repeat-route localization, but an accepted pose-graph loop
  closure remains unverified.

**Next:** the longer two-lap Isaac map and repeat-route saved-map localization
are recorded in `docs/validation/isaac_mapping_localization_2026-09-28.md`.
A Jetson-only replay with gradual odometry TF drift is in
`docs/validation/isaac_odometry_drift_2026-09-30.md`; it maintained roughly
the same pose error despite up to 0.67 m raw input drift. Still test a
distinct Isaac route and explicitly verify an accepted loop closure before
Nav2 control. Record `/cmd_vel/safety_limited` in a future live run to verify
the command-to-motion path. The hardware `base_driver` remains to be rebuilt
under `src/drivers/`.

Bringup supports opt-in `slam_mode:=localization` with a required absolute
`pose_graph_prefix` pointing to paired `.posegraph`/`.data` files and a required
map-frame `map_start_pose:='[x, y, yaw]'`. It uses
upstream SLAM Toolbox and does not launch bag playback or motion control.
The 1× Jetson replay through that bringup mode matched the earlier 158-pose
localization result; see
`docs/validation/isaac_bringup_localization_2026-09-30.md`.

**Unvalidated:** the zone sizes in `collision_monitor.yaml` (stop 0.55 m, slow
1.25 m, 35% throttle) were carried over from the old radial design and have
never been measured against real stopping distance. Measure it at max speed and
set the stop zone larger, with margin, before trusting them. The LiDAR
clustering defaults (0.20 m adjacent-point gap, three-point minimum) and all
tracking and fusion timing/noise/association/expiry thresholds are also
starting values only; tune them against representative Isaac and recorded
hardware scenes.
