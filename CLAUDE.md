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
  `src/platform/amr_interfaces`. Defines only the 3 custom contracts ROS has no
  standard for — `TrackedObstacle`, `TrackedObstacleArray`, `SafetyState`.
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
- `safety` — deterministic command gate. Consumes obstacle sources + e-stop; publishes `/cmd_vel/safety_limited`.
- `localization` — SLAM / localization & mapping.
- `navigation` — nav tasks + teleop relay.
- `drivers` — real hardware IO. Consumes `/cmd_vel/safety_limited`, publishes `/odom/wheel`.
- `bridges` — sim (Isaac) / replay adapters that present the *same* contracts as hardware.
- `tools` — diagnostics, benchmarks, evaluation.
- `robot_bringup` — orchestrator only (no business logic).
- `rosbag/` (outside `src/`) — replay scripts; not a ROS package.

## Settled design decisions

- **Source-agnostic core.** Hardware/sim/bag detail only at the edges (`drivers` / `bridges` / `rosbag`). Core packages never know the data source.
- **`base_interface` retired.** Motion contract: safety → `/cmd_vel/safety_limited` → driver/bridge → `/odom/wheel`. Any shared base math (mecanum IK / odometry / watchdog) that ends up duplicated goes in a **library**, never a resurrected node.
- **Safety is independent of navigation.** Separate, simple, trustworthy last-resort. It subscribes directly to obstacle sources, NOT to the map/planner.
- **Safety is multi-input, worst-case combined** ("anything screaming danger wins"): multiple obstacle topics + an e-stop. Detection-only obstacles are fail-open (blind to untrained objects) — add a class-agnostic geometric source (lidar/sonar) on the safety path. *(Was implemented before the reset; rebuild it: `evaluate_obstacles` + `estop_decision` + `combine_decisions` pure core, multi-source node.)*
- **Component tiers.** REQUIRED core (sensors / perception / safety / motion) is locked in production. OPTIONAL instrumentation (benchmarks, diagnostics, calibration_validator) is a free toggle. VARIANT selectors (teleop / localization / navigation, and the sensor suite) are chosen among *validated* combinations, not free toggles. A specific sensor (e.g. lidar) is required only if it's needed to meet the minimum sensing contract; otherwise it's a variant choice (camera-only vs camera+lidar), each with its own validated speed/stop envelope.
- **Modes** (kept: `debug` + `profile` + `production`). `debug`/`profile` have per-node toggles where `false` ⇒ synthetic substitution; `production` is locked (no toggles). Mode *behavior* (log/assert/timing/metrics) lives in `runtime_modes.yaml`. **Constraint: `safety` may be `false` only when `drivers` is `false`.**
- **Config homes.** Per-package `config/` = node default params; top-level `configs/` = deployment/robot config + mode config. Everything is read from the *installed* share path, so `--symlink-install` (or overriding the path launch-arg) is what makes edits live without a rebuild — location does not change that.
- **`src/<layer>/<pkg>/src` nesting is correct.** Outer `src` = colcon source space; inner `src` = ament C++ sources. colcon discovers packages by `package.xml` recursively, by name not path.

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

- `amr_interfaces` DONE. `SafetyState` constants are ordered by escalating
  severity (`CLEAR < SLOW < SENSOR_DEGRADED < STOP < ESTOP`) so a plain numeric
  compare picks the correct headline — one ordering serves the wire contract and
  the priority ranking.
- `safety_gate` (renamed from `safety_layer` on 2026-07-26) — **policy core
  DONE**, 36 tests.
  Memoryless primitives (`evaluate_obstacles` / `estop_decision` /
  `degraded_decision` / `combine`) plus stateful `step()`, which adds latched
  e-stop (explicit operator reset required), zone hysteresis, and source
  staleness. State is *passed*, never *held*: `step(inputs, prev_state, params)`
  returns the next state, so there is no clock and no member variable, and it
  stays deterministic. The node does not exist yet — that is the next increment.
- `robot_bringup` rebuilt; see above.

**Next:** the `safety_gate` ROS node — subscribe obstacle sources + e-stop +
`/cmd_vel`, measure message ages, call `step()`, publish
`/cmd_vel/safety_limited` and `SafetyState`. It needs an upstream obstacle
source to be useful, so either feed it synthetic obstacles or start the data
path at `sensor_drivers` (note: re-opens the platform-vs-drivers decision).

**Known fail-open holes in `safety_gate` (not yet fixed):** `RadialZones` params
are never validated — NaN radii make every comparison false and report CLEAR at
full speed while simultaneously reporting a 10 cm obstacle. NaN points in a cloud
are silently dropped, so an all-invalid depth frame reads as "saw nothing"
rather than "saw nothing usable". Obstacles are points, not volumes, so whoever
writes the node must decide centroid vs nearest-extent — centroid under-reports
for large objects.
