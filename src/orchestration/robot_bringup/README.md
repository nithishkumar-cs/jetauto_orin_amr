# robot_bringup

Single entrypoint package for the robot runtime. It composes
perception, diagnostics, safety, localization, and navigation nodes using one
launch file plus mode arguments.

Primary entrypoint:

- `ros2 launch robot_bringup robot_stack.launch.py`

Primary launch axes:

- `instrumentation_mode`: `debug`, `profile`, or `production`
- `backend`: `hardware`, `isaac`, or `rosbag`; selects exactly one source edge
- `slam_mode`: `off`, `mapping`, or `localization`
- `navigation_mode`: `off`, `planner_only`, or `follow_path_test` (default `off`)
- component toggles: `enable_drivers`, `enable_detector`,
  `enable_geometry`, `enable_tracking`, `enable_fusion`, and the remaining
  entries in `configs/runtime_modes/system_modes.yaml`

Build with `./scripts/dev_helpers/build_mode.sh <mode>` before launching the
same instrumentation mode. This selects `Debug`, `RelWithDebInfo`, or `Release`
respectively; C/C++ assertions follow that build type. Runtime profile mode
enables detector CUDA event timing and makes the detector, depth projection,
LiDAR clustering, both trackers, and fusion publish per-cycle metrics on
`/diagnostics`. Debug and production omit that instrumentation.

`configs/runtime_modes/system_modes.yaml` supplies shared toggle defaults.
In debug and profile, disabling a required perception stage launches an empty
publisher tied to its input timestamps; disabling safety is allowed only with
drivers disabled and publishes zero on `/cmd_vel/safety_limited`. A disabled
driver starts no actuator. Production locks required components on. Variant
choices are checked: teleop is unavailable, navigation must match
`navigation_mode`, hardware drivers require odometry localization, and the
camera-only perception chain is not validated.
The default `backend:=hardware`
will start `base_driver` once that package exists. `backend:=isaac` launches no
adapter: configure Isaac's ROS 2 graph to publish and consume the canonical AMR
topics directly. `backend:=rosbag` is reserved for a replay launch, which will
be added after the bag manifest and simulation-time policy are defined.

When `localization` is enabled, bringup includes
`localization_odometry_fusion`, which starts the upstream `robot_localization`
EKF. Install `ros-humble-robot-localization` first (the Jetson dependency setup
script includes it); otherwise bringup reports localization as pending. The
Isaac profile consumes `/odom/wheel`, publishes `/odometry/filtered`, uses
simulation time, and leaves `odom -> base_link` TF ownership with Isaac. The
hardware profile consumes wheel odometry and `/imu/data`, publishes the same
filtered topic, and owns that TF. Hardware drivers must not broadcast a second
`odom -> base_link`. A rosbag playback uses the Isaac profile, but bringup does
not yet start the bag player.

Online 2D mapping is a separate, opt-in launch axis:

```bash
ros2 launch robot_bringup robot_stack.launch.py backend:=isaac slam_mode:=mapping
```

`slam_mode:=off` is the default even in production. Mapping starts
`localization_slam_mapping` (upstream SLAM Toolbox) alongside any enabled
odometry component. It consumes `/scan` plus TF from `odom` through
`base_link` to the scan frame, publishes `/map`, and owns `map -> odom`.
It does not command motion. On hardware, keep `enable_localization:=true` so
the EKF supplies `odom -> base_link`; Isaac and rosbag require that TF from
their source instead. Map saving is documented in the mapping package README.
The three-lap Isaac mapping route did not produce an accepted automatic loop
closure with the current 3.0 m loop-search default. An isolated replay with a
1.0 m diagnostic override did reach the acceptance path, but that setting is
not yet validated for production. See
`docs/validation/isaac_loop_closure_probe_2026-10-01.md`.
The larger Isaac route did produce accepted closures at 3.0 m, but one caused
a large map-frame pose jump against exact simulator odometry. That map is not
validated for navigation control; see
`docs/validation/isaac_extended_loop_closure_2026-10-05.md`.

Saved-map localization is the other opt-in SLAM mode. Pass the **absolute
prefix** of a SLAM Toolbox serialized map (both `.posegraph` and `.data`
must exist); a `.pgm`/`.yaml` occupancy map alone is not enough:

```bash
ros2 launch robot_bringup robot_stack.launch.py \
  backend:=rosbag slam_mode:=localization \
  pose_graph_prefix:=/path/to/saved_map \
  map_start_pose:='[-0.5, -1.0, 0.0]' \
  enable_drivers:=false enable_navigation:=false
```

This starts upstream `localization_slam_toolbox_node` with
`configs/localization/slam_localization.yaml`; it does **not** start a bag
player or command motion. With `backend:=rosbag`, replay `/clock`, `/scan`,
`/tf_static`, and the source's `odom -> base_link` TF in a separate process.
The replay must exclude any previously recorded `map -> odom` TF, since the
live localizer now owns it. A hardware run also requires the odometry-fusion
component. `map_start_pose` is required because this Humble localization node
needs an initial map-frame estimate, and its `map_start_at_dock` option is not
supported. The example pose is specific to the saved Isaac map, not a safe
hardware default. Relocalization through `/initialpose` is available after
startup. Never enable navigation control on an unverified initial pose.
On this Jetson, source the patched SLAM Toolbox 2.6.10 overlay before using
localization; otherwise its translation-only scan filter silently skips
in-place turns. Build and replay evidence are in
`docs/validation/isaac_heading_fix_2026-10-01.md`.

Planner-only navigation is an opt-in Isaac/rosbag validation mode. It requires
saved-map localization, `enable_navigation:=true`, and both drivers and teleop
disabled. It starts Nav2 `planner_server` and its lifecycle manager, but **not**
the controller, behavior-tree navigator, or a motor driver. Its global costmap
combines SLAM Toolbox's `/map` with live `/scan`; `/compute_path_to_pose` returns
paths without publishing `/cmd_vel`. The 0.33 m robot radius in
`configs/navigation/planner_only.yaml` is a provisional Isaac footprint, not a
measured hardware safety clearance.

For a recorded run, use the localization command above with
`navigation_mode:=planner_only enable_navigation:=true` instead of
`enable_navigation:=false`; keep the other motion-related toggles off. Then
replay the source bag in a separate terminal. If the bag contains `/clock`, do
**not** pass `--clock` to `ros2 bag play`: that would publish a second clock.
The recorded `/tf` must exclude `map -> odom`, leaving that transform to the
live localizer. See `docs/validation/isaac_planner_only_2026-10-01.md` for the
tested commands and path-only results.
The newer [pillar-cluster planner-only replay](../../../docs/validation/isaac_geometry_planner_only_2026-10-12.md)
also tested a route around an obstacle using the distinct-scene 1.0 m map;
it did not enable motion control.
The [1× live-start and scan-filter follow-up](../../../docs/validation/isaac_live_start_planner_tf_2026-10-13.md)
shows that the checked-in 0.2 s global-costmap TF wait passed only 95 of 532
received scans in that replay. Path success is not motion readiness; this
timing gap must be resolved before using the costmap for controlled driving.
For planner-only Isaac/rosbag validation, bringup now loads an additional
localizer parameter file setting SLAM Toolbox `transform_timeout` to 0.8 s;
normal localization retains 0.2 s. The
[timing report](../../../docs/validation/isaac_localizer_tf_timing_2026-10-13.md)
records the scan-admission, freshness, obstacle, and synthetic-drift checks.
This future-dates the map correction; it does not make scan matching faster
or authorize a controller.

`follow_path_test` is an opt-in **Isaac-only controller bringup**, not a
robot navigation mode. It adds Nav2 `controller_server` and its lifecycle
manager to the saved-map localizer, planner, and software safety chain. It
does not start `bt_navigator`, a driver, or a path goal. A separate caller
must request `ComputePathToPose`, then send that path to Nav2's `FollowPath`
action. The controller's `/cmd_vel` goes through collision monitor,
stop-zone recovery, scan-health gate, and e-stop; only `/cmd_vel/safety_limited` may be consumed
by Isaac's opt-in command-drive subscriber. The subscriber and full safety
chain passed a short wired, synthetic-command motion check; bringup alone
still sends no motion command.

The mode rejects hardware and rosbag backends, disabled safety, enabled
drivers or teleop, and missing saved-map localization. It also declines to
start controller/planner nodes if the safety chain or required Nav2 packages
are unavailable. The conservative 0.10 m/s RPP settings and 0.33 m simulated
footprint in `configs/navigation/follow_path_test.yaml` are starting values,
not validated stopping limits. The simulator command subscriber, watchdog, and
single `/clock` source were checked first in the
[wired command-drive check](../../../docs/validation/isaac_command_drive_wired_2026-10-13.md).

Nav2 has since followed a path in this mode: three bounded 0.40 m goals
succeeded through the full safety chain, and the latched e-stop afterwards
held zero velocity. The runs also showed a post-goal map residual (0.177 m)
larger than the configured 0.15 m `xy_goal_tolerance`, and regulated speed
capped at 0.0667 m/s. Only straight routes at 0.59× wall time were tested — no
turning, obstacle, recovery, or scan-loss interaction during motion. **This
does not authorize hardware motion.** Use
`tests/integration_tests/run_isaac_follow_path.py`, which bounds route length,
travel, and duration and latches the e-stop on exit. Its `--execute` mode
requires `--expected-start-map-x` and `--expected-start-map-y` from an
independent scene/map preflight and rejects a live map pose more than 0.15 m
away. A 2026-10-26 no-motion plan passed but had a 0.260 m start mismatch,
so no new goal was sent. See the
[live ground-truth report](../../../docs/validation/isaac_live_ground_truth_2026-10-26.md).
The no-goal lifecycle
smoke test is recorded in the
[FollowPath bringup report](../../../docs/validation/isaac_follow_path_bringup_2026-10-13.md),
the drive in the
[closed-loop drive report](../../../docs/validation/isaac_follow_path_drive_2026-10-13.md).
