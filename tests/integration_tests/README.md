# Integration Tests

Cross-package launch and contract tests.

`check_safety_chain.py` runs the installed Nav2 collision monitor,
lifecycle manager, `scan_health_gate`, and `estop_gate` with synthetic ROS
messages. Source Humble and the workspace overlay, then run
`python3 tests/integration_tests/check_safety_chain.py`. It asserts that
missing, stale, and unusable LiDAR data force zero at the final output;
see `docs/validation/offline_scan_health_interlock_2026-10-13.md`.
It starts no driver.

## Live Isaac direct-Ethernet acceptance

With one Windows Isaac command-drive instance already running on ROS domain 42
and the direct cable connected, run from the Jetson repository root:

```bash
python3 scripts/validation/run_isaac_wired_validation.py --isaac-simulation-only
```

The runner discovers the Jetson's current `eno1` IPv4 address, requires a
1 Gb/s carrier and a route to the Windows peer (`169.254.71.211` by default),
creates an Ethernet-only Fast DDS profile, and checks 60 seconds of clock,
LiDAR, odometry, TF, RGB, depth, and camera-info streams. It then runs the
direct command/watchdog probe, launches only the software safety chain,
checks e-stop and reset against actual Isaac odometry, and stops its own
Jetson processes. It never starts a hardware driver or sends a Nav2 goal.

It saves logs, the DDS profile, and `summary.json` in a printed directory
under `/tmp`. Exit `0` means all gates passed; exit `2` means the core wired
and motion checks passed but camera streams were missing/unhealthy; exit `1`
means a core, motion, link, or cleanup gate failed. Use a fresh, clear Isaac
command-drive scene for repeat runs because the probes move it short distances.
The Windows simulator must be started/stopped separately; the Jetson cannot
currently reach a Windows SSH server on the direct cable.

## Bounded Nav2 FollowPath drive

`run_isaac_follow_path.py` plans a short straight route on the saved map and
optionally sends it to Nav2's `FollowPath` action. It requires a running
`navigation_mode:=follow_path_test` bringup and a live Isaac command-drive
scene on the same domain:

```bash
python3 tests/integration_tests/run_isaac_follow_path.py --isaac-simulation-only
python3 tests/integration_tests/run_isaac_follow_path.py --isaac-simulation-only --execute
python3 tests/integration_tests/run_isaac_follow_path.py --isaac-simulation-only --verify-stopped
```

Without `--execute` it stops after planning and printing the plan. The
`--isaac-simulation-only` flag is mandatory, the route is restricted to
0.30–0.50 m, the goal is abandoned past 0.65 m of odometry travel or 50 s, and
preflight refuses to proceed unless there is exactly one `/clock` publisher,
one `/cmd_vel` subscriber, Isaac's subscriber on `/cmd_vel/safety_limited`, and
no `base_driver` node. `--verify-stopped` only checks that the latched e-stop
holds zero velocity, without sending a goal.

**It latches the software e-stop on exit by design.** Publish `/estop/reset`
before the next motion run. Results are in
`docs/validation/isaac_follow_path_drive_2026-10-13.md`.

## Collision-monitor zone measurement

`measure_collision_zones.py` drives a hand-built straight path at a real
obstacle to find where the monitor's stop and slowdown squares actually fire:

```bash
python3 tests/integration_tests/measure_collision_zones.py \
    --isaac-simulation-only --survey-only
python3 tests/integration_tests/measure_collision_zones.py \
    --isaac-simulation-only --distance-m 1.80 --travel-cap-m 1.55 --abort-range-m 0.45
```

The path is built locally rather than planned because the global planner routes
around obstacles and cannot produce this approach. `--survey-only` reports zone
occupancy and nearest returns without moving. During the drive the script
transforms every scan return into `base_link`, counts points inside the
configured squares, and compares `/cmd_vel` against
`/cmd_vel/collision_limited` to identify which layer stopped the base.

A travel cap and an abort on measured obstacle proximity both guard the run and
are checked every scan; either engages the e-stop. Set them so the proximity
abort binds first, otherwise dead reckoning can end the run before the monitor
acts. It refuses to start if the stop zone is already occupied. Like the
FollowPath runner, **it latches the e-stop on exit**. Results are in
`docs/validation/isaac_collision_zone_measurement_2026-10-13.md`.
