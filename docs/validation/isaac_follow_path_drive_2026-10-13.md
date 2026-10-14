# Isaac Nav2 FollowPath closed-loop drive — 2026-10-13

## Scope

First closed-loop Nav2 path following in Isaac. The controller server planned a
short straight route on the saved map, sent it to `/follow_path`, and the
resulting commands drove the simulated base **through the complete software
safety chain**. Jetson-side only; Isaac ran on the Windows laptop over the
direct 1 Gb/s Ethernet link. No hardware was driven.

This is the step the [no-goal bringup report](isaac_follow_path_bringup_2026-10-13.md)
deferred and the [wired command-drive report](isaac_command_drive_wired_2026-10-13.md)
prepared by giving the scene a `/cmd_vel/safety_limited` subscriber.

## Setup

- Bringup: `backend:=isaac slam_mode:=localization
  navigation_mode:=follow_path_test instrumentation_mode:=profile`,
  `pose_graph_prefix:=/home/nithish/jetauto_isaac_maps/loop_20260928`,
  `map_start_pose:='[0.3716666667, 0.0, 0.0]'`, drivers/teleop/perception
  disabled, safety and navigation enabled.
- `ROS_DOMAIN_ID=42`, `rmw_fastrtps_cpp`, Ethernet-only Fast DDS profile at
  `/tmp/jetauto_ethernet_only_fastdds.xml`, `ROS2CLI_DISABLE_DAEMON=1`.
- Isaac ran the camera-enabled `-CommandDrive` scene at ~0.59× wall time, so
  the 10 Hz simulated `/scan` arrived at ~5.6 Hz of real time.
- Runner: `tests/integration_tests/run_isaac_follow_path.py`. It refuses to run
  without `--isaac-simulation-only`, restricts the route to 0.30–0.50 m, aborts
  if odometry travel exceeds 0.65 m, caps the goal at 50 s, and latches the
  software e-stop when it finishes.

## Preflight gates (all required before any goal)

The runner asserts, and the run confirmed: exactly one `/clock` publisher; a
single `/cmd_vel` subscriber (collision monitor); exactly two
`/cmd_vel/safety_limited` subscribers including Isaac's `CmdVelSub`; no
`base_driver` node present; both `/compute_path_to_pose` and `/follow_path`
servers available; and the start and goal cells both at global-costmap cost 0.

## Results

Three consecutive `--execute` runs on the same live scene:

| Run | Start map x (m) | Plan length (m) | `follow_status` | Odom travel (m) | Max `/cmd_vel/safety_limited` linear x (m/s) | Pose updates | Scans |
|---|---|---|---|---|---|---|---|
| 1 | 0.588 | 0.4244 | 4 (SUCCEEDED) | 0.2518 | 0.0667 | not recorded | not recorded |
| 2 | 0.832 | 0.4207 | 4 (SUCCEEDED) | 0.2501 | 0.0667 | 2 | 48 |
| 3 | 1.074 | 0.4185 | 4 (SUCCEEDED) | 0.2517 | 0.0667 | 3 | 47 |

Run 3 is the accepted record: it exercised the whole script end-to-end,
including the latched-stop check, and exited 0. Runs 1 and 2 predate the
teardown fix below, so run 2 exited 1 despite its drive succeeding.

Post-goal latched-stop check (run 3, and again standalone after run 2): with
e-stop engaged and 0.05 m/s commands still being published for 1.0 s, the
final safety topic carried **18 commands, all exactly zero**, and odometry
drift was **0.000 m**.

Each run requested a 0.40 m displacement and planned ~0.42 m over 15 poses at
maximum costmap cost 0.

## What this does and does not establish

Established: Nav2's controller server can follow a planned path in this scene;
regulated pure pursuit output traverses collision monitor → scan-health gate →
e-stop latch and reaches the simulator; the saved-map localizer keeps updating
during controlled motion; and the e-stop latch still overrides the controller
after a completed goal.

Not established, and visible in the numbers:

- **Goal residual is larger than the configured tolerance.** Each run stopped
  after ~0.252 m of a 0.40 m route — about 0.148 m short, right at the
  `xy_goal_tolerance: 0.15`. After run 3, `map → base_link` measured
  x = 1.297 m against a 1.474 m goal, a **0.177 m residual**: Nav2 accepted the
  goal on its in-the-moment pose estimate, and the localizer's later correction
  left the robot further out than the tolerance it was checked against. The
  useful travel is barely above the tolerance, so this validates the loop, not
  tracking accuracy.
- **Commanded speed never reached the configured value.** Peak linear x at the
  final topic was 0.0667 m/s against `desired_linear_vel: 0.10`, from regulated
  scaling on such a short path. No speed envelope was measured.
- **Localization updated only 2–3 times per goal.** SLAM Toolbox in
  localization mode publishes on accepted scan matches during motion, not
  continuously; over a ~2.5 s simulated drive that is a handful of corrections.
  Path-following accuracy under sparse pose updates is untested.
- Straight 0.4 m routes only. No turning, no obstacle in the path, no
  recovery behavior, no BT Navigator, no scan-loss or zone-stop interaction
  during motion, and no repeated/long-duration running.
- Simulation ran at 0.59× wall time. **Nothing here is a real-time or
  stopping-distance claim**, and the zone sizes in `collision_monitor.yaml`
  remain unmeasured.

## Fix made during this work

`run_isaac_follow_path.py` passed `flush=True` into `json.dumps` instead of
`print`, raising `TypeError` in teardown after the run's assertions had already
passed. Run 2 hit this: its drive succeeded but the process exited 1 and the
e-stop check never reported. Fixed, and a `--verify-stopped` mode was added to
check the latched stop on its own without sending a goal.

## State left behind

The software e-stop is **latched** by design. Publish `/estop/reset` before any
further simulated motion. Isaac and the Jetson bringup were both left running.

## Next

Turning and multi-segment paths, then a path with an obstacle to exercise the
local costmap and collision-monitor zones during motion. Tighten
`xy_goal_tolerance` relative to route length, or test over a long enough route
that the tolerance is not most of the distance. Record
`/cmd_vel/safety_limited` to a bag during the next live run. Do not run this
mode on the physical robot.
