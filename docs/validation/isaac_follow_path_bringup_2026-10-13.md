# Isaac FollowPath bringup preparation — 2026-10-13

## Scope

Added an opt-in Jetson-side Nav2 controller path for a future Isaac-only
closed-loop drive test. This was a **no-goal, no-driver lifecycle smoke test**:
neither the simulator nor physical base was commanded to move.

## Change

- `navigation_mode:=follow_path_test` launches the saved-map localizer,
  planner server, controller server, their lifecycle managers, and the full
  collision-monitor → scan-health → e-stop software chain. It does not launch
  BT Navigator or a driver. A caller would have to request a plan and send
  it to `/follow_path` explicitly.
- The controller uses Nav2 Regulated Pure Pursuit at a conservative 0.10 m/s
  desired linear speed and an odom-frame local costmap from `/scan`. It
  publishes `/cmd_vel`, which enters the existing safety chain. A future
  Isaac-side subscriber must consume only `/cmd_vel/safety_limited`.
- Bringup rejects this mode unless the backend is `isaac`, the SLAM mode is
  saved-map localization, navigation and safety are enabled, and drivers and
  teleop are disabled. If the safety chain, localizer, or required Nav2
  packages are unavailable, the controller does not launch.

## Verification

- `colcon build --merge-install --packages-select robot_bringup
  --symlink-install` passed. The bringup package's 36 tests passed (18 planner
  mode, 17 SLAM mode, one safety-chain test); the added cases cover mode
  constraints, component selection, topic wiring, and YAML contracts.
- The first live configuration attempt aborted in Nav2 because YAML width
  and height values `3.0` had type `double`; Humble's local costmap declares
  them as integers. Changing both to `3` allowed an isolated localhost-only
  controller configure transition to succeed and load the local costmap,
  progress checker, goal checker, and Regulated Pure Pursuit plugin. A
  regression assertion now requires integer types.
- The corrected full launch used the existing saved pose graph at
  `/tmp/jetauto_geometry_radius1_20261012/final_map` and the single-source
  live Isaac feed over the direct Ethernet link. Collision monitor, planner
  server, and controller server each reached lifecycle state `active [3]`.
  `/compute_path_to_pose` and `/follow_path` actions were advertised.
  The ROS graph showed controller server as the only `/cmd_vel` publisher,
  collision monitor as its subscriber, and **zero subscribers** on the final
  `/cmd_vel/safety_limited` topic. No action goal was sent; all Jetson launch
  processes then exited cleanly. Isaac remained running.

## Limits / next step

This validates configuration and command-chain wiring, **not path following**.
The Windows Isaac scene still lacks a velocity-command subscriber; there is
no actuator watchdog, tested closed-loop drive, real-time guarantee, or
measured stopping-distance envelope. The 0.33 m footprint and 0.10 m/s
controller speed are simulation starting values, not hardware limits. The
map used for this activation test may not match the scene currently running;
no navigation goal was sent, so localization/path accuracy was not assessed.

The live smoke test used `ROS_DOMAIN_ID=42` and the direct-Ethernet Fast DDS
profile, with `backend:=isaac slam_mode:=localization
navigation_mode:=follow_path_test`, the saved pose-graph prefix above,
`map_start_pose:='[-0.5, -1.0, 0.0]'`, `enable_drivers:=false`,
`enable_teleop:=false`, `enable_safety:=true`, and
`enable_navigation:=true`. Perception and diagnostic toggles were disabled
for this focused run. No `FollowPath` goal was sent.

Next, add an Isaac-side subscriber for `/cmd_vel/safety_limited` that applies
Twist motion to the simulated mecanum base, clamps commands to a documented
simulation envelope, and sets velocity to zero on a monotonic-time command
timeout or scene shutdown. The simulator must not subscribe to unscreened
`/cmd_vel`.
Then test a short path at low speed with map pose, scan admission, commanded
and actual velocity, e-stop, and command-loss behavior recorded. Do not use
this mode on a physical robot.

## Later same-day follow-up

The separate Isaac command-drive mode was subsequently added and tested over
direct Ethernet. The final-topic subscriber, single `/clock` source, command
watchdog, odometry/TF response, and full software safety chain passed short
synthetic-command checks. See the
[wired command-drive report](isaac_command_drive_wired_2026-10-13.md).

Nav2 then followed a path in that scene. Three consecutive 0.40 m goals
succeeded through the full safety chain, superseding this report's "no path
following" limit. The goal residual, regulated speed, and sparse pose-update
caveats are in the
[closed-loop drive report](isaac_follow_path_drive_2026-10-13.md). The no-goal
lifecycle result above stands as the historical record of the mode's first
activation.
