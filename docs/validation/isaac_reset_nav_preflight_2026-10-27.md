# Isaac reset and saved-map Nav2 preflight — 2026-10-27

## Scope

The Windows command-drive scene was available over the direct Ethernet link.
This Jetson-side run checked the reset start, saved-map localization, Nav2
planning, and the repaired EKF dependency. It sent no Nav2 goal or motion
command and launched no physical driver.

## Evidence

- `jetauto-direct-test` was active on `eno1` at `169.254.71.212/16`. A 10 s
  domain-42 stream probe saw one `/clock`, `/scan`, `/odom/wheel`, `/tf`, RGB,
  depth, and camera-info publisher and Isaac's sole final-command subscriber.
  It received 62 scans, 59 RGB/depth pairs, and no backward stamps.
- Isaac `/odom/wheel` position was `(0, 0)` m, compared with `(0.8458, 0)` m
  in the prior survey. This indicates that the command-drive start was reset
  or the scene restarted; the Windows reset event itself was not observed.
- With `backend:=isaac`, `slam_mode:=localization`,
  `navigation_mode:=follow_path_test`, `enable_localization:=false`, no
  drivers, and the saved `loop_20260928` pose graph, the controller and planner
  lifecycle nodes reached `active`. SLAM Toolbox owned `map → odom` and Isaac
  owned `odom → base_link`.
- The no-motion runner measured map start `(0.1117, 0.0421)` m. The earlier
  expected start alignment for Isaac odometry zero was
  `(0.3717, 0)` m. Their planar residual is 0.263 m, above the required
  0.15 m motion gate. A 0.40 m request was planned only: 14 poses, 0.390 m
  path length, and zero cost at the start, goal, and sampled path cells.
  Passing expected-start coordinates to the no-motion runner did not apply
  the gate; its implementation checks them only with `--execute`.
- `ros-humble-diagnostic-updater` was upgraded from 4.0.6 to 4.0.7. The new
  package provides `libdiagnostic_updater.so`, and `ldd` resolves the EKF's
  dependency. The normal `localization_odometry_fusion` hardware launch
  started `/odometry_fusion` in an isolated ROS domain with no driver or
  sensor input. This verifies loading and launch wiring, not fused odometry.

## Decision and remaining work

The 0.263 m residual reproduces the earlier approximately 0.260 m offset at
Isaac odometry 0.846 m. A fresh scene start therefore did not resolve the
saved-map alignment discrepancy. Determine whether the original 0.3717 m
start convention or the localizer's scan correction is responsible by
independently comparing the saved map with simulator scene coordinates.
Recheck scan clearance, TF, lifecycle state, and the final-command chain before
any later motion run. Do not weaken the 0.15 m gate. Head-on, lateral, and
diagonal zone tests require the Windows scene to be restarted with each
`-Scenario` selection.
