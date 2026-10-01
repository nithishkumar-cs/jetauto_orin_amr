# Heading-aware SLAM Toolbox replay — 2026-10-01

The [dropout investigation](isaac_pose_dropout_2026-10-01.md) identified a
translation-only pre-filter in the Jetson's SLAM Toolbox 2.6.10. This follow-up
patches that filter and validates its behavior against the same recorded Isaac
route. The simulator, system `/opt/ros/humble` installation, and robot motion
interfaces were not changed.

## Build and behavior

`scripts/setup/build_slam_toolbox_heading_fix.sh` clones upstream tag `2.6.10`
at pinned commit `8293d21fe5c816d0405e0cc1f3eb46c70659c267` into the
Git-ignored `src/localization/slam_toolbox_vendor/`, applies the tracked
`patches/slam_toolbox/2.6.10-heading-aware-scan-filter.patch`, and builds an
overlay package into this repository's `install/`. No system package is
replaced. `ros2 pkg prefix slam_toolbox` resolved to this repository's install
prefix after sourcing it; bringup launched that patched executable.

The patch preserves the existing scan throttle, minimum time interval, and
translation threshold. It additionally passes a scan when normalized heading
change reaches `minimum_travel_heading`, configured as 0.05 rad. The angle is
normalized so crossing the ±π boundary does not appear to be a nearly 2π
turn. This is a local, minimal fix for version 2.6.10, not a claim that the
unmerged [upstream PR #888](https://github.com/SteveMacenski/slam_toolbox/pull/888)
is present in the apt release.

## Reproduction

From the AMR repository root:

```bash
bash scripts/setup/build_slam_toolbox_heading_fix.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix slam_toolbox  # must be this repository's install/
```

The validation used isolated `ROS_DOMAIN_ID=84`, the same saved pose graph
`/home/nithish/jetauto_isaac_maps/loop_20260928`, and the zero-drift filtered
copy of the original bag described in
[`isaac_planner_only_2026-10-01.md`](isaac_planner_only_2026-10-01.md). Bringup
used `backend:=rosbag`, `slam_mode:=localization`,
`navigation_mode:=off`, `map_start_pose:='[-0.5, -1.0, 0.0]'`, and all component
toggles—including drivers, navigation, and teleop—disabled. The filtered bag
was replayed without `--clock` because it already contains `/clock`. Only
the localizer's `/pose` was recorded to
`/tmp/jetauto_pose_heading_fix_20261001`.

The replay acceptance check is:

```bash
python3 rosbag/verify_pose_turn_replay.py /tmp/jetauto_pose_heading_fix_20261001
```

## Results

The patched localizer published 322 poses. Of these, 22 had timestamps between
79.35 s and route completion at about 84.567 s. The final matched pose was
stamped 84.55 s, and the largest gap between turn poses was 0.5 s. The
unpatched live run had no matched pose after 79.35 s during the same turn.

Against the saved mapping alignment and Isaac's exact odometry, the turn-only
matched poses had median/max position error of 0.0678/0.0810 m and median/max
heading error of 0.015/0.025 rad. Across the whole replay, median/95th/max
position error was 0.0754/0.1352/0.1660 m; heading error was
0.0159/0.0306/0.0391 rad. All 322 pose stamps matched reference odometry
within 3.6 ms. The patch thus restored fresh scan-matched updates through
the in-place turn without an obvious error increase in this recorded run.

This is a bag replay with exact simulated odometry, not validation on noisy
wheel/IMU input or a moving Nav2 robot. No controller or base driver ran.
All test processes were stopped. A localization-freshness guard and a separate
hardware-footprint/stopping-distance validation remain necessary before
autonomous motion.
