# Saved-map localization through robot_bringup — 2026-09-30

`robot_bringup` now has an opt-in `slam_mode:=localization`. It requires the
absolute prefix of paired SLAM Toolbox `.posegraph` and `.data` files and an
explicit initial map-frame pose `[x, y, yaw]`. The launch supplies those to
upstream `localization_slam_toolbox_node` with simulation time for the
`rosbag` backend. It does not start bag playback or Nav2 control.

The integration check ran on the Jetson in isolated ROS domain 81. All other
bringup component toggles, including drivers, safety, and navigation, were
disabled. The saved map was
`/home/nithish/jetauto_isaac_maps/loop_20260928`, and the initial pose was
`[-0.5, -1.0, 0.0]`, matching the earlier successful manual localization
profile. The 1× input was the derived one-lap Isaac bag from the
[odometry-drift validation](isaac_odometry_drift_2026-09-30.md). It contains
`/clock`, `/scan`, `/tf_static`, and the deliberately biased
`odom -> base_link` TF, with old `map -> odom` transforms removed. The
original Isaac odometry was retained separately for comparison.

| Output or check | Result |
| --- | ---: |
| `/pose` messages matched to reference odometry | 158 / 158 |
| `/map` messages | 134 |
| Live `map -> odom` transforms in `/tf` | 1,249 |
| Median localized position error | 0.0545 m |
| 95th-percentile localized position error | 0.1113 m |
| Maximum localized position error | 0.1411 m |
| Raw injected odometry drift, maximum | 0.6708 m |
| Maximum pose/reference timestamp skew | 0.0049 s |

The position-error metrics exactly match the prior manual 1× localization
replay, establishing that the new bringup mode passes the correct map,
configuration, initial pose, and simulation-time settings. The output bag
recorded `/pose`, `/map`, and `/tf`; the input bag contains no old
`map -> odom` TF, so the replay did not compete with the live localizer for
that transform. The output bag is
`/tmp/jetauto_localization_bringup_explicit_pose_20260930` and is not
committed.

An earlier trial used `map_start_at_dock: true`. This Humble localization node
warned that dock start is unsupported, and its 154 poses had **1.0033 m**
median position error. That fallback was removed. The launch now rejects a
missing or non-finite `map_start_pose` before starting the node.
Direct launch checks also rejected a missing `pose_graph_prefix` and a missing
`map_start_pose` with nonzero exit codes.

To reproduce, build `robot_bringup` with `--merge-install --symlink-install`,
source ROS Humble and `install/setup.bash`, and launch localization with
`backend:=rosbag`, `slam_mode:=localization`, the saved
`pose_graph_prefix`, and `map_start_pose:='[-0.5, -1.0, 0.0]'`. Disable the
other component toggles for an isolated replay. In the same `ROS_DOMAIN_ID`,
record `/pose /map /tf`, then play the derived bag at 1×. Analyze it with
`rosbag/analyze_saved_map_localization.py`, passing the original mapping bag,
the original localization bag, the new output bag, and the derived bag via
`--perturbed-bag`. The [bringup README](../../src/orchestration/robot_bringup/README.md)
documents the launch arguments and transform-ownership requirements.

This is still a saved-route bag replay, not a distinct live Isaac route or a
physical robot test. An accepted loop closure remains unverified. Nav2
planning and control were not enabled. All replay and localization processes
were stopped after the check.
