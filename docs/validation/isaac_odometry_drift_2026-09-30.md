# Saved-map localization under injected odometry drift — 2026-09-30

The original [two-lap map and one-lap localization run](isaac_mapping_localization_2026-09-28.md)
used exact simulator odometry. This follow-up tests whether saved-map scan
matching can correct a controlled error in the odometry TF. Nav2 control and
the physical base were off throughout.

The Jetson replay bag was derived from the one-lap Isaac recording. It kept
`/clock`, `/scan`, and `/tf_static`; removed the old SLAM-generated
`map -> odom` TF; and ramped the `odom -> base_link` translation by
`(+0.60, -0.30) m` and yaw by `+0.12 rad` over the first 45 s of simulation
time. Unmodified `/odom/wheel` messages were retained as a separate reference
topic. No original bag or saved map was changed.

SLAM Toolbox's localization node loaded the existing serialized pose graph.
At 1× bag playback, a fixed alignment from the **mapping** bag's final
`map -> odom` TF was applied to the original Isaac odometry for comparison.
Using the live localization run's changing `map -> odom` TF for this comparison
would hide the correction being measured.

| Position error | Poses | Median | 95th percentile | Maximum |
| --- | ---: | ---: | ---: | ---: |
| Original replay, exact odometry | 157 | 0.0579 m | 0.1235 m | 0.1440 m |
| Drift-injected replay, localized pose | 158 | 0.0545 m | 0.1113 m | 0.1411 m |
| Raw injected odometry drift at pose stamps | 158 | 0.3123 m | 0.6254 m | 0.6708 m |

All 158 drift-run poses matched a reference odometry timestamp within 5 ms.
Localized heading error was 0.0042 rad median and 0.0134 rad at the 95th
percentile. The raw input drift is a paired diagnostic, not another SLAM
output. The similar localization errors despite the perturbed odometry are
evidence that scan matching corrected this particular gradual TF bias on the
same mapped route.

The derived input and output bags are temporary Jetson data under `/tmp`, not
committed. To repeat the input transformation, source ROS 2 Humble and run:

```bash
python3 rosbag/inject_odometry_tf_drift.py \
  /home/nithish/jetauto_isaac_localization_20260928 \
  /tmp/jetauto_drift_replay \
  --x-m 0.60 --y-m -0.30 --yaw-rad 0.12 --ramp-duration-s 45
```

Start `localization_slam_toolbox_node` with the companion Isaac project's
`config/slam_localization_isaac.yaml`, `use_sim_time:=true`, and
`map_file_name:=/home/nithish/jetauto_isaac_maps/loop_20260928`. Record
`/pose` in a new bag, then play the derived bag at 1× in the same isolated
ROS domain. Analyze the result with:

```bash
python3 rosbag/analyze_saved_map_localization.py \
  /home/nithish/jetauto_isaac_mapping_loop_20260928 \
  /home/nithish/jetauto_isaac_localization_20260928 \
  /path/to/recorded_pose_bag \
  --perturbed-bag /tmp/jetauto_drift_replay
```

This test reuses the same scans and route, changes only the published odometry
TF, and has no physical wheel slip or sensor noise. It is not evidence of
performance on a distinct route. The two-lap mapping bag did not record a
loop-closure event. A fresh replay with loop closure and debug logging enabled
emitted no explicit closure indication, so loop closure remains unverified;
map quality and `map -> odom` movement alone cannot establish it. A distinct
Isaac route and an explicit accepted loop-edge check remain before Nav2
control. The laptop is currently reachable only for outbound SSH to the
Jetson, so a new Isaac run cannot be started from the Jetson.
