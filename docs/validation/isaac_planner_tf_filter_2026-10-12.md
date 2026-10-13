# Planner costmap scan-filter timing — 2026-10-12

## Result

The four scan-drop log lines in the [planner-only run](isaac_geometry_planner_only_2026-10-12.md)
are **throttled notices, not a count of four dropped scans**. A diagnostic
replay with the checked-in 0.2 s costmap `transform_tolerance` reproduced
four notices. The same bag with a temporary 0.8 s tolerance produced no
drop notices; the planner still activated and returned the same valid
cross-pillar detour (115 poses, 2.904 m, no lethal or unknown sampled poses).
Neither observation proves that every scan reached the costmap. The
production configuration remains at 0.2 s.

| Isolated 0.5× replay | Source scans | Fresh `/pose` | Logged filter notices | Cross-pillar plan |
| --- | ---: | ---: | ---: | --- |
| 0.2 s (checked-in) | 534 | 250 | 4 | Previously passed |
| 0.8 s (temporary file only) | 534 | 250 | 0 | Passed, 2.904 m |

The two output bags are at
`/tmp/jetauto_geometry_tf_diagnostic_20261012/observed_bag` and
`/tmp/jetauto_geometry_tf_tolerance_0p8_20261012/observed_bag`. Both runs
used the same clean interior-route bag, 1.0 m test-radius saved map,
localization settings, and planner settings except costmap TF tolerance.
The temporary 0.8 s YAML and launch are
`/tmp/jetauto_planner_tf_tolerance_0p8_20261012.yaml` and
`/tmp/jetauto_tf_tolerance_0p8_20261012.launch.py`; the exact runner is
`/tmp/jetauto_tf_tolerance_0p8_20261012.sh`. These `/tmp` artifacts may
disappear after reboot.

## What the TF trace shows

The raw bag supplied all 534 scans and continuous wheel odometry TF. In
the 0.2 s diagnostic replay, the recorded `odom -> base_link` transform
covering each of the four logged scan timestamps was already present when
the scan arrived. The first `map -> odom` transform covering the scan plus
the filter's 0.05 s scan tolerance arrived **0.261–0.431 s later** in wall
time for those four examples. Across the run, the estimated map-transform
arrival lag had a 0.409 s p95 and 0.754 s maximum. These are observer
arrival-time estimates, not a count of filter failures or a direct measure
of the filter's ROS-clock timeout.

With the complete recorded TF history available offline, every scan after
the first three startup scans can be transformed from `lidar_link` to `map`
at its start timestamp. The first three arrive before localization has
produced adequate map TF. This points to *when* the localizer publishes
`map -> odom` during replay, not a missing or out-of-order raw LiDAR stream.
It does not prove the exact failure reason for each runtime drop.

Nav2 Humble's [obstacle layer implementation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)
passes the costmap `transform_tolerance` to the scan TF filter and adds a
0.05 s scan tolerance. The installed Humble `tf2_ros::MessageFilter` logs
failures at most once every 2.5 s and maps any exception from its transform
future to the `OutTheBack` reason. Consequently, the log text “timestamp
earlier than all the data in the transform cache” alone is not proof of an
old scan timestamp. The timing trace and 0.8 s experiment support a TF
availability race, but not a precise number of dropped scans.

## Decision and next check

Do **not** promote 0.8 s to production from this half-speed bag replay.
A longer wait can delay obstacle updates; the result should be repeated at
normal simulation speed with a direct scan-acceptance counter or costmap
observation instrumentation. Check the live TF age and scan latency there,
then choose a tolerance based on measured worst-case behavior rather than
the absence of throttled log lines. No motion command or physical base was
used in either diagnostic run.

**Follow-up (2026-10-13):** The [1× replay with unthrottled costmap filter
DEBUG counts](isaac_live_start_planner_tf_2026-10-13.md) completed this next
check. It measured 95/532 scans ready at 0.2 s versus 529/532 at a temporary
0.8 s wait. Production remains unchanged pending live-link freshness and
motion-safety validation.
