# Isaac scan-to-costmap obstacle check — 2026-10-13

## Result

An offline replay found a second, independent planner-costmap issue. With
the former Nav2 configuration, scans passed the TF filter but **no synthetic
obstacle was marked**. Nav2 Humble's per-source `scan.max_obstacle_height`
default is `0.0` m; the Isaac `lidar_link` is `0.24` m above `base_link`, so
the observation buffer rejected its returns. The planner-only profile now
sets `scan.max_obstacle_height: 2.0`. This is a parameter correction, not a
claim that a 2.0 m threshold is calibrated for hardware.

With that one height correction and a **test-only** 0.8 s TF wait, a target
at map `(2.0, 0.0)` became lethal in the global costmap while a synthetic
return was present, then cleared after the return was removed:

| Replay | Target before (sim <18 s) | During/just after 18–22 s | After 26 s |
| --- | ---: | ---: | ---: |
| Missing source height | 0 lethal cells | 0 lethal cells | 0 lethal cells |
| Explicit 2.0 m source height | 0 | 2 | 0 |

The corrected run first showed the lethal cells at simulated 19.250 s,
retained them at 20.550 and 22.067 s, and first showed them cleared at
23.467 s. All 40 recorded costmap snapshots ended clear. This verifies
marking and clearing in this synthetic replay, not stopping a moving robot.

## Fixture and isolation

`rosbag/inject_costmap_obstacle.py` took the clean geometry-localization
bag, kept all **10,200 source messages byte-for-byte unchanged**, and added
534 `/scan_costmap` clones. Forty scans from simulated 18 to 22 s had 197
beam ranges shortened toward the test point using Isaac reference odometry,
an estimated fixed `map -> odom` alignment, and the simulated 0.15 m forward
LiDAR mount. The original `/scan` continued feeding SLAM localization.
Only the planner obstacle layer consumed `/scan_costmap` in a temporary
configuration; outside the injection interval it was an exact scan clone.
No controller, command publisher, base driver, or physical robot was used.

The input bag is `/tmp/jetauto_costmap_obstacle_input_20261013`; the two
observer bags are `/tmp/jetauto_costmap_obstacle_replay_20261013/observed_bag`
and `/tmp/jetauto_costmap_obstacle_height_fixed_20261013/observed_bag`.
The saved map is `/tmp/jetauto_geometry_radius1_20261012/final_map`.
These artifacts are outside Git and may disappear after reboot.

The temporary launch and runner are
`/tmp/jetauto_live_start_tf_debug_20261013.launch.py` and
`/tmp/jetauto_costmap_obstacle_replay_20261013.sh`. The temporary planner
config is `/tmp/jetauto_costmap_obstacle_0p8_20261013.yaml`; its only
differences from the checked-in planner profile are the `/scan_costmap`
input and 0.8 s TF wait. The runner used separate ROS domains and recorded
`/global_costmap/costmap` as `nav_msgs/OccupancyGrid`. Run these checks with
ROS Humble and this workspace overlay sourced:

```bash
python3 rosbag/inject_costmap_obstacle.py \
  /tmp/jetauto_geometry_localization_clean_20261012 \
  /tmp/jetauto_costmap_obstacle_input_20261013
JETAUTO_DOMAIN_ID=119 \
  JETAUTO_RUN_DIR=/tmp/jetauto_costmap_obstacle_height_fixed_20261013 \
  bash /tmp/jetauto_costmap_obstacle_replay_20261013.sh
python3 rosbag/analyze_costmap_obstacle.py \
  /tmp/jetauto_costmap_obstacle_height_fixed_20261013/observed_bag \
  --assert-mark-clear
```

The output paths must not already exist on a rerun. The before-fix run used
the same temporary config without `scan.max_obstacle_height`. Source-side
behavior is visible in the [Nav2 Humble obstacle layer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)
and [observation buffer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp):
the former declares a per-source 0.0 m default and the latter removes points
above the configured maximum after transforming them to the global frame.

## TF timing sweep

All rows used the same clean scan bag at 1× **bag-recording time**, a fresh
saved-map localizer, and an otherwise identical planner-only launch. The
source bag's 53.7 simulated seconds span 40.6 recorded wall seconds; these
are not necessarily one simulated second per wall second. Counts are from
unthrottled `tf2_ros_message_filter` DEBUG events. `ready` means a scan
reached the obstacle-layer callback, not that it was marked.

| TF wait | Received | Ready | Discarded | Ready fraction | Median / p95 estimated ROS age at release |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.2 s | 532 | 95 | 437 | 17.9% | 0.134 / 0.305 s |
| 0.4 s | 531 | 171 | 360 | 32.2% | 0.317 / 0.526 s |
| 0.6 s | 525 | 404 | 121 | 77.0% | 0.575 / 0.710 s |
| 0.8 s | 532 | 529 | 3 | 99.4% | 0.604 / 0.837 s |

`rosbag/analyze_costmap_scan_filter.py` estimates age by interpolating a
separate recorder's `/clock` samples to each planner log event's wall time.
The recorder and planner are separate processes, so those ages are observer
estimates, not exact internal Nav2 timestamps. The 0.6 s run received fewer
startup scans than other runs; use received counts as denominators. Longer
waits admit more scans but make admitted obstacle data older. In the
height-corrected obstacle replay at 0.8 s, 523/528 scans were ready and
the estimated ready-age median/p95 was 0.618/0.840 simulated seconds.

## Decision

The per-source height setting is corrected in
`configs/navigation/planner_only.yaml`. Its production TF wait remains
`0.2` s: the sweep does **not** establish that simply raising it to 0.8 s
is safe. The live Isaac-to-Jetson transport, scan freshness, moving-obstacle
response, collision monitor, controller, and real hardware are still
unvalidated. Keep controlled motion disabled until the full command and
sensor path is tested under realistic timing.
