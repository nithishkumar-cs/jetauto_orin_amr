# Localizer-to-costmap TF timing — 2026-10-13

## Outcome

The 1× Isaac geometry replay exposed a localizer timing mismatch, not missing
wheel TF or missing LiDAR messages. SLAM Toolbox publishes `map -> odom` with
the latest scan timestamp plus its `transform_timeout`. At the base 0.2 s
setting, an independent recorder first saw the map TF corresponding to a
scan a median 0.350 s later in wall time (p95 0.441 s). At scan arrival, the
latest map TF stamp lagged the scan stamp by a median 0.400 simulated seconds;
only 56/534 observed scans already had map TF coverage for the scan's
additional 0.05 s Nav2 filter tolerance.

A **planner-only Isaac/rosbag override** now uses a 0.8 s SLAM Toolbox
`transform_timeout` while leaving Nav2's costmap wait at 0.2 s. This did
not speed up scan matching: the first corresponding map TF still arrived
0.358 s after its source scan at median (p95 0.466 s). It instead extends
the validity timestamp of the last available map correction. The localizer
continues to use source scan timestamps, not wall clock, for TF stamping.
This is a replay-scoped timing mitigation, not proof of safe motion.

## Controlled replay comparison

Each row used the same 534-scan, 53.7-simulated-second interior-route bag,
fresh saved-map localization, and planner-only Nav2. The source bag spans
40.6 wall seconds, so 1× playback is at **recording-time** speed. Only the
listed parameter changed. Filter counts are unthrottled
`tf2_ros_message_filter` DEBUG events; the filter was inactive for some
startup scans in each run. ROS ages at release are estimates from a separate
recorder's `/clock`, not exact internal planner times.

| Localizer TF offset | Costmap TF wait | Filter ready / received | Ready fraction | Ready scan age median / p95 (sim s) | `/pose` messages |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.2 s base | 0.2 s base | 95 / 532 | 17.9% | 0.134 / 0.305 | 161 |
| 0.2 s base | 0.8 s test-only | 529 / 532 | 99.4% | 0.604 / 0.837 | 164 |
| 0.6 s test-only | 0.2 s base | 414 / 525 | 78.9% | 0.175 / 0.316 | 162 |
| 0.8 s planner-only | 0.2 s base | 514 / 520 | 98.8% | 0.134 / 0.237 | 165 |
| 0.2 s + 0.10 m minimum travel test | 0.2 s base | 337 / 527 | 63.9% | 0.135 / 0.301 | 119 |

The minimum-travel test doubled the translation threshold for scan matching
and reduced typical map-TF publication lag, but still lost 190 costmap scans
and produced 42 fewer localization poses than base. It was not adopted.
The six drops in the 0.8 s localizer replay were at simulated 4.65, 15.45,
20.95, 26.55, 27.85, and 33.75 s; the remaining received scans passed.

The observer-side [TF coverage analyzer](../../rosbag/analyze_map_tf_coverage.py)
found 422/534 scans already covered at receipt with the 0.8 s localizer
offset, versus 56/534 at the base setting. For the rest, the estimated p95
wall wait for a covering map TF fell from 0.511 s to 0.085 s. This is why
the existing 0.2 s costmap wait now usually suffices. It does **not** mean
the map correction itself is 0.085 s old: its scan-matching latency remained.

## Pose and obstacle checks

Using the same fixed map-to-Isaac-odometry alignment for both replays, the
base localizer's position error was 0.0303 m median / 0.1144 m p95 (161
matched poses). The 0.8 s offset run was 0.0317 m median / 0.1163 m p95
(165 matched poses). Heading errors were 0.0113 / 0.0276 rad for base and
0.0109 / 0.0271 rad for the candidate. This route does not show a meaningful
localization-accuracy penalty from the offset; it is not a guarantee for
other routes, speeds, or hardware.

In a separate replay, only the planner consumed a cloned `/scan_costmap`
whose returns were shortened toward map `(2.0, 0.0)` during simulated
18–22 s. The original `/scan` still fed localization. The planner retained
its 0.2 s wait and explicit 2.0 m source height. It accepted 520/523 scans
and marked up to three target cells lethal at 18.317 s. The target cleared
at 22.650 s and stayed clear in later snapshots. The corrected dynamic
replay's ready-scan age was 0.134 s median / 0.222 s p95. There was no
controller, `/cmd_vel`, physical base, or active collision monitor.

The same 0.8 s planner-only profile was also replayed with a gradual
+0.5 m X, −0.3 m Y, +0.08 rad wheel-TF drift while untouched Isaac odometry
served as ground truth. It accepted 514/522 costmap scans, with estimated
ready-scan age 0.134 s median / 0.241 s p95. Its 160 matched poses had
0.0302 m median / 0.1124 m p95 position error; input translation drift
reached 0.583 m. This is one synthetic smooth drift profile, not a test of
wheel slip or loss of localization.

## Implementation and reproducibility

`configs/localization/slam_localization.yaml` remains at 0.2 s for normal
localization and future hardware review. Bringup loads the small
`configs/localization/slam_localization_planner_only.yaml` override **only**
for `navigation_mode:=planner_only`. That mode already rejects hardware,
drivers, teleop, and motion control. A launch-configuration test checks the
override is scoped and the base file remains unchanged. The override is
installed by `robot_bringup` and was built with `--symlink-install`. ROS
parameter resolution with the base file followed by the override returned
`transform_timeout=0.8`. Bringup's 31 package tests and 14 rosbag Python
tests passed.

All experiment bags/logs are under `/tmp/jetauto_localizer_*_20261013`.
The clean input is `/tmp/jetauto_geometry_localization_clean_20261012`, the
synthetic obstacle input is `/tmp/jetauto_costmap_obstacle_input_20261013`,
and the saved pose graph is `/tmp/jetauto_geometry_radius1_20261012/final_map`.
These may be deleted by a reboot. The drift input is
`/tmp/jetauto_geometry_drift_input_20261013`; its replay output is
`/tmp/jetauto_localizer_0p8_drift_20261013`. The diagnostic launch and runner are
`/tmp/jetauto_live_start_tf_debug_20261013.launch.py` and
`/tmp/jetauto_live_start_replay_20261013.sh`. The temporary test files are
`/tmp/jetauto_localizer_timeout_0p6_20261013.yaml` and
`/tmp/jetauto_localizer_timeout_0p8_20261013.yaml`. The latter is identical
to the base file except the timeout. Example analyzer commands:

```bash
source /opt/ros/humble/setup.bash
python3 rosbag/analyze_costmap_scan_filter.py \
  /tmp/jetauto_localizer_tf_timeout_0p8_20261013/stack_console.log \
  /tmp/jetauto_localizer_tf_timeout_0p8_20261013/observed_bag
python3 rosbag/analyze_map_tf_coverage.py \
  /tmp/jetauto_localizer_tf_timeout_0p8_20261013/observed_bag \
  --localizer-transform-timeout-s 0.8
python3 rosbag/analyze_costmap_obstacle.py \
  /tmp/jetauto_localizer_0p8_costmap_obstacle_20261013/observed_bag \
  --assert-mark-clear
```

## Limits and next gate

The 0.8 s offset future-dates an older map correction; it cannot cure slow
scan matching. It worked on this replay because map-to-odom correction
changed slowly on this route. It was not tested against abrupt localization
jumps, severe wheel slip, different maps, live Isaac-to-Jetson latency, or a
moving obstacle. The six residual drops are also real. Do **not** use this
planner-only result to enable a controller or claim navigation safety. The
next gate is a live-link timing run with fresh costmap observation checks and
the full collision-monitor/e-stop command path before controlled motion.
