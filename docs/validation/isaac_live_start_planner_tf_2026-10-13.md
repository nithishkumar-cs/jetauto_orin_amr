# Live-start Nav2 planning, scan acceptance, and odometry drift — 2026-10-13

Follow-up: [the scan-to-costmap obstacle check](isaac_costmap_obstacle_2026-10-13.md)
found and corrected a separate per-source height parameter omission. It
also measured 0.4/0.6 s waits and verified synthetic obstacle mark/clear
with a temporary 0.8 s wait. The 0.2 s production TF wait remains unchanged.

## Outcome

Three `ComputePathToPose` requests using Nav2's **current robot pose**
(`use_start=false`) succeeded at changing positions on the distinct-geometry
interior route. The planner's path starts tracked the latest available
`map -> base_link` TF, within 0.016–0.027 m at output-bag receipt in the
checked-in-parameter run. No controller or velocity command was present.

The critical finding is the planner global costmap's scan TF filter. At
1× bag-recording speed, with its checked-in 0.2 s `transform_tolerance`,
the actual filter debug events counted **95 ready and 437 discarded scans**
out of 532 scans it received. With **only** a temporary 0.8 s tolerance
substituted, it counted **529 ready and 3 discarded** out of 532. A separate
1× run without debug logging still emitted 19 throttled drop notices and
produced a similar number of localization poses. The path-planning success
therefore does **not** validate live-obstacle costmap coverage at the current
setting. The production YAML is unchanged; controlled motion remains off.

## Replays and counts

The clean source bag was
`/tmp/jetauto_geometry_localization_clean_20261012` (534 `/scan`, 3,221
`/clock`, and 3,221 wheel TF messages). The saved 1.0 m **test-radius** pose
graph was `/tmp/jetauto_geometry_radius1_20261012/final_map`. All replays
used fresh SLAM Toolbox localization, the planner-only Nav2 profile,
`map_start_pose=[-0.5,-1.0,0]`, separate ROS domains, and `ros2 bag play
--rate 1.0` without an additional `--clock`. No prior `map -> odom` TF or
`/pose` was replayed. The source bag's 53.7 s simulated clock span was
recorded in 40.6 s of bag wall time; 1× playback here means **recording-time
speed**, not necessarily one simulated second per wall second.

| Costmap TF wait / logging | Domain | Filter input | Ready | Discarded | Fresh `/pose` | Throttled drop notices |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.2 s / filter DEBUG | 112 | 532 | 95 | 437 | 161 | 19 |
| 0.2 s / normal logging | 113 | Not counted | Not counted | Not counted | 163 | 19 |
| 0.8 s / filter DEBUG | 114 | 532 | 529 | 3 | 164 | 2 |

The two unreceived source scans in each DEBUG run preceded costmap filter
subscription. Counts come from that filter's unthrottled `Added message`,
`Message ready`, and `Discarding message` DEBUG events with `target=map`,
not from the INFO drop notices. All 532 received scans reached a ready or
discarded terminal event; no queue-full event was logged. A `ready` event
means the TF filter passed the scan to the obstacle-layer callback, **not**
that a costmap update completed or an obstacle was correctly marked.

The normal-logging 0.2 s control replay recorded 531 `/scan` messages (its
bag recorder missed startup messages), 163 `/pose`, and 19 drop notices.
This does not reveal its exact filter acceptance count, but similar pose
coverage and notices make a purely DEBUG-logging artifact unlikely. The
filter's INFO notice is throttled and must never be used as a drop count.

An offline observer of the 0.2 s DEBUG bag found wheel `odom -> base_link`
TF already available before each scan's arrival. The first `map -> odom`
transform covering the scan plus the filter's 0.05 s scan tolerance arrived
afterward: median 0.346 s, p95 0.511 s, maximum 0.604 s in observer wall
time. This supports late localization TF as the immediate filter bottleneck
on this replay. It does not measure the exact internal ROS-clock timeout.

## Plans from the live robot pose

`rosbag/probe_planner_only.py --live-start` sends `use_start=false` and no
explicit start pose. In the 0.2 s DEBUG run, the repeated goal was
`(2.27,1.20)` in `map`:

| Path stamp (sim s) | First path point (m) | Path length | TF distance at output receipt | Result |
| ---: | --- | ---: | ---: | --- |
| 7.683 | (−0.523, 0.600) | 2.877 m | 0.016 m | Success |
| 20.200 | (1.577, 1.200) | 0.693 m | 0.018 m | Success |
| 33.217 | (2.177, −1.050) | 2.267 m | 0.027 m | Success |

Every sampled path pose was known and below lethal cost in a fresh global
costmap response. The 0.8 s DEBUG run also succeeded three times. TF was
compared using what an independent recorder had received when each `/plan`
arrived; it is not an atomic measurement of the planner's internal TF
lookup. The latest common TF stamp could lag the path header stamp by
roughly 0.45–0.63 simulated seconds in the 0.2 s run. A path can therefore
begin at a recently available but not perfectly current robot pose. This
test does not establish safe closed-loop control or dynamic-obstacle response.

`rosbag/probe_planner_only.py` now supports mutually exclusive `--start x y`
and `--live-start`. The test-only launch and runner are
`/tmp/jetauto_live_start_tf_debug_20261013.launch.py` and
`/tmp/jetauto_live_start_replay_20261013.sh`. Their output bags and logs are
under `/tmp/jetauto_live_start_replay_20261013`,
`/tmp/jetauto_live_start_no_debug_20261013`, and
`/tmp/jetauto_live_start_tf_0p8_20261013`. The 0.8 s settings live only in
`/tmp/jetauto_planner_tf_tolerance_0p8_20261012.yaml`. These `/tmp` files
may disappear after reboot.

With ROS Humble and this workspace overlay sourced, the test-only runner
used these commands (each output directory must be new on a rerun):

```bash
bash /tmp/jetauto_live_start_replay_20261013.sh
JETAUTO_DOMAIN_ID=113 \
  JETAUTO_RUN_DIR=/tmp/jetauto_live_start_no_debug_20261013 \
  JETAUTO_LAUNCH_FILE=/tmp/jetauto_live_start_no_debug_20261013.launch.py \
  bash /tmp/jetauto_live_start_replay_20261013.sh
JETAUTO_DOMAIN_ID=114 \
  JETAUTO_RUN_DIR=/tmp/jetauto_live_start_tf_0p8_20261013 \
  JETAUTO_PLANNER_CONFIG=/tmp/jetauto_planner_tf_tolerance_0p8_20261012.yaml \
  bash /tmp/jetauto_live_start_replay_20261013.sh
```

## Separate odometry-drift check

`rosbag/inject_odometry_tf_drift.py` made a new bag from the original
distinct-geometry localization recording. It gradually added +0.5 m X,
−0.3 m Y, and +0.08 rad yaw bias to **only** `odom -> base_link` TF, beginning
at simulated 5 s and reaching full bias by 50 s. The untouched Isaac
`/odom/wheel` was retained as `/validation/reference_odom`; scans, clock,
and static TF were unchanged. The injected translation magnitude was
0.583 m. The derived input bag is
`/tmp/jetauto_geometry_drift_input_20261013` and the localizer-only output
is `/tmp/jetauto_geometry_drift_localization_20261013/output_bag`.
The derived bag and localizer replay were made with:

```bash
python3 rosbag/inject_odometry_tf_drift.py \
  /home/nithish/jetauto_geometry_validation_20261012/localization_bag \
  /tmp/jetauto_geometry_drift_input_20261013 \
  --x-m 0.5 --y-m -0.3 --yaw-rad 0.08 \
  --ramp-start-s 5.0 --ramp-duration-s 45.0
JETAUTO_INPUT_BAG=/tmp/jetauto_geometry_drift_input_20261013 \
  bash /tmp/jetauto_geometry_localization_20261012.sh \
  /tmp/jetauto_geometry_drift_localization_20261013 \
  /tmp/jetauto_geometry_radius1_20261012/final_map 115
```

Both the earlier clean 1× localizer output and this drifted 1× output were
compared to Isaac odometry with **the same fixed map alignment**, taken from
the clean output's final `map -> odom` TF. This common alignment differs
from the alignment used in the prior map-comparison report, so compare
these two rows with each other, not their absolute values with that report:

| Input | Poses / coverage | Position error median / p95 / max | Heading error median / p95 / max |
| --- | --- | --- | --- |
| Clean | 167; 0.25–53.15 s | 0.0287 / 0.1250 / 0.1443 m | 0.0111 / 0.0265 / 0.0309 rad |
| Drifted | 164; 0.25–53.15 s | 0.0276 / 0.1221 / 0.1469 m | 0.0120 / 0.0264 / 0.0309 rad |

All listed poses matched untouched Isaac odometry within 0.001 s. At drifted
pose times, the input translation drift reached 0.583 m (median 0.281 m),
yet the saved-map localization error did not materially increase on this
route. This tests one smooth, deterministic bias profile—not wheel slip,
stochastic odometry noise, relocalization after losing scans, or hardware.

## Decision

Do not enable motion from these path-only results. The global costmap's
current 0.2 s TF wait admitted too few scans in the 1× replay. The temporary
0.8 s result demonstrates a candidate timing remedy but can delay obstacle
updates and has not been validated against a live Isaac-to-Jetson transport,
moving obstacles, or controller behavior. Before closed-loop motion, verify
scan-to-costmap freshness and collision-monitor behavior under the actual
link and command path. No production parameter, source bag, saved map,
Isaac project, or hardware state was changed for this check.
