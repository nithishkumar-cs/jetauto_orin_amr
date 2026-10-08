# Extended Isaac route: false loop match and inaccurate mapping — 2026-10-05

## Outcome

The new two-lap Isaac route **triggered SLAM Toolbox's loop-closure acceptance
path at the normal 3.0 m search distance**. It did **not** produce a
trustworthy map-frame trajectory. One incorrect link was followed by a
1.37 m increase in position error within 0.2 s, and a same-input control run
with loop closing disabled did not make that jump. Do not use this mapping
result as a Nav2 control map or treat an acceptance marker as proof of a
graph link or a correct loop closure. The subsequent
[focused diagnosis](isaac_false_loop_match_2026-10-08.md) found that four
early markers failed Karto's link gate, while a later match to a physically
different location passed it and caused the pose jump.

This follows the [small-route candidate probe](isaac_loop_closure_probe_2026-10-01.md),
which found no candidates at 3.0 m. The larger route therefore solved the
candidate-availability problem, but exposed a wrong-match or optimization
problem that needs separate investigation.

## Inputs and replay

The Windows Isaac project added a separate `-LoopClosureRoute` option and
recorded the raw bag at
`/home/nithish/jetauto_loop_closure_route_20261005/bag`. It has 5,298
`/scan` messages, 31,807 `/clock` and source `/tf` messages, 31,805
`/odom/wheel` messages, and three `/tf_static` messages over 539 s of
recording. Its `/tf` contains only `odom -> base_link`; no previously recorded
`map -> odom` transform needed removal. The simulator handoff reported a
6.80×4.90 m route extent, two completed laps, and a 0.1056 s maximum scan
gap. The recording used an unmeasured Tailscale/Fast DDS unicast-peers link
because the laptop and Jetson were on different Wi-Fi subnets. This replay
validates the recorded stream, not same-LAN network performance.

On isolated ROS domain 88, the locally patched SLAM Toolbox 2.6.10 mapper
was launched with `configs/localization/slam_mapping.yaml`,
`use_sim_time:=true`, and the optional
`patches/slam_toolbox/2.6.10-loop-closure-probe.patch`. Parameter queries
confirmed `loop_search_maximum_distance=3.0` and `do_loop_closing=true`;
there was no loop-search override. The raw bag played at 1×, while a separate
recorder captured only the live mapper's `/pose` in
`/tmp/jetauto_loop_route_probe_20261005/pose_bag`. Both the player and
recorder exited cleanly.

The original probe logged **15 fine-accepted candidates** after calling
`LinkChainToScan` and `CorrectPoses`: four at 122.75–123.35 s, one at 224.85 s, nine at
237.45–248.95 s, and one at 302.75 s. The log is
`/tmp/jetauto_loop_route_probe_20261005/mapper_console.log`. The final
count of actual new graph edges in that full replay is unverified: the
`LinkChainToScan` call can return without linking when its distance gate fails.
The old marker name `KARTO_ACCEPTED_LOOP_CLOSURE` overstated what it proved.
The optional probe was corrected on 2026-10-08 to report the gate separately.
The final
serialized pose graph returned `result=0` and is at
`/tmp/jetauto_loop_route_probe_20261005/final_map.{posegraph,data}`.
The occupancy map exported successfully at 216×219 cells, 0.05 m/cell, to
`final_map.{pgm,yaml}` in the same directory. Export success and visible room
walls are not accuracy checks.

## Accuracy against Isaac odometry

`rosbag/analyze_mapping_pose.py` matches each mapper `/pose` stamp to the
nearest unmodified Isaac `/odom/wheel` stamp, with a 0.02 s maximum skew.
It compares map-frame coordinates directly with odometry because mapping
started with the two frames aligned: the first pose at `(-0.5,-1.0)` matched
the reference exactly. No fitted transform is used to hide a map-frame jump.
All 1,234 poses matched within 5.6 ms.

| Full replay | Median | 95th percentile | Maximum |
| --- | ---: | ---: | ---: |
| Position error | 0.469 m | 2.161 m | 2.237 m |
| Heading error | 0.033 rad | 0.162 rad | 0.774 rad |

The decisive local comparison in this replay is around scan ID 557. The
focused retry linked scan ID 559 at 123.55 s instead, showing that the exact
event timing is replay-sensitive:

| Simulated time | Loop closing enabled | Loop closing disabled |
| ---: | ---: | ---: |
| 123.15 s | 0.184 m | 0.165 m |
| 123.35 s | **1.550 m** | **0.164 m** |
| 136.85 s | 2.237 m | 0.640 m |

For the control, the first 160 s of the same raw bag was copied to
`/tmp/jetauto_loop_route_first160_20261005` by retaining messages through
`/clock=160 s`. On isolated domain 89, the same mapper and configuration
were run with only `do_loop_closing:=false`; a parameter query confirmed it.
The control `/pose` bag is
`/tmp/jetauto_loop_route_probe_20261005/no_loop_pose_bag` (722 poses).
In that first-160-s window, position error was 0.192 m median, 0.943 m at
the 95th percentile, and 0.986 m maximum without closure; with closure it
was 0.192/2.162/2.237 m across 712 poses. The control still has substantial
scan-matching error, so disabling loop closing is not a validated production
fix. The matching timestamps and absence of the 123.35 s jump in the control
strongly implicate the false match/link in the large discontinuity.

Recompute the metrics after sourcing ROS Humble:

```bash
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_loop_closure_route_20261005/bag \
  /tmp/jetauto_loop_route_probe_20261005/pose_bag \
  --end-s 160 --sample-time-s 123.15 --sample-time-s 123.35 --sample-time-s 136.85
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_loop_closure_route_20261005/bag \
  /tmp/jetauto_loop_route_probe_20261005/no_loop_pose_bag \
  --end-s 160 --sample-time-s 123.15 --sample-time-s 123.35 --sample-time-s 136.85
```

## Status and next check

All replay, mapper, and recorder processes are stopped. The optional
diagnostic patch was removed from the Git-ignored vendor checkout and the
heading-fix-only SLAM overlay rebuilt; the installed library no longer
contains the diagnostic marker. No production SLAM parameter, Nav2
controller, physical driver, or original bag was changed.

Before using an Isaac map for Nav2 control, test a justified change against
the false match documented in the focused diagnosis and the pre-existing
no-loop error. Evaluate **pose accuracy**, not merely the count of fine
acceptances. Repeat live network capture on the same LAN when available.
