# Isaac mapping loop-closure probe — 2026-10-01

## Result

The existing three-lap Isaac mapping run did **not** demonstrate an accepted
automatic SLAM Toolbox loop closure. In a replay with diagnostics at Karto's
actual `MapperGraph::TryCloseLoop` path, no eligible candidate chain was found.
There was consequently no coarse match, fine match, or accepted-closure event.
The long-range edges in the previously saved graph remain insufficient as
proof of an accepted closure.

This is a negative result for this route and configuration, not evidence that
SLAM Toolbox's loop-closing feature is broken. The recorded route spans only
3.178 m in X and 2.08 m in Y, or about 3.80 m corner-to-corner. The ROS
wrapper sets `loop_search_maximum_distance` to 3.0 m and
`link_scan_maximum_distance` to 1.5 m when our YAML omits them, overriding
Karto's internal defaults. Its candidate search excludes near-linked scans;
the compact, repeatedly traversed path may keep the old scans in that
excluded set. This explains the observation plausibly, but the probe did not
inspect every rejected scan link, so it is an inference rather than a proven
cause.

## Method and evidence

The source bag was
`/home/nithish/jetauto_live_validation_20260930/mapping_bag`: 2,189 scans,
three rectangular laps, and 30.38 m traveled. A zero-drift copy under
`/tmp/jetauto_mapping_loop_replay_20261001` retained `/clock`, `/scan`, and
source TF, removed 2,348 previously recorded `map -> odom` transforms, and
excluded previously recorded `/pose` and `/map`. The copy was made with
`rosbag/inject_odometry_tf_drift.py` using zero X/Y/yaw drift and a positive
one-second ramp duration.

The local, pinned SLAM Toolbox 2.6.10 build retained the separately validated
heading-aware scan filter. The optional diagnostic patch
`patches/slam_toolbox/2.6.10-loop-closure-probe.patch` prints a marker when
`FindPossibleLoopClosure` returns a nonempty chain, coarse scores for such a
chain, fine rejection, and an acceptance marker only **after**
`LinkChainToScan` and `CorrectPoses`. It also prints a heartbeat every 50 scan
IDs, so an empty log cannot be mistaken for an inactive mapper. This patch is
for diagnostics only and is not applied by the normal build script.

On isolated ROS domain 85, a 1.0× replay with an acceptance-only marker
completed without an accepted event. On domain 86, a 1.25× replay with the
full diagnostic patch completed. Its console log is
`/tmp/jetauto_loop_probe_logs_20261001/candidate_probe_console.log`.
The mapper registered the LiDAR and logged 12 heartbeat checks, at scan IDs
0, 50, ..., 550, all with `first_candidate_chain_size=0`. The patch would
also log every nonempty candidate regardless of scan ID; there are no such
markers, no coarse scores, and no accepted-closure markers in the complete
replay log. The player exited successfully and the mapper was then stopped.

The diagnostic edits were removed from the Git-ignored local vendor checkout
after the replay, and the normal heading-fix-only overlay was rebuilt.
`git -C src/localization/slam_toolbox_vendor diff --stat` then showed only
`src/slam_toolbox_common.cpp` changed by the tracked heading patch. No system
ROS package or robot driver was changed.

## Smaller-radius follow-up — 2026-10-05

The same filtered bag was replayed on isolated ROS domain 87 with the same
diagnostic patch and a single runtime override:
`loop_search_maximum_distance:=1.0`. A ROS parameter query confirmed the
node loaded `1.0`; the bag played to completion at 1.25×. Unlike the 3.0 m
run, this produced eligible candidate chains and three direct acceptance
markers from after `LinkChainToScan` and `CorrectPoses`:

| Simulated time | Scan ID | Candidate chain | Coarse response |
| ---: | ---: | ---: | ---: |
| 25.5501 s | 111 | 26 | 0.830014 |
| 57.0501 s | 255 | 10 | 0.773472 |
| 76.85 s | 345 | 20 | 0.872042 |

The route returned near its start at about 44–48 s, 92–95 s, and 139 s.
Thus the first accepted event was on lap one, while the other two were on a
repeated section of lap two. There was no further acceptance on lap three.
These markers prove that the automatic acceptance path executed; they do not
prove that every acceptance was a beneficial correction or that closure at
each complete lap was achieved.

The exported pose graph returned `result=0` and is stored at
`/tmp/jetauto_loop_probe_1m_20261005.{posegraph,data}`. A direct Nav2 map
saver call with a 10 s timeout exported
`/tmp/jetauto_loop_probe_1m_20261005.{pgm,yaml}`. The map was 166×125 cells
at 0.05 m/cell; the original run's map was 167×125 at the same resolution.
The two occupancy maps look similar, but a visual comparison and valid export
are not an accuracy or false-closure test. The built-in SLAM Toolbox
`save_map` service first returned `255` because its map-saver subprocess
timed out after 2 s; the direct map-saver retry succeeded.

The diagnostic console log is
`/tmp/jetauto_loop_probe_logs_20261005/search_radius_1m_console.log`.
This one-bag, exact-odometry parameter study does **not** authorize changing
the production YAML from its current 3.0 m wrapper default to 1.0 m. The
optional probe was removed again and the heading-fix-only overlay rebuilt.

## Reproduce or extend

Apply the optional probe to the pinned local vendor checkout, then rebuild:

```bash
bash scripts/setup/build_slam_toolbox_heading_fix.sh
git -C src/localization/slam_toolbox_vendor apply \
  "$PWD/patches/slam_toolbox/2.6.10-loop-closure-probe.patch"
bash scripts/setup/build_slam_toolbox_heading_fix.sh
```

Launch `localization_slam_mapping` with `backend:=rosbag` on an isolated ROS
domain, then play the filtered bag **without** `--clock` because the bag
already publishes `/clock`. Capture the mapper console output and look for
`KARTO_ACCEPTED_LOOP_CLOSURE`. After the test, reverse the optional patch and
rebuild the normal overlay:

```bash
git -C src/localization/slam_toolbox_vendor apply --reverse \
  "$PWD/patches/slam_toolbox/2.6.10-loop-closure-probe.patch"
bash scripts/setup/build_slam_toolbox_heading_fix.sh
```

The next meaningful test needs an Isaac route/map with a larger, distinctive
excursion out of the near-linked region followed by a return to an earlier
mapped area, plus a deliberate odometry-drift trial. Run with the same
diagnostics and measure map/pose accuracy after the accepted events. The
larger route needs a new simulator run; a controlled drift trial can start
from a recorded bag. Neither requires Nav2 motion control or a physical robot.
