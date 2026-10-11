# Isaac loop-match relative-pose audit — 2026-10-11

## Result

The earlier claim that separated scan centers prove a false loop match was
incorrect. Karto stores a **relative-pose constraint** between scans, so
overlapping LiDAR views from different robot positions can link correctly.
This audit compares each proposed relative transform with Isaac's exact
odometry in the LiDAR frame. It confirms the damaging original-bag match was
wrong (2.176 m relative-translation error), while a narrower **1.0 m loop
search radius** produced only small relative-transform errors in the tested
replays. The 1.0 m setting is promising, but this is a replay experiment,
not a production configuration change or Nav2-control validation.

## Method

The optional [Karto diagnostic probe](../../patches/slam_toolbox/2.6.10-loop-closure-probe.patch)
now prints the prior, matched, and closest old scan's sensor poses immediately
before `LinkChainToScan`, along with the existing link-distance gate result.
The [audit script](../../rosbag/audit_karto_loop_matches.py) pairs each scan
timestamp to exact Isaac `/odom/wheel` within 0.02 s, applies the scene's
0.15 m forward `base_link -> lidar_link` offset, and compares the proposed
old-to-current relative pose with the odometry-derived one. This removes
global map translation/rotation from the comparison. A gate pass means Karto
*attempts* a link, not that `AddEdge` inserted a new graph edge.

The original raw bag is
`/home/nithish/jetauto_loop_closure_route_20261005/bag`. Its 0–160 s
replay input is `/tmp/jetauto_loop_route_first160_20261008`. The asymmetric
landmark raw bag is
`/home/nithish/jetauto_false_loop_landmarks_20261008/bag_final`.
The first bag was replayed at 1.0× and the landmark bag at 1.8×, the same
rates used by the previous diagnostic work. Each mapper was launched with
`use_sim_time=true`, `do_loop_closing=true`, the installed mapping YAML, and
only the listed search-radius override. The normal fine-response threshold
was 0.45 and the link-distance gate was 1.5 m in the pinned mapper wrapper;
the search radius was confirmed on each running node. No source bag was
modified.

| Bag / search radius | Fine-accepted candidates | Gate passes | Relative-position error of gate passes | `/pose` position error median / p95 / max |
| --- | ---: | ---: | --- | --- |
| Original, 3.0 m, first 160 s | 5 | 1 | **2.176 m** | 0.194 / 1.896 / 1.974 m (717 poses) |
| Original, 1.0 m, first 160 s | 9 | 9 | 0.016–0.115 m | 0.104 / 0.510 / 0.541 m (712 poses) |
| Landmarks, 3.0 m, full 315 s | 188 | 5 | 0.026, 0.359, 0.373, **2.289**, **4.261** m | 0.461 / 2.520 / 2.692 m (1,063 poses) |
| Landmarks, 1.0 m, full 315 s | 7 | 7 | 0.011–0.213 m | 0.092 / 0.519 / 0.623 m (1,250 poses) |
| Landmarks, 1.0 m, repeat, full 315 s | 6 | 6 | 0.012–0.258 m | 0.103 / 0.584 / 0.659 m (1,244 poses) |

The `/pose` counts differ, so aggregate statistics are not identical-sample
comparisons. All compared `/pose` stamps were within 5.6 ms of Isaac
odometry. The 3.0 m landmark replay was worse than the earlier same-bag run
documented in the [landmark report](isaac_landmark_loop_replay_2026-10-09.md),
showing replay/timing sensitivity; neither run should be treated as a
deterministic performance estimate.

## Why the previous gate-distance reasoning failed

In the original 3.0 m replay, scans 555–558 at 122.75–123.35 s produced
7–9 cm relative-translation errors but **failed** the 1.5 m matched-pose
gate. Scan 559 at 123.55 s passed the gate, despite a **2.176 m** relative
translation error. Its fine response was 0.565 and it moved the prior scan
estimate by 1.989 m. The mapper's `/pose` error jumped from 0.203 m to
1.502 m at that time. This directly establishes a damaging wrong match;
scan-center distance alone did not.

The landmark 3.0 m replay had mixed outcomes. The gate pass at 142.05 s
corrected the prior by **2.150 m** yet its relative-translation error was
only **0.026 m**. A simple maximum-pose-jump filter would therefore reject
a useful correction. Two gross errors, at 124.25 s and 176.55 s, had fine
responses 0.575 and 0.698. Conversely, the two 0.359–0.373 m-error passes
scored 0.947 and 0.969. Conversely, accurate 1.0 m-run matches scored as low
as 0.665 (original bag) and 0.677 (landmark bag). A simple score threshold
would reject some correct lower-scoring matches yet still accept those less
accurate high-scoring ones.

The 1.0 m runs retained gate-passing links with bounded relative-transform
errors; the landmark repeat gave a similar trajectory-error range despite
different accepted scan IDs and counts. Both bags included a genuine
near-revisit around 94.15 s: the
selected scans were 0.359 m apart in Isaac truth, and the proposed relative
translation was within 0.115 m (original bag) or 0.123 m (landmark bag).
They also correctly linked overlapping scans whose centers were 1.6–2.9 m
apart. The search-radius parameter restricts *candidate selection*; it is
not a guarantee that every eventual graph constraint is true.

## Decision and limits

Keep the production SLAM YAML unchanged for now. A 1.0 m candidate-search
radius is the leading test configuration, not a safety or localization
guarantee. The second landmark replay had similar but not identical metrics;
that is useful repeat evidence, not an independent scene. Before promoting
the setting, replay the full original two-lap bag, test a distinct scene and
route, inspect actual graph-edge insertion and saved-map quality, and verify
localization and Nav2 behavior separately. These bags use exact Isaac
odometry and do not exercise hardware wheel/IMU drift or real-sensor noise.

Diagnostic output and `/pose` bags are at:

- `/tmp/jetauto_large_base_radius3_pose_trace_20261011/`
- `/tmp/jetauto_large_base_radius1_pose_trace_20261011/`
- `/tmp/jetauto_landmarks_radius3_pose_trace_20261011/`
- `/tmp/jetauto_landmarks_radius1_pose_trace_20261011/`
- `/tmp/jetauto_landmarks_radius1_repeat_20261011/`

Recompute one audit with:

```bash
source /opt/ros/humble/setup.bash
python3 rosbag/audit_karto_loop_matches.py \
  /home/nithish/jetauto_false_loop_landmarks_20261008/bag_final \
  /tmp/jetauto_landmarks_radius1_pose_trace_20261011/mapper_console.log
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_false_loop_landmarks_20261008/bag_final \
  /tmp/jetauto_landmarks_radius1_pose_trace_20261011/pose_bag
```

These `/tmp` artifacts are local and may disappear after restart. The probe
is optional diagnostic code. It was reversed after these experiments, and
the heading-fix-only overlay rebuilt successfully. The installed Karto
library contains no diagnostic probe markers. No production mapping YAML,
Nav2 controller, source bag, or physical base was changed.
