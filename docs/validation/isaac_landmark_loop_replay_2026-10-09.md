# Isaac landmark variant: loop-match audit — 2026-10-09

## Outcome

The opt-in asymmetric LiDAR landmarks avoided the **specific** false link
seen near 123.55 s in the original bag, but they did **not** establish
trustworthy automatic loop closure. Three later matches passed Karto's 1.5 m
matched-pose link gate. This report originally called all three false because
Isaac placed their scan centers 2.37–3.64 m apart; that inference was wrong.
Karto's graph constraint is a **relative pose**, which can be valid between
separated but overlapping LiDAR scans. A later
[relative-pose audit](isaac_loop_match_relative_audit_2026-10-11.md) found
both accurate and grossly wrong matches in a fresh replay. This map is not
cleared for Nav2 motion control.

This extends the [original false-match diagnosis](isaac_false_loop_match_2026-10-08.md).
The Isaac-side variant is local commit `7283b35` in the separate simulator
repository. Neither the simulator change nor this Jetson work has been
pushed as part of this validation.

## Inputs and method

The original raw bag is
`/home/nithish/jetauto_loop_closure_route_20261005/bag`; the landmark bag
is `/home/nithish/jetauto_false_loop_landmarks_20261008/bag_final`. The
landmark bag contains 3,150 `/scan` messages and 18,923 each of `/clock`,
`/tf`, and `/odom/wheel`, plus three `/tf_static` messages. Its simulation
timestamps run to 315.43 s. **All 18,923 landmark odometry samples have an
exact timestamp, XY pose, and yaw match in the original bag.** Thus the
route and odometry input are identical through the landmark recording; the
LiDAR-visible scene is the intended difference. The original bag continues
past that interval.

On isolated ROS domain 92, the pinned SLAM Toolbox 2.6.10 overlay ran with
the normal mapping configuration and the optional
`patches/slam_toolbox/2.6.10-loop-closure-probe.patch`. Parameter queries
confirmed `do_loop_closing=true`, `loop_search_maximum_distance=3.0`, and
`link_scan_maximum_distance=1.5`. The landmark bag played at 1.8× recorded
wall time: the Tailscale recording spans 577 wall seconds but only 315
simulation seconds, so this approximated 1× simulation-time progression.
The recorder captured 1,094 mapper `/pose` messages. The console log and
pose bag are under `/tmp/jetauto_landmarks_replay_20261009_retry/`.

The same landmark bag was then replayed at the same rate on isolated domain
94 with only `do_loop_closing=false`; that override was queried and
confirmed. It produced 1,401 `/pose` messages under
`/tmp/jetauto_landmarks_no_loop_20261009_retry3/`. These `/tmp` artifacts
are local and may disappear after a restart. The prior original-bag first-160-s
replay at 1×, with loop closing enabled, is
`/tmp/jetauto_loop_trace_20261008_retry/pose_bag`. No map, Nav2 controller,
physical base, or production SLAM parameter was changed.

## Pose error against Isaac truth

`rosbag/analyze_mapping_pose.py` paired each `/pose` header stamp with the
nearest `/odom/wheel` stamp; every pose below matched within 5.6 ms. The
first-160-s rows use the common simulated-time window, but the mapper
published different numbers of poses, so these are not identical-sample
statistics.

| Run | Poses | Position error median / p95 / max | Heading error median / p95 / max |
| --- | ---: | ---: | ---: |
| Original bag, loop on, first 160 s | 717 | 0.194 / 1.896 / 1.974 m | 0.030 / 0.165 / 0.194 rad |
| Landmark bag, loop on, first 160 s | 597 | 0.495 / 1.172 / 1.230 m | 0.073 / 0.314 / 0.338 rad |
| Landmark bag, loop off, first 160 s | 723 | 0.664 / 1.432 / 1.528 m | 0.265 / 0.422 / 0.454 rad |
| Landmark bag, loop on, full 315 s | 1,094 | 0.180 / 1.228 / 1.776 m | 0.035 / 0.303 / 0.338 rad |
| Landmark bag, loop off, full 315 s | 1,401 | 1.098 / 2.231 / 2.415 m | 0.408 / 0.635 / 0.716 rad |

At 123.55 s, the original replay jumped from 0.203 m to 1.502 m position
error. The landmark replay had no analogous gate pass there, but its nearest
pose was already 0.757 m wrong at 123.35 s. At the first later gate pass,
the landmark run improved from 1.172 m at 140.35 s to 0.084 m at 141.55 s;
the no-loop control was 1.436 m wrong near 141.55 s. The landmark loop-on
run subsequently reached its 1.776 m maximum near 224 s. It ended near
ground truth, whereas the no-loop run ended 1.501 m away. This demonstrates
the impact of the loop-closing path, **not the validity of its matches**.

## Gate-passing matches: center distance alone was inconclusive

The probe logged 171 fine-accepted candidates, but only three passed the
subsequent link-distance gate. For all 171 fine-accepted log entries, Karto's
current scan ID equals the `/pose` array index and its logged scan time
matches that pose's header stamp within 0.011 s. This establishes the same
ID-to-time mapping for the older candidate IDs below. Isaac `/odom/wheel`
then gives:

| Current scan | Older scan selected | Matched-pose gate distance | Isaac true separation |
| --- | --- | ---: | ---: |
| ID 565, 141.55 s | ID 27, 4.95 s | 1.385 m | **2.373 m** |
| ID 655, 180.15 s | ID 248, 57.15 s | 0.887 m | **3.638 m** |
| ID 727, 212.95 s | ID 456, 102.35 s | 1.459 m | **3.586 m** |

The matched-pose distances are square roots of the probe's logged squared
distances (1.918, 0.788, and 2.130 m²); they are *not* ground-truth
distances. Karto's gate checks where the scan matcher placed the current
scan, so a false match can pass it. A gate pass means `LinkChainToScan` will
attempt a link; the probe does not independently log whether `AddEdge`
inserted a new constraint. The table proves these are not *same-position*
revisits, but it does **not** prove their relative-pose constraints are false.
The subsequent [audit](isaac_loop_match_relative_audit_2026-10-11.md)
compares proposed relative transforms with Isaac ground truth. Do not count
these gate passes as verified correct closures either.

## Reproduction and next decision

Source ROS Humble and this workspace overlay. Apply the optional probe to
the pinned vendor checkout, rebuild with
`scripts/setup/build_slam_toolbox_heading_fix.sh`, launch
`localization_slam_mapping` with `backend:=rosbag` on an isolated domain, and
record `/pose`. Play the landmark bag with `ros2 bag play BAG --rate 1.8`
**without `--clock`**; the bag already has `/clock`. Compare poses with:

```bash
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_false_loop_landmarks_20261008/bag_final \
  /tmp/jetauto_landmarks_replay_20261009_retry/pose_bag
```

For the control, use the same mapper configuration with only
`do_loop_closing=false`. Afterward reverse the optional probe and rebuild.
That cleanup was done for this run: the installed mapper again contains only
the tracked heading-aware scan fix.

The [subsequent audit](isaac_loop_match_relative_audit_2026-10-11.md) tests
candidate selection and match displacement against Isaac truth on both bags.
Do not promote a score or distance threshold from this single replay into
production configuration.
