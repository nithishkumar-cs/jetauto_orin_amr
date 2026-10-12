# Full Isaac route at 1 m loop search, then diagonal-route localization — 2026-10-12

## Outcome

The complete saved two-lap Isaac bag replayed on the Jetson with
`loop_search_maximum_distance=1.0`. All 1,328 published mapping poses matched
Isaac odometry; position error was **0.092 / 0.445 / 0.562 m**
(median / p95 / maximum). The earlier full 3.0 m replay reported
0.469 / 2.161 / 2.237 m across 1,234 poses, so the two distributions are
not identical-sample comparisons. The new map and pose graph exported
successfully.

The same Jetson then loaded that pose graph and localized on the already
recorded **diagonal validation route**. Its 132 fresh poses had
**0.031 / 0.055 / 0.061 m** position error. This is a useful different-route
check in the same Isaac room, not a new-scene or hardware validation. No
Nav2 motion control was enabled, and the production SLAM YAML remains
unchanged.

A second replay of that diagonal route injected a ramped odometry TF bias
up to 0.671 m and 0.12 rad. Fresh localization still had
**0.029 / 0.056 / 0.065 m** position error (median / p95 / maximum),
similar to the zero-drift replay. This tests one synthetic bias profile,
not real wheel slip or sensor noise.

## Full mapping replay

Input: `/home/nithish/jetauto_loop_closure_route_20261005/bag`, containing
5,298 scans, exact `/odom/wheel`, `/clock`, and the odom/base/LiDAR TF.
The bag lasts 539 s; odometry shows the robot back at the start and
stationary by 350 s, with an idle tail. Playback was 1× on isolated ROS
domain 102. The pinned SLAM Toolbox 2.6.10 overlay used the installed
mapping YAML, `use_sim_time=true`, `do_loop_closing=true`, and only the
experimental 1.0 m search-radius override. The optional diagnostic patch
logged matched relative poses and newly created Karto graph edges.

| Mapper output | Result |
| --- | ---: |
| `/pose` messages matched to Isaac odometry | 1,328 / 1,328 |
| Position error, median / p95 / maximum | 0.0922 / 0.4445 / 0.5619 m |
| Heading error, median / p95 / maximum | 0.0175 / 0.0910 / 0.1188 rad |
| Maximum pose/odometry timestamp skew | 0.0056 s |
| Explicit fine-accepted loop candidates | 9 |
| Passed link gate / newly inserted graph edges for those candidates | 9 / 9 |
| Their relative-translation error against Isaac truth | 0.0142–0.1229 m |
| Their relative-heading error | at most 0.0174 rad |

The nine explicit `TryCloseLoop` candidates occurred from 24.55 to
143.25 s; none occurred on the second lap. One was a genuine near-revisit:
scan 427 at 94.15 s linked to scan 355 at 78.45 s, with Isaac scan centers
0.359 m apart and a 0.123 m relative-translation error. Karto actually
created that graph edge, rather than merely passing a response threshold.

This does **not** mean the second lap had no graph constraints. The new
diagnostic marker logged 3,088 created edges in total. Matching scan IDs
to their `/pose` stamps shows **1,006** edges from scans before 160 s to
scans at or after 160 s; **84** of those end at or after 300 s. For example,
the final scan at 315.15 s gained an edge from scan 1 at 0.65 s. Source
inspection suggests Karto's normal near-chain matching path for these
long-range links; the edge marker itself does not label the code path.
The marker proves insertion and temporal span, **not** the ground-truth
accuracy of every such constraint.
Thus inter-lap graph connectivity is demonstrated, while a second-lap
explicit `TryCloseLoop` acceptance is not.

The pose graph serialization service returned `result=0`, writing
`final_map.{posegraph,data}`. The map saver succeeded with a
**169×132-cell, 0.05 m/cell** occupancy map at origin
`(-2.19, -3.42) m`, an 8.45×6.60 m extent around the scene's nominal
8×6 m room. Visual inspection shows one room perimeter and the fixed
obstacles without an obvious doubled wall. This is a map sanity check,
not a quantitative occupancy-ground-truth comparison.

Mapping artifacts are under
`/tmp/jetauto_large_base_full_radius1_20261012/`: `mapper_console.log`,
`pose_bag`, `final_map.pgm`, `final_map.yaml`, `final_map.posegraph`, and
`final_map.data`. They are local temporary files and may disappear after a
restart. The original bag was not changed.

## Localization on the recorded diagonal route

The source bag was
`/home/nithish/jetauto_live_validation_20260930/localization_bag_retry`.
A **new** bag at `/tmp/jetauto_diagonal_route_clean_20261012` retained
`/clock`, `/scan`, `/tf_static`, and unchanged `odom -> base_link` TF;
it removed 1,536 old `map -> odom` transforms and did not replay the old
`/pose` or `/map`. Zero drift was requested in
`rosbag/inject_odometry_tf_drift.py`, and the untouched 9,582 odometry
samples were preserved as `/validation/reference_odom`. This prevents the
previous localizer output from contaminating the test.

On isolated ROS domain 103, `localization_slam_toolbox_node` loaded the new
`final_map` pose-graph prefix with `use_sim_time=true` and
`map_start_pose=[-0.5, -1.0, 0.0]`. The filtered route bag played at 1×.
The node published 132 fresh `/pose`, 147 `/map`, and 1,422 `map -> odom`
transforms. Every pose matched exact Isaac odometry within 0.0036 s.

| Localizer metric | Median | P95 | Maximum |
| --- | ---: | ---: | ---: |
| Position error | 0.0307 m | 0.0549 m | 0.0614 m |
| Heading error | 0.0120 rad | 0.0283 rad | 0.0697 rad |

The last fresh pose at 84.75 s was near the route's final orientation:
estimated yaw was -0.0087 rad versus Isaac's 0.0 rad after the final turn.
Its position error was 0.0235 m. The route-clearance analyzer placed all
9,582 reference odometry centers inside free map cells, none outside the
map, and found a 0.765 m minimum center-to-non-free-cell-center distance.
That is a map-cell check, **not** a robot-footprint collision certificate.

Localization artifacts are under
`/tmp/jetauto_diagonal_new_map_localization_20261012/`.

## Controlled odometry-drift replay

The same source bag was transformed again with
`rosbag/inject_odometry_tf_drift.py`, ramping `odom -> base_link` by
`(+0.60, -0.30) m` and `+0.12 rad` over the first 45 simulated seconds.
The new input bag at `/tmp/jetauto_diagonal_drift_20261012` removed the same
1,536 old `map -> odom` transforms and old `/pose`/`/map`, kept the original
9,582 odometry samples separately, and did not alter the source bag. The
normal, probe-free localization overlay loaded the new map and replayed this
bag at 1× on isolated domain 104.

| Drift-run measure at fresh `/pose` timestamps | Median | P95 | Maximum |
| --- | ---: | ---: | ---: |
| Localized position error (136 matched poses) | 0.0294 m | 0.0558 m | 0.0650 m |
| Localized heading error | 0.0115 rad | 0.0265 rad | 0.0291 rad |
| Raw input-TF position drift from Isaac truth | 0.6425 m | 0.6708 m | 0.6708 m |
| Raw input-TF heading drift | 0.1149 rad | 0.1200 rad | 0.1200 rad |

The 136/136 fresh poses matched reference odometry and biased TF within
0.02 s; maximum pose/reference skew was 0.004 s. The final pose at 84.75 s
was 0.0171 m from Isaac truth. Different callback timing produced 136 poses
instead of the zero-drift run's 132, so the distributions are not
identical-sample comparisons. Raw drift was measured from the biased
`odom -> base_link` TF against the unmodified `/odom/wheel` at each output
pose stamp; localized pose error was measured independently against the same
unmodified odometry. Output is under
`/tmp/jetauto_diagonal_new_map_drift_localization_20261012/`.

## Reproduction and remaining decision

The optional [Karto probe](../../patches/slam_toolbox/2.6.10-loop-closure-probe.patch)
was applied only for validation, then reversed. The overlay was rebuilt
with only the tracked heading-aware scan fix; its installed Karto library
contains no diagnostic probe markers. The mapper and localizer processes
started for these checks are stopped. Six unit tests for the
[relative-pose audit](../../rosbag/audit_karto_loop_matches.py) pass.

After sourcing ROS Humble and the workspace overlay, the main checks are:

```bash
python3 rosbag/audit_karto_loop_matches.py \
  /home/nithish/jetauto_loop_closure_route_20261005/bag \
  /tmp/jetauto_large_base_full_radius1_20261012/mapper_console.log
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_loop_closure_route_20261005/bag \
  /tmp/jetauto_large_base_full_radius1_20261012/pose_bag
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_live_validation_20260930/localization_bag_retry \
  /tmp/jetauto_diagonal_new_map_localization_20261012/output_bag
python3 rosbag/analyze_mapping_pose.py \
  /home/nithish/jetauto_live_validation_20260930/localization_bag_retry \
  /tmp/jetauto_diagonal_new_map_drift_localization_20261012/output_bag
```

For the drifted input, the bag transformation was:

```bash
python3 rosbag/inject_odometry_tf_drift.py \
  /home/nithish/jetauto_live_validation_20260930/localization_bag_retry \
  /tmp/jetauto_diagonal_drift_20261012 \
  --x-m 0.60 --y-m -0.30 --yaw-rad 0.12 --ramp-duration-s 45
```

The mapping replay launched `async_slam_toolbox_node` with the installed
`configs/localization/slam_mapping.yaml` and the three overrides above,
recorded `/pose /map`, and played the raw bag without `--clock` (the bag
already contains `/clock`). The localization replays launched
`localization_slam_toolbox_node` with
`configs/localization/slam_localization.yaml`, `use_sim_time=true`,
`map_file_name=/tmp/jetauto_large_base_full_radius1_20261012/final_map`,
and `map_start_pose=[-0.5,-1.0,0.0]`, then recorded `/pose /map /tf`.
The temporary launch and shell harnesses are under `/tmp` and are not part
of the production bringup.

The 1.0 m radius remains a **test setting**, not a production default.
The original and diagonal bags share a simulator scene and exact odometry;
replay timing affects scan selection and output counts. A distinct scene,
newly generated route, realistic odometry drift/noise, and Nav2 planning
and controlled-motion checks are still needed before changing the default
or calling this navigation-ready. A distinct scene requires a new Isaac
run; the validation above required only existing bags on the Jetson.
