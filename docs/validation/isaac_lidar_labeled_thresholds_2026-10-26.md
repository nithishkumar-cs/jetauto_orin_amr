# Static-scene LiDAR clustering labels — 2026-10-26

## Input and labeling

The 2026-09-28 mapping bag has 1,070 scans over 107.2 s of simulated time.
The robot drove two laps and then remained parked; 812 scans show base motion.
This run disabled the moving person. Its static room is specified by the Jetson
Isaac snapshot `~/projects/robotics/jetauto_isaac_sim/isaac/standalone/jetauto_scene.py`
(2026-09-30): two boxes, four chair legs at the 0.24 m LiDAR height, and the
room walls. The script `tests/integration_tests/score_lidar_scene_clusters.py`
stores those planar bounds, interpolates the bag's `odom -> base_link` TF at
each scan stamp, adds the 0.15 m LiDAR mount offset, and raycasts the first
scene surface. A recorded return is labeled only when its range is within
0.12 m of that first surface. This labels 97.8% of valid returns. The scorer
rejects TF gaps over 0.2 s; all 1,070 scans were admitted.

The scorer mirrors the C++ clusterer's invalid-beam breaks, Euclidean
adjacent-point gap, full-circle seam join, and minimum-point filter. For the
checked-in 0.20 m/three-point baseline, its cluster count matched all 1,067
available `/perception/lidar/clusters` arrays in the earlier real-node replay
bag, scan stamp by scan stamp. The three missing replay outputs were not
treated as clustering failures.

Each box is expected in a scan if at least three of its recorded returns match
the scene model. A box counts as captured at 50% or 80% when one cluster is at
least 80% box-labeled points and contains that fraction of the box's labeled
returns. An extra substantial piece (at least 10% of the box's returns and
three points) counts as fragmentation. A cluster containing at least three
points from the box and three from another modeled surface counts as a merge.
Chair legs are evaluated as narrow *features*, not four separate objects: each
leg with at least two labeled returns is retained if a cluster has at least two
of its returns and is at least 80% that leg. A cluster with no scene-matched
return is called unsupported; this is a model-coverage diagnostic, not a
measured false-positive rate.

## Results

The gap sweep below fixes the two-point minimum. Counts are over 2,140
box-scan observations (two boxes in every scan) and 2,888 visible chair-leg
features. `Split` counts extra substantial box pieces; `merge` counts box
frames joined to another modeled surface.

| Gap (m) | Box capture ≥50% | Box capture ≥80% | Split | Merge | Leg retained | Unsupported clusters/scan |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.10 | 1,904 | 1,179 | 1,343 | 0 | 2,128 | 0.57 |
| 0.15 | 2,058 | 1,445 | 1,036 | 0 | 2,129 | 0.45 |
| **0.20** | **2,062** | **1,467** | **1,002** | **0** | **2,129** | **0.42** |
| 0.25 | 2,061 | 1,466 | 1,002 | 0 | 2,129 | 0.40 |
| 0.30 | 2,061 | 1,466 | 1,002 | 0 | 2,128 | 0.40 |
| 0.35 | 2,061 | 1,466 | 1,001 | 0 | 2,055 | 0.40 |
| 0.40 | 2,060 | 1,465 | 999 | 3 | 1,766 | 0.39 |
| 0.45 | 2,046 | 1,452 | 985 | 34 | 1,491 | 0.37 |
| 0.50 | 1,984 | 1,393 | 958 | 129 | 1,445 | 0.35 |

At the 0.20 m gap, varying the minimum leaves box capture and substantial
fragmentation unchanged until eight points. The apparent reduction in
fragmentation at higher minima mostly discards small valid pieces. It also
discards narrow chair-leg features:

| Minimum points | Box capture ≥50% | Split | Leg retained | Clusters/scan | Unsupported clusters/scan |
| ---: | ---: | ---: | ---: | ---: | ---: |
| **2** | **2,062** | 1,002 | **2,129/2,888 (73.7%)** | 19.33 | 0.42 |
| 3 (previous) | 2,062 | 1,002 | 1,351/2,888 (46.8%) | 16.95 | 0.23 |
| 5 | 2,062 | 847 | 884/2,888 (30.6%) | 14.76 | 0.13 |
| 8 | 2,061 | 480 | 333/2,888 (11.5%) | 12.28 | 0.05 |

The 0.20 m gap is the best point in the tested plateau: it has the highest
50% and 80% box capture, retains narrow geometry, and produces no modeled
box merge. The difference from 0.25–0.35 m is only one box frame, so it is
not evidence for a precisely optimal gap. The two-point minimum gains 778
modeled chair-leg observations over the previous three-point minimum without
reducing box capture. It adds 2.38 clusters/scan, of which about 0.18/scan
have no scene-matched return. The checked-in minimum is therefore now two;
the gap remains 0.20 m.

Changing the labeling tolerance to 0.08 or 0.16 m preserved the large loss at
0.10 m, the merge rise above 0.40 m, and the narrow-feature gain from two
points. At 0.08 m, 0.15 m gap captured 14 more box frames than 0.20 m, while
0.20 m had 29 fewer extra fragments. This is a near-tie, not a stable ranking
between those two gaps.

## Follow-up checks: time split, TF offset, and small clusters

The checked-in defaults changed in both `configs/perception/lidar_clustering.yaml`
and the standalone C++ `LidarClustererConfig`: the gap is 0.20 m and the minimum
is two points. The first and second 535 scans were scored separately to check
whether the choice depended on one part of the recording. With a two-point
minimum, the 0.20 m gap captured 1,029 and 1,033 of 1,070 expected box views
at 50%, versus 1,031 and 1,027 for 0.15 m. At 80%, 0.20 m captured 785 and
682, versus 780 and 665 for 0.15 m. It produced 427 and 575 extra substantial
box pieces, versus 433 and 603. The two-point minimum retained 1,205/1,658
and 924/1,230 visible chair-leg features; three points retained 987/1,658 and
364/1,230. The second half includes 258 parked scans, so this is a split of
one route and scene, not independent validation data.

To probe scan/TF alignment, TF was queried at each scan stamp shifted by
-0.10, -0.05, +0.05, and +0.10 s, holding the measured scan points fixed.
Model-matched valid returns fell from 97.8% at zero offset to 91.2%, 94.7%,
97.0%, and 94.2%, respectively. At these offsets, the 0.20 m gap captured
17, 21, 7, and 9 more box views at 80% than 0.15 m, with 28, 28, 36, and
38 fewer extra substantial fragments. At 50% capture, 0.15 m won by seven
and four views at the two positive offsets; 0.20 m won by five at both
negative offsets. The two-point minimum retained 731, 734, 683, and 621 more
visible leg features than three points, respectively. Other nearby gaps
remain near-tied; these checks support 0.20 m as a stable plateau choice,
not a uniquely optimized threshold or an estimate of an actual clock offset.

At nominal timing, two points admit 2,543 additional clusters over the
three-point minimum, all of them two-return groups. Of these, 197 have **no**
return matched to the static scene model (448 unsupported clusters versus
251 at three points, across 1,070 scans). The other 2,346 have at least one
modeled return, which does not establish that every such cluster is a real
obstacle. Likewise, unmatched clusters may reflect model omissions or pose
error; 197 is an unsupported-cluster count, not a false-positive count.

## Scope and reproduction

This selects clustering settings for the recorded **static Isaac scene**.
Model agreement does not prove a noise or false-positive rate. The chair legs
are geometry features, not independent semantic obstacles. Box fragmentation
is still common (1,002 extra substantial pieces across 2,140 observations);
changing only these two parameters does not solve every split. The bag has no
moving target, and it cannot calibrate LiDAR tracker motion classification,
camera tracking, fusion, or a physical LiDAR. The separate reconstructed
moving-person bag has an unresolved time offset and was not pooled with these
static labels. Recheck on exported Isaac ground truth and physical LiDAR data
before broader claims.

The later [two-versus-three tracker replay](isaac_lidar_tracker_minimum_comparison_2026-10-26.md)
found that confirmation did not absorb all added output: 159 more confirmed
track observations on 68 shared moving-person frames, with unchanged modeled
person recall. Those extra observations are not measured false positives.

```bash
source /opt/ros/humble/setup.bash
python3 tests/integration_tests/score_lidar_scene_clusters.py \
  ~/jetauto_isaac_mapping_loop_20260928 > /tmp/lidar_scene_scores.json
# Reproduce each half with --scan-range 0 535 and --scan-range 535 1070.
# Reproduce offset checks with --tf-offset-s -0.10, -0.05, 0.05, or 0.10.
```

Raw bags and score JSON remain outside Git. The source bag and existing
real-node replay outputs are described in
`docs/validation/isaac_lidar_threshold_sweep_2026-10-14.md`.
