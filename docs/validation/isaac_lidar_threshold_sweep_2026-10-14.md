# Offline Isaac LiDAR threshold sweep — 2026-10-14

The later [scene-labeled pass](isaac_lidar_labeled_thresholds_2026-10-26.md)
supersedes the clustering selection below: it retained the 0.20 m gap and
changed the minimum from three to two points for this static Isaac scene.
Tracker settings were not changed.

## Evidence and method

The recorded `jetauto_isaac_mapping_loop_20260928` bag has 1,070 scans over
190.8 seconds, plus `/clock`, moving TF, and odometry. This mapping-only Isaac
scene disabled the moving pedestrian and RGB-D render product. The scan-only
bags with 3,150 and 5,298 scans were also confirmed to contain `/scan`, but
they do not add labeled moving-object ground truth.

The existing `tests/integration_tests/replay_perception_chain.py` replayed the
raw topics at 2× through the real LiDAR clustering and tracking nodes. Each
candidate changed one parameter; all others used the checked-in configs.
`tests/integration_tests/summarize_lidar_replay.py` streamed the output bags
for counts. The eight existing cluster-parameter replays yielded 1,066–1,070
cluster arrays, or at least 99.6% of the source scan count. Output artifacts
are under `.agent-work/sw/` and are excluded from Git.

| Candidate | Clusters/scan | Tracks/message | Unique IDs | Dynamic output entries |
| --- | ---: | ---: | ---: | ---: |
| Current: 0.20 m gap, 3 points | 17.0 | 25.5 | 1,329 | 65.7% |
| Gap 0.10 m | 26.3 | 40.3 | 1,549 | 68.1% |
| Gap 0.15 m | 17.9 | 27.2 | 1,370 | 67.0% |
| Gap 0.30 m | 16.8 | 25.3 | 1,320 | 65.7% |
| Gap 0.50 m | 17.2 | 25.6 | 1,317 | 65.7% |
| Minimum 2 points | 19.3 | 28.2 | 1,369 | 61.9% |
| Minimum 5 points | 14.8 | 22.5 | 1,240 | 68.4% |
| Minimum 8 points | 12.3 | 18.8 | 1,076 | 70.0% |

Three additional replays varied only the LiDAR tracker. Their cluster counts
remained at 17.0 per scan. The output count varied by at most four messages
across runs.

| Tracker candidate | Tracks/message | Unique IDs | Median ID output frames | Dynamic output entries |
| --- | ---: | ---: | ---: | ---: |
| Current: 0.75 m gate, 0.50 s expiry, 2 hits | 25.5 | 1,329 | 11 | 65.7% |
| Association gate 0.35 m | 26.7 | 2,039 | 8 | 58.5% |
| Unobserved expiry 0.25 s | 19.0 | 1,923 | 5 | 65.1% |
| Confirmation 3 hits | 22.3 | 1,036 | 12 | 63.9% |

## Interpretation

The 0.10 m gap fragmented the same static scene into 55% more clusters than
the current 0.20 m gap. Values from 0.15 to 0.50 m changed aggregate counts
little; those counts cannot reveal whether a larger gap merges adjacent
objects. Requiring five or eight points filtered more clusters, but this bag
has no labeled narrow obstacles to establish the resulting missed-object rate.
The current 0.20 m gap and three-point minimum remain the best-supported
provisional settings, so the config was not changed.

The narrower association gate and shorter expiry fragmented identities: they
produced 53% and 45% more IDs than the current tracker, respectively. Three
confirmation hits suppressed 293 IDs from *output* without changing internal
associations, but would delay or omit short-lived obstacles. Without labeled
obstacle presence and identity, the lower output volume is not an accuracy
gain. The current tracker gate, expiry, and confirmation settings remain
provisional and unchanged.

Unique IDs and the dynamic percentage are diagnostic counts, not identity or
motion accuracy. The tracker treats LiDAR cluster centroids as objects; a
visible wall segment's centroid can move in `odom` as the robot changes view.
Even with no moving pedestrian, 65.7% of baseline track *output entries* had
`is_dynamic=true`. Repeated predicted tracks also count in that percentage.
It would be unsound to raise the dynamic speed threshold just to suppress
this symptom without checking true moving-target recall.

The scan-only bags cannot calibrate the camera tracker or fusion parameters.
The short 2026-09-27 RGB-D bags have 34, 70, and 268 scans and a person, but
no labeled LiDAR tracks or independent obstacle trajectories. Before setting
production tracker/fusion thresholds, record representative static and moving
obstacles with measured positions, sizes, and velocities; compare detection
recall, false merges, identity switches, and false dynamic classifications.
