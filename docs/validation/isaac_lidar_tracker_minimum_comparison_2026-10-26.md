# LiDAR tracker after two- versus three-point clustering (2026-10-26)

## Method

Replay the 2026-09-27 Isaac validation bag through the installed LiDAR
clustering and tracking nodes at 1×, once with the checked-in 0.20 m gap and
two-point minimum and once with only `min_cluster_points:=3` overridden. The
source has 70 scans, a crossing person, three modeled static objects, recorded
TF, and no exported Isaac ground truth. Detector inference was skipped; the
bag's recorded `/perception/detections_3d` fed the camera tracker, which is not
part of this comparison. The replay tool now records `/tf` with its outputs so
the standard tracker scorer can use each bag directly.

The four obstacle centers were reconstructed from the 2026-09-30 Jetson Isaac
scene snapshot, including its eight-second person motion law, at 20 Hz from
65 to 76 s. `reconstruct_isaac_default_scene_gt.py` generates the CSV. This
model omits walls, other geometry, occlusion, and any change between the
snapshot and the earlier recording. It is a comparison label set, not a
measured false-positive reference. The scorer uses a 0.75 m center gate and
recorded `odom -> base_link` TF.

The two-point replay yielded 70 cluster arrays and 69 track arrays; the
three-point replay yielded 69 and 68. For the comparison below, score only the
68 track-array scan stamps present in **both** runs, using TF from the source
bag. This removes the differing output count, although DDS scheduling and
tracker startup can still vary between separate replays. Both runs had one
initial common frame with no confirmed modeled object.

## Result on common frames

| Measure | Two points | Three points |
| --- | ---: | ---: |
| Track observations on 68 shared frames | 1,837 | 1,678 |
| Observations unmatched to four modeled centers | 1,569 | 1,410 |
| Modeled object matches / expected | 268/272 | 268/272 |
| Person matches / expected | 67/68 | 67/68 |
| Person ID switches | 1 | 0 |
| All four modeled objects' ID switches | 5 | 3 |
| Fragmentations | 0 | 0 |

The two-point minimum produced **159 more confirmed track observations** on
the same output stamps, 2.34 per frame. All 159 are unmatched to the four
modeled centers; the matched count is identical. Tracker confirmation therefore
did **not** absorb all the extra clustering output in this recording. It did
not improve or reduce person detection recall on the shared frames. The one
additional person ID switch and two additional switches overall are observed
in this pair of replays, not a calibrated identity-error difference.

The earlier 1,070-scan *static* mapping replay gives a consistent output-volume
check: with the same tracker settings, the two-point run published 28.2 versus
25.5 tracks per message and 1,369 versus 1,329 distinct output IDs. Those
figures are whole-run diagnostics from
[the prior sweep](isaac_lidar_threshold_sweep_2026-10-14.md), not object
accuracy scores.

The 1,569 and 1,410 unmatched observations are **not false-positive counts**:
walls and other real returns are absent from the four-object label set. The
197 extra clusters without a static-scene model match came from a different
bag. Neither replay records which input cluster created each track, so this
comparison cannot count how many of those particular 197 survived tracker
confirmation. It establishes the downstream cost in output clutter, while
true false positives and moving-target identity accuracy still require paired,
exported Isaac labels and representative runs. The two-point default stays in
place because its static-scene narrow-feature gain is measured and this short
replay does not support a robust replacement threshold.

## Reproduce

Run from the repository root after sourcing `/opt/ros/humble/setup.bash` and
`install/setup.bash`. Use an isolated `ROS_DOMAIN_ID` and a writable
`ROS_LOG_DIR`. Output bags and scores belong under ignored `.agent-work/`.

```bash
python3 tests/integration_tests/reconstruct_isaac_default_scene_gt.py \
  --start-s 65 --stop-s 76 > .agent-work/lidar_tracker_reconstructed_gt.csv
python3 tests/integration_tests/replay_perception_chain.py \
  ~/jetauto_isaac_validation_20260927_final .agent-work/lidar_tracker_mp2 \
  --no-detector --rate 1
python3 tests/integration_tests/replay_perception_chain.py \
  ~/jetauto_isaac_validation_20260927_final .agent-work/lidar_tracker_mp3 \
  --no-detector --rate 1 \
  --set perception_lidar_clustering.min_cluster_points=3
python3 rosbag/score_trackers.py .agent-work/lidar_tracker_mp2 \
  --topics /perception/lidar/tracks \
  --ground-truth-csv .agent-work/lidar_tracker_reconstructed_gt.csv
python3 rosbag/score_trackers.py .agent-work/lidar_tracker_mp3 \
  --topics /perception/lidar/tracks \
  --ground-truth-csv .agent-work/lidar_tracker_reconstructed_gt.csv
```

The common-stamp comparison used source-bag TF for both runs; the individual
scorer commands above use the TF recorded in each replay bag. Raw bags, the
generated CSV, and score JSON remain outside Git.
