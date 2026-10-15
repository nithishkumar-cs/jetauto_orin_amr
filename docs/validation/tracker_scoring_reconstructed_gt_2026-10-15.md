# Tracker scoring: first real-bag run with reconstructed ground truth (2026-10-15)

## Purpose

Exercise `rosbag/score_trackers.py` end to end on recorded Isaac tracker
output before exported Isaac ground truth is available. The Ethernet link is
down, so no new bag could be recorded.

## Data

- Bag: `~/jetauto_isaac_validation_20260927_final` (live camera, LiDAR, and
  fused tracks plus `/tf`; 10.0 s of simulation time, 65.683–75.683 s).
- Ground truth was **reconstructed**, not exported: the default scene in the
  Jetson snapshot `projects/robotics/jetauto_isaac_sim/isaac/standalone/jetauto_scene.py`
  (2026-09-30) places `box_near` (0.7 m cube) at (2.0, −2.2), `box_far`
  (0.8 × 0.8 m) at (4.2, 1.7), and the chair at (3.5, −0.5); the crossing
  person sits at x = 2.8 with y following an 8 s ±0.6 m/s triangle wave of
  simulation time. These were sampled at 10 Hz into a `stamp,id,x,y` CSV. In
  this window the robot drove +x at 0.1 m/s and back, so the TF transform
  path is exercised.

```bash
python3 rosbag/score_trackers.py ~/jetauto_isaac_validation_20260927_final \
  --ground-truth-csv gt_final.csv
python3 rosbag/score_trackers.py ~/jetauto_isaac_validation_20260927_final \
  --ground-truth-csv gt_final.csv --topics /perception/camera/tracks \
  --ids crossing_person,chair --fov-deg 81
```

Scoring took 1.7 s wall time. Gate 0.75 m; motion truth threshold 0.20 m/s.

## Results

| topic | obstacle | recall | ID sw | frag | IDs | median / p95 error (m) | motion acc. |
|---|---|---|---|---|---|---|---|
| camera (81° FOV) | crossing_person | 0.980 | 1 | 1 | 2 | 0.260 / 0.311 | 0.880 |
| camera (81° FOV) | chair | 0.078 | 0 | 0 | 1 | 0.201 / 0.201 | 1.000 |
| LiDAR | crossing_person | 0.984 | 2 | 1 | 3 | 0.160 / 0.209 | 0.950 |
| LiDAR | chair | 0.951 | 1 | 1 | 2 | 0.320 / 0.356 | 0.931 |
| LiDAR | box_near | 0.984 | 2 | 1 | 3 | 0.334 / 0.424 | 0.317 |
| LiDAR | box_far | 0.984 | 3 | 1 | 4 | 0.321 / 0.544 | 0.283 |
| fused | crossing_person | 1.000 | 1 | 0 | 2 | 0.214 / 0.280 | 0.981 |
| fused | chair | 0.962 | 1 | 1 | 2 | 0.288 / 0.319 | 1.000 |
| fused | box_near | 1.000 | 1 | 0 | 2 | 0.385 / 0.470 | 0.288 |
| fused | box_far | 1.000 | 2 | 0 | 3 | 0.330 / 0.528 | 0.231 |

LiDAR and fused output carried about 21 tracks per frame that matched no
ground-truth obstacle (1,257 of 1,495 and 1,088 of 1,294 observations),
mostly walls and other scene geometry.

## Findings

- **Static boxes are mostly labelled dynamic** by the LiDAR tracker while the
  robot moves (motion accuracy 0.28–0.32), and fusion inherits it. This
  quantifies the viewpoint-dependent face-center shift suspected in the
  [LiDAR sweep](isaac_lidar_threshold_sweep_2026-10-14.md). The person is
  classified correctly in 95–98% of matched frames.
- **Box and chair errors are a measurement-geometry floor**, not tracking
  error: the sensor sees the near face, about half the box depth (0.35–0.40 m)
  from the ground-truth center.
- **Person error is time-offset sensitive.** Shifting ground truth 0.2 s later
  cut the LiDAR median from 0.160 to 0.108 m and fused from 0.214 to 0.150 m;
  camera changed little (0.260 to 0.250 m at +0.1 s). That is either tracker
  lag or an offset between the reconstructed motion law and `/clock`; this bag
  cannot separate them.
- The camera rarely confirms the chair (4 of 51 frames).

## Limits

Reconstructed ground truth from a scene snapshot, one 10 s window, a single
crossing person, and no occlusion model. These numbers validate the scorer's
bag, TF, association, and metric paths; they are not tracker calibration
evidence. Exported Isaac ground truth recorded alongside the tracks with
`rosbag/record_tracker_scoring.sh` is required for that.
