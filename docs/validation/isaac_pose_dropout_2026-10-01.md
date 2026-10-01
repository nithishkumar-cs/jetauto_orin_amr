# Isaac final-turn `/pose` dropout — 2026-10-01

## Finding

The final-turn dropout in the 2026-09-30 distinct-route validation is caused
by SLAM Toolbox 2.6.10's translation-only scan pre-filter. This is not a LiDAR
gap or a stopped `odom -> base_link` TF stream. It is also not evidence that
scan matching failed on the turn: those scans never reached the matcher.

The installed package is `ros-humble-slam-toolbox 2.6.10`. In that release,
[`SlamToolbox::shouldProcessScan`](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp)
rejects a scan while `last_pose.SquaredDistance(pose) < 0.8 *
minimum_travel_distance²`. Its test does not use `minimum_travel_heading`.
The installed Karto `Pose2::SquaredDistance` delegates to its XY position's
squared distance, excluding yaw (`/opt/ros/humble/include/karto_sdk/Karto.h`).
The configured `minimum_travel_distance` is 0.05 m, so the effective XY gate
is `sqrt(0.8) * 0.05 = 0.04472 m`. Pose publication happens only after a scan
passes this filter and is processed. [Upstream issue #807](https://github.com/SteveMacenski/slam_toolbox/issues/807)
also documents the in-place-rotation behavior; the installed 2.6.10 source is
the basis for this diagnosis.

## Recorded evidence

Read-only inspection of
`/home/nithish/jetauto_live_validation_20260930/localization_bag_retry`:

- Last scan-matched `/pose` stamp: 79.350143 s simulated time.
- From then to route completion at about 84.567 s: 52 scans, continuous
  60 Hz odometry and `odom -> base_link` TF, and about 2.541 rad of yaw change.
- Maximum XY displacement from odometry at the last pose: 0.015414 m—less
  than one-third of the 0.04472 m pre-filter threshold. From 80.0 s onward,
  recorded XY odometry is exactly constant while yaw continues changing.
- Scans remained populated (681–701 valid returns per 720-beam scan in the
  first 10 s after the last pose). Debug logs show the TF message filter
  marking the turn's scans ready in `odom`.
- `map -> odom` TF continued to be published. Its value made one final update
  around 79.85 s, then remained constant through the turn while timestamps
  advanced. That continued TF is not evidence of fresh scan correction.

## Isolated control replay

On ROS domain 83, replayed the same recorded `/clock`, `/scan`, and source TF
with previously recorded `map -> odom` removed. Ran the saved-map localizer
with the same configuration except `minimum_travel_distance:=0.001`; recorded
only its `/pose` output to
`/tmp/jetauto_pose_dropout_control_20261001`. The parameter query confirmed
the 0.001 m override. It produced 320 poses in the replay, including four
between 79.35 and 79.85 s. The last pose was at 79.85 s; there were no later
poses during the remaining in-place rotation. Recorded XY displacement after
80.0 s was zero, so even the 1 mm gate could not pass another scan. This
independent replay is consistent with the translation-only filter, not a
network or recording gap. All replay/localizer/recorder processes were stopped.

## Implication and next validation

Do not treat a current `map -> odom` TF timestamp alone as proof of recent
scan matching. Before enabling Nav2 motion, monitor age of the last matched
`/pose`, and explicitly validate localization during an in-place turn with a
heading-aware upstream fix or a tested replacement. Setting the distance
threshold to zero would bypass the pre-filter but could send every 10 Hz scan
to scan matching; that CPU and accuracy trade-off has **not** been validated
here. The subsequent [heading-aware patch and replay](isaac_heading_fix_2026-10-01.md)
resolved the turn dropout without changing the configured distance threshold.
