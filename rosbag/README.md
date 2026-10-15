# Rosbag Workflows

Rosbag playback is intentionally outside `robot_bringup`.

The robot stack should not know whether standard topics are produced by live
hardware, rosbag playback, or an external simulator. Start playback separately,
then launch `robot_bringup` normally.

For the isolated software-safety replay, run
`python3 tests/integration_tests/check_isaac_safety_replay.py` from the repo
root after sourcing ROS and the workspace. It reads the recorded Isaac bag
without modifying it, replays `/clock`, `/scan`, and TF, and deliberately drops
scans and then moving TF. It starts only the safety chain and a synthetic
command publisher; no Nav2 controller or driver is launched. See the
[validation report](../docs/validation/offline_isaac_safety_replay_2026-10-13.md).

Example:

```bash
ros2 bag play /path/to/bag --clock
ros2 launch robot_bringup robot_stack.launch.py instrumentation_mode:=profile
```

## Perception bag analysis

Source ROS and the workspace so custom message types can be deserialized, then
run the analyzer against a bag directory:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 rosbag/analyze_perception_bag.py /path/to/bag
```

The report separates record-time rate from header-stamp rate, finds sensor
timestamp gaps, checks exact RGB/depth and perception-stage stamp coverage,
prints image formats, and summarizes semantic track IDs. This makes a slow
simulator distinguishable from missing source frames and exposes identity
restarts without relying on visual inspection.

## Tracker scoring against Isaac ground truth

Run Isaac with `-GroundTruth` (optionally `-Scenario` and `-WalkingPerson`) and
the perception stack, then record on the Jetson from a shell with the same
`ROS_DOMAIN_ID` and Fast DDS profile as bringup. The script refuses to start
while any scoring topic lacks a publisher and never overwrites a bag:

```bash
rosbag/record_tracker_scoring.sh ~/jetauto_scoring_head_on_$(date +%Y%m%d_%H%M)
```

It records `/clock`, TF, `/scan`, `/ground_truth/obstacles`, the tracker
inputs (`/perception/detections_3d`, `/perception/lidar/clusters`), and the
camera, LiDAR, and fused tracks. Extra arguments go to `ros2 bag record`.

`score_trackers.py` moves each track array into `odom` with the bag's
`odom -> base_link` TF at the array stamp and matches it to ground truth
interpolated to that stamp (CLEAR-MOT: last frame's pairs persist inside the
gate, the rest use optimal assignment). Ground truth comes from the bag topic,
or from Isaac's `ground_truth.csv` with `--ground-truth-csv`; its times must be
Isaac simulation seconds.

```bash
python3 rosbag/score_trackers.py BAG [--ground-truth-csv ground_truth.csv] [--json scores.json]
python3 rosbag/score_trackers.py BAG --topics /perception/camera/tracks \
  --ids crossing_person,chair --fov-deg 81 --max-range-m 6
```

(81° is the Isaac camera's horizontal field: 4.8 mm aperture, 2.8 mm focal length.)

Per topic and per obstacle it reports recall over *expected* frames, ID
switches, fragmentations (re-acquired after an expected miss), distinct track
IDs, center position error, and a moving/static confusion matrix. Motion truth
is ground-truth speed at or above `--moving-speed-mps` (default 0.20, the
tracker threshold), not the CSV label. `--ids`, `--fov-deg`, and
`--max-range-m` make other obstacles "don't care": they absorb tracks but are
never misses; occlusion is not modelled. Tracks matching no ground truth
(walls and other scene geometry) are reported as unmatched, not as an error
rate. Position error is to the obstacle center, so a sensor seeing only the
near face of a 0.7 m box reads about 0.35 m even when tracking is exact.
Exit status is 1 if a requested topic is absent or no frame overlapped
ground truth and TF.

The ground-truth field names are **assumed** (`time=stamp`, `list=obstacles`,
`id`, `x`, `y`); the authoritative schema is the `ground_truth` entry of the
Windows Isaac repo's `config/topic_contract.yaml`. A mismatch fails with the
names actually present; map them with e.g.
`--gt-fields time=sim_time_s,id=name,x=position.x,y=position.y` (dotted paths
address nested JSON objects and list indices, e.g. `x=position.0`;
`{sec, nanosec}` stamps are accepted). Ground truth that gives one obstacle two
positions at the same time is rejected: it mixes runs, e.g. a CSV appended
across Isaac launches.
See the [first real-bag run](../docs/validation/tracker_scoring_reconstructed_gt_2026-10-15.md).

## Saved-map localization stress check

`inject_odometry_tf_drift.py` makes a **new** replay bag from an Isaac
localization bag. It gradually biases only `odom -> base_link` TF, removes
previously generated `map -> odom` TF and `/pose`, and preserves the original
odometry as `/validation/reference_odom`. It never overwrites its output path.
Play this derived bag through a fresh saved-map SLAM Toolbox localization node
with `use_sim_time:=true`, recording the new `/pose` output separately.

`analyze_saved_map_localization.py` compares those poses to the untouched
Isaac odometry using the mapping run's **fixed** final `map -> odom` alignment.
Pass the original localization bag as the reference and the derived bag with
`--perturbed-bag` to report both localization error and raw input drift.
See the [drift validation report](../docs/validation/isaac_odometry_drift_2026-09-30.md)
for the tested amplitudes, commands, results, and limits.

## SLAM loop-match relative-pose audit

`audit_karto_loop_matches.py` reads console output from the optional Karto
loop-closure probe and compares each fine-accepted match with Isaac's exact
`/odom/wheel` poses. It transforms odometry to the LiDAR frame using the
configured rigid offset (default: 0.15 m forward). A large distance between
scan centers is **not** by itself evidence of a false match: the graph edge
stores their relative pose. The report distinguishes a fine-accepted candidate
from one that passed Karto's link-distance gate. With the current probe, it
also correlates Karto's `AddEdge` creation marker with the matching scan pair;
a gate pass alone does not prove that a new graph edge was inserted. Older
probe logs without edge markers report edge creation as unknown.

```bash
source /opt/ros/humble/setup.bash
python3 rosbag/audit_karto_loop_matches.py \
  /path/to/isaac_raw_bag /path/to/mapper_console.log --all
```

Use `--sensor-x-m`, `--sensor-y-m`, and `--sensor-yaw-rad` if the LiDAR
mount differs from the Isaac scene. Omit `--all` to print only gate-passing
matches. The probe is diagnostic only; remove it and rebuild the normal
SLAM Toolbox overlay after collecting a trace.
See the [Isaac relative-pose audit](../docs/validation/isaac_loop_match_relative_audit_2026-10-11.md)
for the two-bag comparison and its limitations, and the
[full-route follow-up](../docs/validation/isaac_full_route_1m_map_and_localization_2026-10-12.md)
for actual graph-edge insertion, saved-map export, and diagonal-route
localization.
The [distinct-geometry comparison](../docs/validation/isaac_distinct_geometry_slam_2026-10-12.md)
replays a new pillar-cluster scene at both 1.0 m and 3.0 m and localizes a
separate interior route against each saved map.

## Planner-only path probe

With a planner-only stack already running, `probe_planner_only.py` asks Nav2
to compute a path and optionally checks every returned path pose against a
fresh global-costmap snapshot. `--start x y` tests an explicit map-frame
start; `--live-start` instead sets `use_start=false` so Nav2 obtains the
robot's current pose from TF. These modes are mutually exclusive:

```bash
python3 rosbag/probe_planner_only.py --live-start --goal 2.27 1.20 \
  --expect success --inspect-costmap
```

Neither path computation nor this probe publishes `/cmd_vel`. A path can
succeed even when most live scans fail the costmap TF filter; consult the
[1× live-start validation](../docs/validation/isaac_live_start_planner_tf_2026-10-13.md)
before interpreting a plan as obstacle-awareness evidence.

## Scan-to-costmap obstacle check

`inject_costmap_obstacle.py` makes an isolated `/scan_costmap` copy of an
Isaac bag's scans, adding a short-lived return at a chosen map point without
changing `/scan` or the saved-map localizer input. With a planner configured
to consume `/scan_costmap`, `analyze_costmap_scan_filter.py` measures TF-filter
admission and estimated scan age, while `analyze_costmap_obstacle.py`
checks whether costmap snapshots mark and then clear the target. See the
[replay and findings](../docs/validation/isaac_costmap_obstacle_2026-10-13.md),
including the per-source height configuration issue and the limits of the
temporary 0.8 s TF wait.
