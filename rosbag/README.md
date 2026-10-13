# Rosbag Workflows

Rosbag playback is intentionally outside `robot_bringup`.

The robot stack should not know whether standard topics are produced by live
hardware, rosbag playback, or an external simulator. Start playback separately,
then launch `robot_bringup` normally.

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
