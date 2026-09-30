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
