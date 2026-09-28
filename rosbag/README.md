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
