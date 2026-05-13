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
