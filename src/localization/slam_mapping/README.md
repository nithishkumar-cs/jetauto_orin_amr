# localization_slam_mapping

This package launches upstream SLAM Toolbox in online asynchronous mapping
mode. It consumes `/scan` and the time-aligned TF chain
`odom -> base_link -> lidar_link`, then publishes `/map` and owns
`map -> odom`. It does not own `odom -> base_link` or drive the robot.

Start it independently with:

```bash
ros2 launch localization_slam_mapping slam_mapping.launch.py backend:=isaac
```

Or use `robot_bringup` with `slam_mode:=mapping`. The default is `off`, including
in production, so a normal start does not build a fresh map. `backend:=isaac`
and `backend:=rosbag` use `/clock`; `backend:=hardware` uses wall time. Bag
playback is a separate process and must include `/scan`, `/tf`, `/tf_static`,
and `/clock` with compatible stamps. If the sensor frame or TF is missing,
SLAM cannot incorporate scans.

The range and motion thresholds in `configs/localization/slam_mapping.yaml`
are starting values for the current Isaac LiDAR, not calibrated hardware
settings. Verify map geometry and loop closures over a longer route before
using a saved map for navigation. Save both the occupancy image and the
serialized pose graph after mapping; the image alone cannot resume SLAM.

After mapping, save both products with a writable path prefix:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/path/to/maps/jetauto'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/path/to/maps/jetauto'}"
```

The first command writes `.pgm` and `.yaml`; the second writes `.posegraph`
and `.data`. Check both service results before treating a map as saved.
