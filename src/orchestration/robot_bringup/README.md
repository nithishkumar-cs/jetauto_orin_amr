# robot_bringup

Single entrypoint package for the robot runtime. It composes platform,
perception, diagnostics, safety, localization, and navigation nodes using one
launch file plus mode arguments.

Primary entrypoint:

- `ros2 launch robot_bringup robot_stack.launch.py`

Primary launch axes:

- `instrumentation_mode`: `debug`, `profile`, or `production`
- `backend`: `hardware`, `isaac`, or `rosbag`; selects exactly one source edge
- component toggles: `enable_drivers`, `enable_preproc`, `enable_detector`,
  `enable_geometry`, `enable_tracking`, `enable_fusion`, and the remaining
  entries in `configs/runtime_modes/system_modes.yaml`

`system_modes.yaml` lives at the package root and defines two component sets:
one shared by `debug` and `profile`, and one for `production`. Launch arguments
can override component toggles when needed. The default `backend:=hardware`
will start `base_driver` once that package exists. `backend:=isaac` launches no
adapter: configure Isaac's ROS 2 graph to publish and consume the canonical AMR
topics directly. `backend:=rosbag` is reserved for a replay launch, which will
be added after the bag manifest and simulation-time policy are defined.

When `localization` is enabled, bringup includes
`localization_odometry_fusion`, which starts the upstream `robot_localization`
EKF. Install `ros-humble-robot-localization` first (the Jetson dependency setup
script includes it); otherwise bringup reports localization as pending. The
Isaac profile consumes `/odom/wheel`, publishes `/odometry/filtered`, uses
simulation time, and leaves `odom -> base_link` TF ownership with Isaac. The
hardware profile consumes wheel odometry and `/imu/data`, publishes the same
filtered topic, and owns that TF. Hardware drivers must not broadcast a second
`odom -> base_link`. A rosbag playback uses the Isaac profile, but bringup does
not yet start the bag player.

Online 2D mapping is a separate, opt-in launch axis:

```bash
ros2 launch robot_bringup robot_stack.launch.py backend:=isaac slam_mode:=mapping
```

`slam_mode:=off` is the default even in production. Mapping starts
`localization_slam_mapping` (upstream SLAM Toolbox) alongside any enabled
odometry component. It consumes `/scan` plus TF from `odom` through
`base_link` to the scan frame, publishes `/map`, and owns `map -> odom`.
It does not command motion. On hardware, keep `enable_localization:=true` so
the EKF supplies `odom -> base_link`; Isaac and rosbag require that TF from
their source instead. Map saving is documented in the mapping package README.
