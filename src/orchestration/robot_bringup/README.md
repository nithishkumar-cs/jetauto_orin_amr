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
