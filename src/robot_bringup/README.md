# robot_bringup

Single entrypoint package for the robot runtime. It composes platform,
perception, diagnostics, safety, localization, and navigation nodes using one
launch file plus mode arguments.

Primary entrypoint:

- `ros2 launch robot_bringup robot_stack.launch.py`

Primary launch axes:

- `instrumentation_mode`: `debug`, `profile`, or `production`
- `detector_backend`: `debug`, `yolo`, or `rf_detr`
- component toggles: `enable_base_interface`, `enable_sensor_drivers`,
  `enable_preproc`, `enable_depth_pipeline`, `enable_teleop`,
  `enable_localization`, `enable_navigation_tasks`, `enable_benchmarks`

`system_modes.yaml` lives at the package root and defines two component sets:
one shared by `debug` and `profile`, and one for `production`. Launch arguments
can override component toggles when needed. Rosbag playback is handled outside
this package.
