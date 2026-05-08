# Calibration And TF

The expected frame chain is:

```text
map -> odom -> base_link
base_link -> rgb_camera_link -> rgb_camera_optical_frame
base_link -> depth_camera_link -> depth_camera_*_optical_frame
base_link -> lidar_link
```

`tf_and_calibration` provides:

- static transform launch placeholders
- frame naming conventions
- camera info validation
- TF availability validation

The placeholder transforms in `frames.yaml` must be replaced with measured extrinsics once the camera and LiDAR are mounted. Do not tune safety distances or depth projection quality around guessed extrinsics.

Minimum calibration checklist:

- USB camera intrinsics recorded or approximated with clear notes.
- Depth camera intrinsics come from the vendor driver.
- Camera-to-base transform measured after final mounting.
- LiDAR-to-base transform measured after final mounting.
- TF tree captured with `ros2 run tf2_tools view_frames`.
