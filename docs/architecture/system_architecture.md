# Architecture

`jetauto_orin_amr` is organized as a layered ROS 2 workspace:

1. System orchestration: `robot_bringup` under `src/orchestration/robot_bringup`.
2. Platform integration: `sensor_drivers`, `tf_and_calibration`, plus hardware drivers and simulator bridges as needed.
3. High-performance perception: `perception_detector`.
4. Robotics interpretation: `perception_detection_depth_projection`,
   `perception_lidar_clustering`, `perception_tracking`, and `perception_fusion`.
5. Localization: `localization_odometry_fusion` configures the upstream
   `robot_localization` EKF; optional `localization_slam_mapping` configures
   upstream SLAM Toolbox for online 2D mapping.
6. Safety and operations: `nav2_collision_monitor`, `estop_gate`, navigation,
   and diagnostics.
7. Measurement: `benchmarks`, `evaluation_tools`.

The debug mode is intentionally 2D-first. It proves image capture, CUDA preprocessing, detector wiring, and health reporting with the robot hardware path. The profile and production modes expand the launched subsystem set through `system_modes.yaml`.

## Data Flow

```text
RGB/depth camera
  -> perception_detector
  -> perception_detection_depth_projection -> camera tracker --+
                                                              +-> perception_fusion
2D LiDAR -> /scan -> perception_lidar_clustering -> lidar tracker+

/scan -> nav2_collision_monitor -> estop_gate -> base driver or simulator

/odom/wheel (+ /imu/data on hardware) -> robot_localization EKF
                                    -> /odometry/filtered

/scan + odom -> base_link -> lidar_link TF -> SLAM Toolbox
                                         -> /map + map -> odom TF
```

The detector node selects implementations through a backend factory without
changing downstream topics. `yolo_tensorrt` is currently registered; adding
RF-DETR means implementing the shared contract and registering its factory entry,
not redesigning the ROS graph.

The two tracking processes use the same `perception_tracking` executable with
different topic and source parameters. Both track in `odom`, so robot motion is
removed from obstacle velocity estimates, then publish in `base_link` for
fusion and robot-local consumers. The required time-aligned TF chain is supplied
by the odometry source and sensor extrinsic transforms;
the tracker does not consume raw IMU messages directly.

On hardware, the EKF combines wheel velocity with IMU yaw rate and owns
`odom -> base_link`; the base driver must not broadcast that transform. In the
current Isaac scene, there is no IMU, so the EKF uses Isaac wheel odometry
alone and Isaac owns the transform. These are local odometry profiles, not
SLAM or global localization. The zero covariance in the current Isaac odometry
is not a calibrated uncertainty estimate.

Online mapping is selected explicitly with `slam_mode:=mapping` in bringup.
SLAM Toolbox owns `map -> odom`; the EKF or Isaac owns `odom -> base_link`,
and the static sensor transform connects `base_link` to the LiDAR. No second
publisher should claim either moving TF edge. Mapping is not navigation and
does not send velocity commands.

`perception_fusion` approximately synchronizes the two track streams within a
strict time bound, associates tracks one-to-one in `base_link`, and combines
matched planar position and velocity using their covariance. Unmatched evidence
is preserved. Its identity registry produces globally unique fused IDs that
remain stable as a source appears or disappears. The output contract is
`/perception/fused_obstacles` (`amr_interfaces/TrackedObstacleArray`).

## Hot-Path Policy

- Keep runtime-critical code in C++.
- Keep dense image preprocessing in CUDA.
- Reuse device buffers and streams.
- Publish diagnostics from every critical node.
- Benchmark each stage before optimizing it.
