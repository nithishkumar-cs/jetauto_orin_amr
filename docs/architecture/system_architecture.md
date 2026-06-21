# Architecture

`jetauto_orin_amr` is organized as a layered ROS 2 workspace:

1. System orchestration: `robot_bringup` under `src/robot_bringup`.
2. Platform integration: `sensor_drivers`, `tf_and_calibration`, plus hardware drivers and simulator bridges as needed.
3. High-performance perception: `cuda_common`, `perception_preproc`, `perception_inference`.
4. Robotics interpretation: `perception_geometry`, `tracking_fusion`, `sensor_fusion`.
5. Safety and operations: `safety_layer`, `navigation_tasks`, `logging_and_diagnostics`.
6. Measurement: `benchmarks`, `evaluation_tools`.

The debug mode is intentionally 2D-first. It proves image capture, CUDA preprocessing, detector wiring, and health reporting with the robot hardware path. The profile and production modes expand the launched subsystem set through `system_modes.yaml`.

## Data Flow

```text
RGB/depth camera
  -> sensor_drivers
  -> perception_preproc CUDA path
  -> perception_inference detector backend
  -> perception_geometry depth lifting
  -> tracking_fusion
  -> sensor_fusion
  -> safety_layer
  -> base driver or simulator bridge
```

The detector node accepts backend names (`debug`, `yolo_tensorrt`, `rf_detr_tensorrt`) without changing downstream topics. That makes RF-DETR an implementation swap plus benchmark comparison, not a graph redesign.

## Hot-Path Policy

- Keep runtime-critical code in C++.
- Keep dense image preprocessing in CUDA.
- Reuse device buffers and streams.
- Publish diagnostics from every critical node.
- Benchmark each stage before optimizing it.
