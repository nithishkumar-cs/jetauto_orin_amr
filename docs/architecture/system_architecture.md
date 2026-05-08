# Architecture

`jetauto_orin_amr` is organized as a layered ROS 2 workspace:

1. Platform integration: `base_interface`, `sensor_drivers`, `tf_and_calibration`, `robot_bringup`.
2. High-performance perception: `cuda_common`, `perception_preproc`, `perception_inference`.
3. Robotics interpretation: `perception_geometry`, `tracking_fusion`, `sensor_fusion`.
4. Safety and operations: `safety_layer`, `navigation_tasks`, `logging_and_diagnostics`.
5. Measurement: `benchmarks`, `evaluation_tools`.

The webcam development profile is intentionally 2D-only. It proves image capture, CUDA preprocessing, detector wiring, health, and benchmark reporting with the hardware available today. The depth profile enables 2D detection plus depth lifting, tracking, fusion, and safety once the JetAuto depth camera is connected.

## Data Flow

```text
USB/depth camera
  -> sensor_drivers
  -> perception_preproc CUDA path
  -> perception_inference detector backend
  -> perception_geometry depth lifting
  -> tracking_fusion
  -> sensor_fusion
  -> safety_layer
  -> base_interface
```

The detector node accepts backend names (`debug`, `yolo_tensorrt`, `rf_detr_tensorrt`) without changing downstream topics. That makes RF-DETR an implementation swap plus benchmark comparison, not a graph redesign.

## Hot-Path Policy

- Keep runtime-critical code in C++.
- Keep dense image preprocessing in CUDA.
- Reuse device buffers and streams.
- Publish diagnostics from every critical node.
- Benchmark each stage before optimizing it.

