# Profile-metrics replay baseline (2026-10-21)

Replay of `jetauto_isaac_validation_20260927_final` through the full perception chain
(`tests/integration_tests/replay_perception_chain.py`, Release build, 1x, Jetson Orin) with
`export_metrics:=true` on every node and `enable_cuda_timing:=true` on the detector;
`/diagnostics` recorded and summarized per `<node>/metrics` status.

| Node | cycles | process/detect wall ms (mean / p95 / max) | dropped |
|---|---|---|---|
| perception_detector | 55 | 15.3 / 13.8 / 112.8 (CUDA inference 13.9 / 12.1 / 111.0) | 0 |
| detection_depth_projection | 50 | 0.30 / 0.34 / 0.77 | 0 |
| lidar_clustering | 69 | 0.23 / 0.28 / 0.32 | 0 |
| perception_lidar_tracker | 69 | 0.31 / 0.40 / 0.48 | 0 |
| perception_camera_tracker | 50 | 1.28 / 0.31 / 51.0 | 0 |
| perception_fusion | 50 | 0.27 / 0.38 / 0.87 | 0 |

- Every status published on schedule with all documented keys; no rejected or dropped inputs.
- The detector max (112.8 ms) is the first-inference warm-up outlier; steady state is about 14 ms.
- `input_age_ms` is quantized (16.7 ms steps; LiDAR p95 600 ms, fusion max 800 ms) because it
  is measured against replayed sim `/clock`; treat it as indicative only, not as latency.
- Fusion `lidar_tracks` (mean 25) exceeds clusters (mean 17.6): tracks persist across dropouts.
- Full `colcon build` + `colcon test` passes in Release and Debug (192 tests, 0 failures each).
