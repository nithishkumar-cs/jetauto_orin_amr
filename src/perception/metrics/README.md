# perception_metrics

Header-only `perception_metrics::MetricsReporter`. A node constructs it with
`metrics_(*this)`; it declares `export_metrics` (default false). Disabled: no
publisher, no clock reads. Enabled (bringup profile mode): one
`diagnostic_msgs/DiagnosticStatus` per processed cycle on `/diagnostics`, named
`<node name>/metrics`, values as strings.

Every report ends with `input_age_ms` (node clock minus input stamp) and
`dropped_total` (inputs rejected since start).

| Status name | Node-specific keys |
|---|---|
| `perception_detector/metrics` | `detect_wall_ms`, `detection_count`, `cuda_inference_ms` (profile only) |
| `detection_depth_projection/metrics` | `process_wall_ms`, `input_detections`, `projected_detections` |
| `lidar_clustering/metrics` | `process_wall_ms`, `scan_points`, `cluster_count` |
| `perception_camera_tracker/metrics`, `perception_lidar_tracker/metrics` | `process_wall_ms`, `observation_count`, `rejected_count`, `track_count` |
| `perception_fusion/metrics` | `process_wall_ms`, `camera_tracks`, `lidar_tracks`, `fused_count`, `rejected_count`, `sync_skew_ms` |

Reports are emitted only for completed cycles; a stalled node shows up as a
missing status, and `dropped_total` counts rejected inputs.

## Querying

```
ros2 topic echo /diagnostics --field status --filter "m.status[0].name == 'perception_fusion/metrics'"
ros2 topic hz /diagnostics
```

Field extraction from Python:

```python
def metrics(msg):  # DiagnosticArray -> {status name: {key: float}}
    return {s.name: {kv.key: float(kv.value) for kv in s.values} for s in msg.status}
```
