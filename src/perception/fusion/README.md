# perception_fusion

Combines independently tracked camera and planar-LiDAR obstacles into the
authoritative perception output:

```text
/perception/camera/tracks --+
                            +-> perception_fusion -> /perception/fused_obstacles
/perception/lidar/tracks  --+   TrackedObstacleArray
```

Both inputs and the output use `amr_interfaces/TrackedObstacleArray` in
`base_link`. The node applies bounded approximate-time synchronization and
refuses mismatched frames. A configured 50 ms synchronization bound limits the
error caused by combining robot-relative tracks from slightly different times.
Both tracker processes must continue publishing arrays; an empty array is valid,
but a completely absent stream cannot form synchronized fusion cycles.

## Fusion model

For each synchronized pair of arrays, the ROS-independent core:

1. Associates camera and LiDAR tracks one-to-one using gated XY distance.
2. Fuses matched XY positions and velocities using their full 2x2 covariance
   matrices in information form, so the lower-uncertainty estimate contributes
   more strongly.
3. Keeps the semantic classification and vertical position from the camera,
   uses conservative component-wise maximum box extents, and marks the result
   dynamic if either input does.
4. Passes unmatched camera or LiDAR tracks through instead of deleting evidence
   seen by only one sensor.
5. Assigns a globally unique fused ID. A short-lived identity registry connects
   the camera and LiDAR source IDs, keeping the fused ID stable as either source
   appears or disappears. Spatial reassociation also handles an upstream tracker
   restarting with a new source ID.

Only planar position and linear velocity are statistically fused. Other
dimensions retain the configured `unobserved_state_variance`; the message does
not claim certainty the sensors and trackers did not estimate.

Runtime parameters are in `configs/perception/fusion.yaml`. Association,
identity age, and synchronization values are starting points and need Isaac and
recorded-hardware tuning.

## Test tiers

- `test_multi_sensor_fusion_unit`: ROS-independent covariance fusion,
  one-to-one association, unmatched evidence, global IDs, source dropout,
  identity expiry/reassociation, covariance validity, duplicate IDs, invalid
  data, and timestamp ordering.
- `test_fusion_node_component`: the production node with real ROS topics,
  bounded approximate synchronization, 25 skewed/out-of-order cycles, custom
  message conversion, frame validation, invalid-track filtering, stable IDs,
  unmatched tracks, parameter overrides, and construction validation.
