# perception_tracking

Tracks standard `vision_msgs/Detection3DArray` observations over time and
publishes the project-specific `amr_interfaces/TrackedObstacleArray` contract.
Bringup starts the same executable twice:

```text
/perception/detections_3d       -> perception_camera_tracker -> /perception/camera/tracks
/perception/lidar/clusters      -> perception_lidar_tracker  -> /perception/lidar/tracks
vision_msgs/Detection3DArray                                  amr_interfaces/TrackedObstacleArray
```

The streams remain separate here. Camera observations carry detector classes;
LiDAR clusters may be unclassified. The later fusion node decides whether tracks
from different sensors represent the same physical obstacle.

## Tracking model

The ROS-independent core maintains one constant-velocity Kalman filter per
obstacle with state `[x, y, vx, vy]`. Every observation cycle:

1. Existing tracks are predicted to the observation timestamp.
2. Tracks that have exceeded `max_unobserved_duration_s` are removed.
3. Predicted tracks and observations are associated one-to-one by XY distance,
   bounded by `association_distance_m`.
4. Matched tracks receive a Kalman correction; unmatched observations create
   new IDs; unmatched tracks remain predicted through short dropouts.
5. Tracks become publishable after `min_confirmed_hits`, and speed determines
   `is_dynamic`.

The association policy is deterministic greedy nearest-neighbour matching. It
is deliberately understandable and testable for this first stack version; more
crowded scenes may later justify global assignment and richer motion models.

## Frames and ego motion

Tracking runs internally in `odom`, which is locally stable while the robot
moves. Each output is transformed to `base_link` at the observation timestamp.
This prevents stationary obstacles from appearing to move merely because the
robot moved between sensor frames.

The node therefore requires TF transforms from each input sensor frame to
`odom`, and from `odom` to `base_link`, at the message timestamp. Localization
will produce the moving transform from wheel odometry and IMU; camera/LiDAR
mount calibration supplies the static sensor transforms. Raw IMU data does not
belong in the tracker itself. If either transform is unavailable, that
observation array is rejected rather than tracked in an inconsistent frame.

Position and velocity covariance are transformed into `base_link`. Z,
roll/pitch/yaw, angular velocity, and other unestimated state dimensions carry
the configurable `unobserved_state_variance` instead of pretending they are
known precisely.

Runtime parameters for both instances are in
`configs/perception/tracking.yaml`. The initial thresholds and noise values must
be tuned against representative Isaac and hardware recordings.

## Test tiers

- `test_multi_object_tracker_unit`: ROS-independent confirmation, IDs,
  association, velocity, dynamic classification, prediction/dropout, expiry,
  metadata, covariance, and invalid-input behavior.
- `test_tracking_node_component`: the production ROS node with real topics,
  callbacks, TF transforms, custom output messages, camera and unclassified
  LiDAR observations, dropout/expiry, invalid detections, missing TF, and
  parameter validation.
