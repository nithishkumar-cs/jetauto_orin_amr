# Perception packages

Each folder below will hold one source-agnostic ROS package. The folder gives
the architecture a readable shape; the package names remain globally unique.

- `detector/` → `perception_detector`: RGB image to `vision_msgs/Detection2DArray`.
- `detection_depth_projection/` → `perception_detection_depth_projection`: 2D image detections plus
  aligned RGB-D camera depth to `vision_msgs/Detection3DArray`. **Built.**
- `lidar_clustering/` → `perception_lidar_clustering`: `/scan` to unclassified
  geometric obstacle observations. It is separate from collision monitoring.
- `tracking/` → `perception_tracking`: detections over time to stable tracks.
- `fusion/` → `perception_fusion`: combines camera and lidar tracks into the
  project's `TrackedObstacleArray` contract.

`preproc/` is intentionally absent. Add it only if the chosen detector backend
needs a shared preprocessing step; do not create a node that no consumer uses.
