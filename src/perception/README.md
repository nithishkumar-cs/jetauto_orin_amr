# Perception packages

Each folder below will hold one source-agnostic ROS package. The folder gives
the architecture a readable shape; the package names remain globally unique.

- `detector/` → `perception_detector`: rectified RGB image through a pluggable
  C++ detector backend to `vision_msgs/Detection2DArray`. **Built and validated
  on the Orin with fused CUDA preprocessing and a YOLO26n FP16 TensorRT engine.**
- `detection_depth_projection/` → `perception_detection_depth_projection`: 2D image detections plus
  aligned RGB-D camera depth to `vision_msgs/Detection3DArray`. **Built.**
- `lidar_clustering/` → `perception_lidar_clustering`: `/scan` to unclassified
  geometric `vision_msgs/Detection3DArray` observations in the scan frame.
  **Built.** It is separate from collision monitoring.
- `tracking/` → `perception_tracking`: separate camera and LiDAR observation
  streams to persistent, velocity-estimating `TrackedObstacleArray` tracks in
  `base_link`. **Built.**
- `fusion/` → `perception_fusion`: combines camera and lidar tracks into the
  project's authoritative `/perception/fused_obstacles`
  `TrackedObstacleArray` contract. **Built.**

`preproc/` is intentionally absent. Add it only if the chosen detector backend
needs a shared preprocessing step; do not create a node that no consumer uses.
