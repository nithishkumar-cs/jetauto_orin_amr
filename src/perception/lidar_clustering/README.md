# perception_lidar_clustering

Clusters adjacent valid returns from a standard `sensor_msgs/LaserScan` and
publishes unclassified planar obstacle boxes as a standard
`vision_msgs/Detection3DArray`.

```text
/scan                                      sensor_msgs/LaserScan
  -> perception_lidar_clustering
/perception/lidar/clusters                 vision_msgs/Detection3DArray
```

The core is source-agnostic and performs no ROS allocation or copying of the
range array. Each valid range is converted from polar coordinates into the scan
frame, and consecutive points remain in one cluster while their Euclidean
distance is no greater than `max_adjacent_point_distance_m`. Invalid returns and
larger gaps separate clusters. Candidates smaller than `min_cluster_points` are
dropped. The first and last candidates are joined when a full-circle scan wraps
through its angle boundary and their endpoint gap passes the same test.

Each result contains an axis-aligned XY bounding box in the input scan frame.
`Detection3D.results` is empty because a LiDAR geometry cluster has no class
hypothesis. The center Z and box height are zero because a planar LiDAR cannot
measure either one. Tracking and fusion are responsible for transforms,
persistent IDs, velocities, and camera classification.

Scans with an empty frame ID or inconsistent geometry metadata are rejected,
because publishing coordinates without a valid reference frame would make the
result unusable downstream.

Parameters are defined in `configs/perception/lidar_clustering.yaml`.
The default gap and minimum-point thresholds are starting values; they must be
tuned using the selected LiDAR resolution and representative scenes.

## Test tiers

- `test_laser_scan_clusterer_unit`: ROS-independent scan validation, coordinate
  conversion, segmentation, filtering, bounding boxes, and 360-degree wrapping.
- `test_lidar_clustering_node_component`: the production node with real ROS
  topics, QoS, callback execution, parameters, message conversion, empty output,
  and malformed-input rejection.
