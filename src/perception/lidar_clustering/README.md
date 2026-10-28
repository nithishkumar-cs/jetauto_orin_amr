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
The default 0.20 m gap was selected against labeled static geometry in the
1,070-scan Isaac mapping scene (see the
[`static-scene report`](../../../docs/validation/isaac_lidar_labeled_thresholds_2026-10-26.md)).
The three-point minimum limits additional confirmed track output in a paired
moving-person replay; two points retained more narrow chair-leg returns in the
static scene, with no improvement in four-object matches in the live replay
(see the [`live comparison`](../../../docs/validation/isaac_live_ground_truth_2026-10-26.md)).
Recheck both settings against representative moving obstacles and the physical
LiDAR's noise and angular resolution.

## Test tiers

- `test_laser_scan_clusterer_unit`: ROS-independent scan validation, coordinate
  conversion, segmentation, filtering, bounding boxes, and 360-degree wrapping.
- `test_lidar_clustering_node_component`: the production node with real ROS
  topics, QoS, callback execution, parameters, message conversion, empty output,
  and malformed-input rejection.
