# A single tracked obstacle in 3D. Produced by tracking and fusion.
#
# Frame and stamp are carried by the enclosing TrackedObstacleArray.header;
# every obstacle in one array shares that frame (expected: base_link).

uint32 track_id                              # persistent id across frames
vision_msgs/ObjectHypothesis classification  # class_id + score (standard type)
geometry_msgs/PoseWithCovariance pose        # position + orientation + 6x6 covariance
geometry_msgs/Vector3 size                   # box extents (l, w, h) in the box frame
geometry_msgs/TwistWithCovariance velocity   # velocity in the array frame + covariance
bool is_dynamic                              # moving vs static
string source                                # producing sensor / pipeline id
