# All tracked/fused obstacles for one cycle, in one common frame.
# header.frame_id (e.g. base_link) and header.stamp apply to every obstacle below.

std_msgs/Header header
TrackedObstacle[] obstacles
