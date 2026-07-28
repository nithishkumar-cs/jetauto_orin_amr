# Detection-depth projection

`detection_depth_projection_node` converts class-labelled 2D bounding boxes into 3D
detections using an aligned depth image and camera calibration.

```text
/perception/detections_2d  (vision_msgs/Detection2DArray)
/camera/depth/image_raw    (sensor_msgs/Image: 16UC1 or 32FC1)
/camera/depth/camera_info  (sensor_msgs/CameraInfo)
                         ↓
/perception/detections_3d  (vision_msgs/Detection3DArray)
```

The three inputs are approximately synchronized. Their frame IDs must match;
output positions are therefore in the depth camera's optical frame. For each
box, the node takes the median valid depth from its central ROI and applies the
camera pinhole model. Detections without enough valid in-range depth are omitted
rather than guessed.

`BoundingBox3D.size.x` and `.size.y` are the 2D box's projected width and
height. `.size.z` is `0.0`: a single depth image cannot reliably estimate an
object's physical thickness.
