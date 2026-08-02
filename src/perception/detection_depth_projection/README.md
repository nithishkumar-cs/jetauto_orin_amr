# Detection-depth projection

`detection_depth_projection_node` converts class-labelled 2D bounding boxes into 3D
detections using an aligned depth image and camera calibration.

```text
/perception/detections_2d  (vision_msgs/Detection2DArray)
/camera/aligned_depth_to_rgb/image_raw  (sensor_msgs/Image: 16UC1 or 32FC1)
/camera/rgb/camera_info                 (sensor_msgs/CameraInfo)
                         ↓
/perception/detections_3d  (vision_msgs/Detection3DArray)
```

The three inputs are approximately synchronized and must describe one RGB pixel
grid. The maximum timestamp span is configured with `sync_max_interval_ms` and
defaults to 50 ms. The detector must convert any resized or letterboxed
inference output back to original RGB-image coordinates. The depth source must
register depth into that same RGB grid before publishing it. The node checks the
shared frame and image size, then uses the RGB CameraInfo intrinsics for
projection. For each box, it takes the median valid depth from the central ROI.
Invalid or mismatched data is omitted rather than guessed.

`BoundingBox3D.size.x` and `.size.y` are the 2D box's projected width and
height. `.size.z` is `0.0`: a single depth image cannot reliably estimate an
object's physical thickness.
