#ifndef PERCEPTION_DETECTION_DEPTH_PROJECTION__DETECTION_DEPTH_PROJECTION_NODE_HPP_
#define PERCEPTION_DETECTION_DEPTH_PROJECTION__DETECTION_DEPTH_PROJECTION_NODE_HPP_

#include <memory>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "perception_detection_depth_projection/depth_projector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace perception_detection_depth_projection
{

class DetectionDepthProjectionNode : public rclcpp::Node
{
public:
  explicit DetectionDepthProjectionNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  void on_synchronized_input(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);

  DepthProjector projector_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detections_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_subscription_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
};

}  // namespace perception_detection_depth_projection

#endif  // PERCEPTION_DETECTION_DEPTH_PROJECTION__DETECTION_DEPTH_PROJECTION_NODE_HPP_
