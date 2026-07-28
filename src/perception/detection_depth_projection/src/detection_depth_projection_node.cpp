#include "perception_detection_depth_projection/detection_depth_projection_node.hpp"

#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>

namespace perception_detection_depth_projection
{
namespace
{

std::optional<DepthImage> depth_image_from_message(const sensor_msgs::msg::Image & message)
{
  if (message.data.size() < static_cast<std::size_t>(message.step) * message.height) {
    return std::nullopt;
  }
  const auto pixel_count = static_cast<std::size_t>(message.width) * message.height;
  DepthImage depth{message.width, message.height, {}};
  depth.values_m.reserve(pixel_count);

  if (message.encoding == "32FC1" && message.step >= message.width * sizeof(float)) {
    for (std::size_t row = 0U; row < message.height; ++row) {
      for (std::size_t column = 0U; column < message.width; ++column) {
        float value;
        std::memcpy(
          &value, message.data.data() + row * message.step + column * sizeof(float), sizeof(value));
        depth.values_m.push_back(value);
      }
    }
    return depth;
  }
  if (message.encoding == "16UC1" && message.step >= message.width * sizeof(std::uint16_t)) {
    for (std::size_t row = 0U; row < message.height; ++row) {
      for (std::size_t column = 0U; column < message.width; ++column) {
        std::uint16_t value_mm;
        std::memcpy(
          &value_mm, message.data.data() + row * message.step + column * sizeof(value_mm),
          sizeof(value_mm));
        depth.values_m.push_back(static_cast<float>(value_mm) / 1000.0F);
      }
    }
    return depth;
  }
  return std::nullopt;
}

}  // namespace

DetectionDepthProjectionNode::DetectionDepthProjectionNode(const rclcpp::NodeOptions & options)
: Node("detection_depth_projection", options),
  projector_(DepthProjectorConfig{
    declare_parameter<double>("center_roi_fraction", 0.25),
    declare_parameter<double>("min_depth_m", 0.15),
    declare_parameter<double>("max_depth_m", 8.0),
    static_cast<std::size_t>(declare_parameter<int>("min_valid_samples", 5)),
  })
{
  const auto detections_in_topic =
    declare_parameter<std::string>("detections_in_topic", "/perception/detections_2d");
  const auto depth_in_topic =
    declare_parameter<std::string>("depth_in_topic", "/camera/depth/image_raw");
  const auto camera_info_in_topic =
    declare_parameter<std::string>("camera_info_in_topic", "/camera/depth/camera_info");
  const auto detections_out_topic =
    declare_parameter<std::string>("detections_out_topic", "/perception/detections_3d");
  const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 10);

  publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(detections_out_topic, 10);
  detections_subscription_.subscribe(
    this, detections_in_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  depth_subscription_.subscribe(
    this, depth_in_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  camera_info_subscription_.subscribe(
    this, camera_info_in_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(sync_queue_size), detections_subscription_, depth_subscription_,
    camera_info_subscription_);
  synchronizer_->registerCallback(std::bind(
    &DetectionDepthProjectionNode::on_synchronized_input, this, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3));
}

void DetectionDepthProjectionNode::on_synchronized_input(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_message,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  if (
    camera_info->header.frame_id != depth_message->header.frame_id ||
    (!detections->header.frame_id.empty() &&
     detections->header.frame_id != depth_message->header.frame_id)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Detection, depth, and camera-info frames must match before depth projection.");
    return;
  }

  const auto depth = depth_image_from_message(*depth_message);
  if (!depth) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Depth encoding must be 16UC1 (millimetres) or 32FC1 (metres).");
    return;
  }

  const CameraIntrinsics intrinsics{
    camera_info->k[0], camera_info->k[4], camera_info->k[2], camera_info->k[5]};
  vision_msgs::msg::Detection3DArray output;
  output.header = depth_message->header;
  output.header.stamp = detections->header.stamp;

  for (const auto & detection : detections->detections) {
    const BoundingBox2D bbox{
      detection.bbox.center.position.x,
      detection.bbox.center.position.y,
      detection.bbox.size_x,
      detection.bbox.size_y,
    };
    const auto projection = projector_.project(bbox, *depth, intrinsics);
    if (!projection) {
      continue;
    }

    vision_msgs::msg::Detection3D projected_detection;
    projected_detection.header = output.header;
    projected_detection.results = detection.results;
    for (auto & result : projected_detection.results) {
      result.pose.pose.position.x = projection->x_m;
      result.pose.pose.position.y = projection->y_m;
      result.pose.pose.position.z = projection->z_m;
      result.pose.pose.orientation.w = 1.0;
    }
    projected_detection.id = detection.id;
    projected_detection.bbox.center.position.x = projection->x_m;
    projected_detection.bbox.center.position.y = projection->y_m;
    projected_detection.bbox.center.position.z = projection->z_m;
    projected_detection.bbox.center.orientation.w = 1.0;
    projected_detection.bbox.size.x = projection->width_m;
    projected_detection.bbox.size.y = projection->height_m;
    projected_detection.bbox.size.z = 0.0;
    output.detections.push_back(projected_detection);
  }
  publisher_->publish(output);
}

}  // namespace perception_detection_depth_projection
