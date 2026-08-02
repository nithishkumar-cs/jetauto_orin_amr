#include "perception_detection_depth_projection/detection_depth_projection_node.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>

#include "geometry_msgs/msg/pose.hpp"

namespace perception_detection_depth_projection
{
namespace
{

std::optional<DepthImageView> depth_view_from_ros_message(const sensor_msgs::msg::Image & message)
{
  DepthEncoding encoding;
  if (message.encoding == "32FC1") {
    encoding = DepthEncoding::FLOAT32_METERS;
  } else if (message.encoding == "16UC1") {
    encoding = DepthEncoding::UINT16_MILLIMETERS;
  } else {
    return std::nullopt;
  }

  return DepthImageView::create(
    message.width, message.height, message.step, encoding, message.is_bigendian != 0U,
    message.data.data(), message.data.size());
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
    declare_parameter<std::string>("depth_in_topic", "/camera/aligned_depth_to_rgb/image_raw");
  const auto camera_info_in_topic =
    declare_parameter<std::string>("camera_info_in_topic", "/camera/rgb/camera_info");
  const auto detections_out_topic =
    declare_parameter<std::string>("detections_out_topic", "/perception/detections_3d");
  const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 10);
  const auto sync_max_interval_ms = declare_parameter<int>("sync_max_interval_ms", 50);
  if (sync_queue_size <= 0 || sync_max_interval_ms < 0) {
    throw std::invalid_argument("Sync queue size must be positive and max interval non-negative.");
  }

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
  synchronizer_->setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(static_cast<double>(sync_max_interval_ms) / 1000.0));
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
    camera_info->header.frame_id.empty() ||
    camera_info->header.frame_id != depth_message->header.frame_id ||
    detections->header.frame_id != depth_message->header.frame_id ||
    camera_info->width != depth_message->width || camera_info->height != depth_message->height) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Detections, aligned depth, and CameraInfo must use one frame and image size.");
    return;
  }

  const auto depth = depth_view_from_ros_message(*depth_message);
  if (!depth) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Depth image must have a valid buffer and use 16UC1 (millimetres) or 32FC1 (metres).");
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

    geometry_msgs::msg::Pose projected_pose;
    projected_pose.position.x = projection->x_m;
    projected_pose.position.y = projection->y_m;
    projected_pose.position.z = projection->z_m;
    projected_pose.orientation.w = 1.0;

    vision_msgs::msg::Detection3D projected_detection;
    projected_detection.header = output.header;
    projected_detection.results.reserve(detection.results.size());
    for (const auto & result_2d : detection.results) {
      auto & result_3d = projected_detection.results.emplace_back();
      result_3d.hypothesis = result_2d.hypothesis;
      result_3d.pose.pose = projected_pose;
    }
    projected_detection.id = detection.id;
    projected_detection.bbox.center = projected_pose;
    projected_detection.bbox.size.x = projection->width_m;
    projected_detection.bbox.size.y = projection->height_m;
    projected_detection.bbox.size.z = 0.0;
    output.detections.push_back(projected_detection);
  }
  publisher_->publish(output);
}

}  // namespace perception_detection_depth_projection
