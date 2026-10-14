#ifndef PERCEPTION_DETECTOR__DETECTOR_NODE_HPP_
#define PERCEPTION_DETECTOR__DETECTOR_NODE_HPP_

#include <memory>

#include "perception_detector/detector_backend.hpp"
#include "perception_metrics/metrics_reporter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace perception_detector
{

/// Convert camera images into detections and optional ROS timing diagnostics.
class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Injection seam for tests and externally assembled detector processes.
  DetectorNode(
    std::unique_ptr<DetectorBackend> backend,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Configure detector topics and optional diagnostics publication.
  void initialize(std::unique_ptr<DetectorBackend> backend);
  /// Process a valid image and publish detections with optional per-frame timing.
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message);

  std::unique_ptr<DetectorBackend> backend_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_publisher_;
  perception_metrics::MetricsReporter metrics_;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__DETECTOR_NODE_HPP_
