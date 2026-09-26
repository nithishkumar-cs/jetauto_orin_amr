#ifndef PERCEPTION_DETECTOR__DETECTOR_NODE_HPP_
#define PERCEPTION_DETECTOR__DETECTOR_NODE_HPP_

#include <memory>

#include "perception_detector/detector_backend.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace perception_detector
{

class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Injection seam for tests and externally assembled detector processes.
  DetectorNode(
    std::unique_ptr<DetectorBackend> backend,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initialize(std::unique_ptr<DetectorBackend> backend);
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message);

  std::unique_ptr<DetectorBackend> backend_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_publisher_;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR__DETECTOR_NODE_HPP_
