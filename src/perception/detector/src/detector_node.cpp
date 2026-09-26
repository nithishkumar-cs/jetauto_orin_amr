#include "perception_detector/detector_node.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "perception_detector/detector_backend_factory.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace perception_detector
{

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("perception_detector", options)
{
  initialize(create_detector_backend(*this));
}

DetectorNode::DetectorNode(
  std::unique_ptr<DetectorBackend> backend, const rclcpp::NodeOptions & options)
: Node("perception_detector", options)
{
  if (!backend) {
    throw std::invalid_argument("DetectorNode requires a non-null injected backend");
  }
  initialize(std::move(backend));
}

void DetectorNode::initialize(std::unique_ptr<DetectorBackend> backend)
{
  const auto image_in_topic =
    declare_parameter<std::string>("image_in_topic", "/camera/rgb/image_rect_color");
  const auto detections_out_topic =
    declare_parameter<std::string>("detections_out_topic", "/perception/detections_2d");
  const auto image_queue_size = declare_parameter<int>("image_queue_size", 1);

  if (image_in_topic.empty() || detections_out_topic.empty()) {
    throw std::invalid_argument("detector topic parameters cannot be empty");
  }
  if (image_queue_size <= 0) {
    throw std::invalid_argument("detector image_queue_size must be positive");
  }

  backend_ = std::move(backend);

  detections_publisher_ =
    create_publisher<vision_msgs::msg::Detection2DArray>(detections_out_topic, rclcpp::QoS(10));
  auto image_qos = rclcpp::SensorDataQoS();
  image_qos.keep_last(static_cast<std::size_t>(image_queue_size));
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    image_in_topic, image_qos, std::bind(&DetectorNode::on_image, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Detector backend '%s': %s -> %s", backend_->name().c_str(),
    image_in_topic.c_str(), detections_out_topic.c_str());
}

void DetectorNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
{
  PixelEncoding encoding;
  if (message->encoding == sensor_msgs::image_encodings::RGB8) {
    encoding = PixelEncoding::RGB8;
  } else if (message->encoding == sensor_msgs::image_encodings::BGR8) {
    encoding = PixelEncoding::BGR8;
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "Ignoring unsupported detector image encoding '%s'",
      message->encoding.c_str());
    return;
  }

  const auto image = ImageView::create(
    message->width, message->height, message->step, encoding, message->data.data(),
    message->data.size());
  if (!image) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Ignoring malformed detector image");
    return;
  }

  std::vector<Detection> detections;
  try {
    detections = backend_->detect(*image);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Detector inference failed: %s", error.what());
    return;
  }

  vision_msgs::msg::Detection2DArray output;
  output.header = message->header;
  output.detections.reserve(detections.size());
  for (const auto & detection : detections) {
    auto & result = output.detections.emplace_back();
    result.header = message->header;
    result.bbox.center.position.x = detection.center_x;
    result.bbox.center.position.y = detection.center_y;
    result.bbox.size_x = detection.size_x;
    result.bbox.size_y = detection.size_y;
    auto & hypothesis = result.results.emplace_back();
    hypothesis.hypothesis.class_id = detection.class_id;
    hypothesis.hypothesis.score = detection.score;
  }
  detections_publisher_->publish(std::move(output));
}

}  // namespace perception_detector
