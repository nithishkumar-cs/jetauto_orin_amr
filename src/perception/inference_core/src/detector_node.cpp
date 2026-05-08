#include "perception_inference/detector_backend.hpp"

#include <amr_interfaces/msg/detection2_d_array.hpp>
#include <amr_interfaces/msg/node_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace perception_inference
{

class DetectorNode final : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions& options)
  : Node("detector_node", options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/sensors/rgb/image_raw");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/perception/detections_2d");

    DetectorConfig config;
    config.backend = declare_parameter<std::string>("backend", "debug");
    config.model_name = declare_parameter<std::string>("model_name", config.backend);
    config.engine_path = declare_parameter<std::string>("engine_path", "");
    config.labels_path = declare_parameter<std::string>("labels_path", "");
    config.confidence_threshold = static_cast<float>(declare_parameter<double>("confidence_threshold", 0.25));
    config.publish_debug_detections = declare_parameter<bool>("publish_debug_detections", true);
    source_model_ = config.model_name;

    backend_ = make_detector_backend(config);
    if (!backend_) {
      throw std::runtime_error("failed to create detector backend");
    }

    detections_pub_ = create_publisher<amr_interfaces::msg::Detection2DArray>(detections_topic_, 10);
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/detector/health", 10);

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Detector backend %s: %s", backend_->id().c_str(), backend_->status().c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto detections = backend_->detect(*msg);
    const auto t1 = std::chrono::steady_clock::now();
    const auto latency_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    amr_interfaces::msg::Detection2DArray out;
    out.header = msg->header;
    out.source_model = source_model_;
    out.detections = std::move(detections);
    detections_pub_->publish(std::move(out));

    const uint8_t status = backend_->ready() ?
      amr_interfaces::msg::NodeHealth::OK :
      amr_interfaces::msg::NodeHealth::WARN;
    publish_health(status, backend_->status());

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Detector %s produced frame result in %ld us",
      backend_->id().c_str(),
      latency_us);
  }

  void publish_health(uint8_t status, const std::string& message)
  {
    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = now();
    health.node_name = get_name();
    health.status = status;
    health.message = message;
    health.frequency_hz = 0.0f;
    health_pub_->publish(std::move(health));
  }

  std::string image_topic_;
  std::string detections_topic_;
  std::string source_model_;
  std::unique_ptr<DetectorBackend> backend_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<amr_interfaces::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
};

}  // namespace perception_inference

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_inference::DetectorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
