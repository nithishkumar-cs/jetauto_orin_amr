#include "perception_preproc/image_preprocessor.hpp"

#include <amr_interfaces/msg/node_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <array>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace perception_preproc
{

class PreprocNode final : public rclcpp::Node
{
public:
  explicit PreprocNode(const rclcpp::NodeOptions& options)
  : Node("preproc_node", options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/sensors/rgb/image_raw");
    tensor_topic_ = declare_parameter<std::string>("tensor_topic", "/perception/preproc/tensor_debug");

    ImagePreprocessConfig config;
    config.network_width = declare_parameter<int>("network_width", 640);
    config.network_height = declare_parameter<int>("network_height", 640);
    config.keep_aspect_ratio = declare_parameter<bool>("keep_aspect_ratio", true);
    config.pad_value = static_cast<float>(declare_parameter<double>("pad_value", 114.0));
    config.scale = static_cast<float>(declare_parameter<double>("scale", 1.0 / 255.0));

    const auto mean = declare_parameter<std::vector<double>>("mean", {0.0, 0.0, 0.0});
    const auto stddev = declare_parameter<std::vector<double>>("stddev", {1.0, 1.0, 1.0});
    if (mean.size() != 3 || stddev.size() != 3) {
      throw std::runtime_error("mean and stddev parameters must contain exactly three values");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      config.mean[i] = static_cast<float>(mean[i]);
      config.stddev[i] = static_cast<float>(stddev[i]);
    }

    preprocessor_ = std::make_unique<GpuImagePreprocessor>(config);

    tensor_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(tensor_topic_, 2);
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/preproc/health", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PreprocNode::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    HostImageView view;
    view.data = msg->data.data();
    view.width = static_cast<int>(msg->width);
    view.height = static_cast<int>(msg->height);
    view.step = static_cast<int>(msg->step);

    if (msg->encoding == "bgr8") {
      view.channels = 3;
      view.input_is_bgr = true;
    } else if (msg->encoding == "rgb8") {
      view.channels = 3;
      view.input_is_bgr = false;
    } else if (msg->encoding == "mono8") {
      view.channels = 1;
      view.input_is_bgr = false;
    } else {
      publish_health(amr_interfaces::msg::NodeHealth::WARN, "unsupported image encoding");
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unsupported image encoding: %s",
        msg->encoding.c_str());
      return;
    }

    try {
      const auto stats = preprocessor_->preprocess_to_host(view, tensor_);
      publish_tensor();
      publish_health(amr_interfaces::msg::NodeHealth::OK, "preprocessing active");
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "CUDA preproc %ux%u %s -> 1x3x%dx%d kernel=%.3f ms",
        msg->width,
        msg->height,
        msg->encoding.c_str(),
        preprocessor_->config().network_height,
        preprocessor_->config().network_width,
        stats.kernel_ms);
    } catch (const std::exception& ex) {
      publish_health(amr_interfaces::msg::NodeHealth::ERROR, ex.what());
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Preprocessing failed: %s", ex.what());
    }
  }

  void publish_tensor()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(4);
    msg.layout.dim[0].label = "n";
    msg.layout.dim[0].size = 1;
    msg.layout.dim[0].stride = static_cast<uint32_t>(tensor_.size());
    msg.layout.dim[1].label = "c";
    msg.layout.dim[1].size = 3;
    msg.layout.dim[1].stride = static_cast<uint32_t>(preprocessor_->config().network_width * preprocessor_->config().network_height);
    msg.layout.dim[2].label = "h";
    msg.layout.dim[2].size = static_cast<uint32_t>(preprocessor_->config().network_height);
    msg.layout.dim[2].stride = static_cast<uint32_t>(preprocessor_->config().network_width);
    msg.layout.dim[3].label = "w";
    msg.layout.dim[3].size = static_cast<uint32_t>(preprocessor_->config().network_width);
    msg.layout.dim[3].stride = 1;
    msg.data = tensor_;
    tensor_pub_->publish(std::move(msg));
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
  std::string tensor_topic_;
  std::unique_ptr<GpuImagePreprocessor> preprocessor_;
  std::vector<float> tensor_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tensor_pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
};

}  // namespace perception_preproc

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_preproc::PreprocNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
