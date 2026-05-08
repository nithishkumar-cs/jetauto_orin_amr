#include <amr_interfaces/msg/detection2_d_array.hpp>
#include <amr_interfaces/msg/node_health.hpp>
#include <amr_interfaces/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <string>

namespace benchmarks
{

class RateCounter
{
public:
  void tick()
  {
    ++count_;
  }

  double sample_and_reset(double period_s)
  {
    const double rate = period_s > 0.0 ? static_cast<double>(count_) / period_s : 0.0;
    count_ = 0;
    return rate;
  }

private:
  uint64_t count_{0};
};

class LatencyProbeNode final : public rclcpp::Node
{
public:
  explicit LatencyProbeNode(const rclcpp::NodeOptions& options)
  : Node("latency_probe_node", options)
  {
    report_period_s_ = declare_parameter<double>("report_period_s", 2.0);
    const auto image_topic = declare_parameter<std::string>("image_topic", "/sensors/rgb/image_raw");
    const auto detections_topic = declare_parameter<std::string>("detections_topic", "/perception/detections_2d");
    const auto obstacles_topic = declare_parameter<std::string>("obstacles_topic", "/perception/fused_obstacles");

    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/benchmarks/latency", 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr) { image_rate_.tick(); });
    detections_sub_ = create_subscription<amr_interfaces::msg::Detection2DArray>(
      detections_topic,
      10,
      [this](amr_interfaces::msg::Detection2DArray::ConstSharedPtr msg) {
        detection_rate_.tick();
        last_detection_latency_ms_ = stamp_latency_ms(msg->header.stamp);
      });
    obstacles_sub_ = create_subscription<amr_interfaces::msg::ObstacleArray>(
      obstacles_topic,
      10,
      [this](amr_interfaces::msg::ObstacleArray::ConstSharedPtr msg) {
        obstacle_rate_.tick();
        last_obstacle_latency_ms_ = stamp_latency_ms(msg->header.stamp);
      });

    last_report_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(report_period_s_)),
      std::bind(&LatencyProbeNode::report, this));
  }

private:
  double stamp_latency_ms(const builtin_interfaces::msg::Time& stamp_msg) const
  {
    const rclcpp::Time stamp(stamp_msg, RCL_ROS_TIME);
    if (stamp.nanoseconds() == 0) {
      return -1.0;
    }
    return (now() - stamp).seconds() * 1000.0;
  }

  void report()
  {
    const auto stamp = now();
    const double period_s = std::max(1e-3, (stamp - last_report_).seconds());
    last_report_ = stamp;

    std::ostringstream message;
    message << "image_fps=" << image_rate_.sample_and_reset(period_s)
            << " detection_fps=" << detection_rate_.sample_and_reset(period_s)
            << " obstacle_fps=" << obstacle_rate_.sample_and_reset(period_s)
            << " detection_latency_ms=" << last_detection_latency_ms_
            << " obstacle_latency_ms=" << last_obstacle_latency_ms_;

    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = stamp;
    health.node_name = get_name();
    health.status = amr_interfaces::msg::NodeHealth::OK;
    health.message = message.str();
    health_pub_->publish(std::move(health));
    RCLCPP_INFO(get_logger(), "%s", message.str().c_str());
  }

  double report_period_s_{2.0};
  double last_detection_latency_ms_{-1.0};
  double last_obstacle_latency_ms_{-1.0};
  RateCounter image_rate_;
  RateCounter detection_rate_;
  RateCounter obstacle_rate_;
  rclcpp::Time last_report_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<amr_interfaces::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<amr_interfaces::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace benchmarks

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<benchmarks::LatencyProbeNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
