#include <amr_interfaces/msg/node_health.hpp>
#include <amr_interfaces/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <string>

namespace sensor_fusion
{

class ObstacleFusionNode final : public rclcpp::Node
{
public:
  explicit ObstacleFusionNode(const rclcpp::NodeOptions& options)
  : Node("obstacle_fusion_node", options)
  {
    stale_input_timeout_s_ = declare_parameter<double>("stale_input_timeout_s", 0.50);
    const auto input_topic = declare_parameter<std::string>("input_topic", "/perception/tracked_obstacles");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/perception/fused_obstacles");

    pub_ = create_publisher<amr_interfaces::msg::ObstacleArray>(output_topic, 10);
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/fusion/health", 10);
    sub_ = create_subscription<amr_interfaces::msg::ObstacleArray>(
      input_topic,
      10,
      std::bind(&ObstacleFusionNode::callback, this, std::placeholders::_1));

    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ObstacleFusionNode::watchdog, this));
  }

private:
  void callback(const amr_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
  {
    last_input_time_ = now();

    auto output = *msg;
    for (auto& obstacle : output.obstacles) {
      obstacle.source = obstacle.source.empty() ? "tracked_obstacles" : obstacle.source + "+tracking";
    }
    pub_->publish(std::move(output));
    publish_health(amr_interfaces::msg::NodeHealth::OK, "fused obstacle stream active");
  }

  void watchdog()
  {
    if (last_input_time_.nanoseconds() == 0) {
      publish_health(amr_interfaces::msg::NodeHealth::STALE, "waiting for tracked obstacles");
      return;
    }
    if ((now() - last_input_time_).seconds() > stale_input_timeout_s_) {
      publish_health(amr_interfaces::msg::NodeHealth::STALE, "tracked obstacle stream stale");
    }
  }

  void publish_health(uint8_t status, const std::string& message)
  {
    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = now();
    health.node_name = get_name();
    health.status = status;
    health.message = message;
    health_pub_->publish(std::move(health));
  }

  double stale_input_timeout_s_{0.50};
  rclcpp::Time last_input_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<amr_interfaces::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<amr_interfaces::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace sensor_fusion

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_fusion::ObstacleFusionNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
