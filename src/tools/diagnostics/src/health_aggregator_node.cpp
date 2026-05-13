#include <amr_interfaces/msg/node_health.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace logging_and_diagnostics
{

class HealthAggregatorNode final : public rclcpp::Node
{
public:
  explicit HealthAggregatorNode(const rclcpp::NodeOptions& options)
  : Node("health_aggregator_node", options)
  {
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 1.5);
    const auto topics = declare_parameter<std::vector<std::string>>(
      "health_topics",
      std::vector<std::string>{
        "/diagnostics/rgb_camera/health",
        "/diagnostics/preproc/health",
        "/diagnostics/detector/health",
        "/diagnostics/calibration/health"});

    summary_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/system/health", 10);

    for (const auto& topic : topics) {
      subscriptions_.push_back(create_subscription<amr_interfaces::msg::NodeHealth>(
        topic,
        10,
        [this, topic](amr_interfaces::msg::NodeHealth::ConstSharedPtr msg) {
          latest_[topic] = *msg;
          received_at_[topic] = now();
        }));
      latest_[topic] = amr_interfaces::msg::NodeHealth{};
      received_at_[topic] = rclcpp::Time{0, 0, RCL_ROS_TIME};
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&HealthAggregatorNode::publish_summary, this));
  }

private:
  void publish_summary()
  {
    uint8_t worst = amr_interfaces::msg::NodeHealth::OK;
    std::ostringstream message;
    bool first = true;

    for (const auto& [topic, health] : latest_) {
      const auto received_it = received_at_.find(topic);
      const bool missing = received_it == received_at_.end() || received_it->second.nanoseconds() == 0;
      const bool stale = missing || (now() - received_it->second).seconds() > stale_timeout_s_;
      const uint8_t status = stale ? amr_interfaces::msg::NodeHealth::STALE : health.status;

      worst = std::max(worst, status);
      if (status != amr_interfaces::msg::NodeHealth::OK) {
        if (!first) {
          message << "; ";
        }
        first = false;
        message << topic << "=" << static_cast<int>(status);
        if (!health.message.empty()) {
          message << "(" << health.message << ")";
        }
      }
    }

    amr_interfaces::msg::NodeHealth summary;
    summary.header.stamp = now();
    summary.node_name = get_name();
    summary.status = worst;
    summary.message = first ? "all monitored systems healthy" : message.str();
    summary_pub_->publish(std::move(summary));
  }

  double stale_timeout_s_{1.5};
  std::map<std::string, amr_interfaces::msg::NodeHealth> latest_;
  std::map<std::string, rclcpp::Time> received_at_;
  std::vector<rclcpp::Subscription<amr_interfaces::msg::NodeHealth>::SharedPtr> subscriptions_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace logging_and_diagnostics

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<logging_and_diagnostics::HealthAggregatorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
