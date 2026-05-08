#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace teleop_tools
{

class TwistSafetyRelayNode final : public rclcpp::Node
{
public:
  explicit TwistSafetyRelayNode(const rclcpp::NodeOptions& options)
  : Node("twist_safety_relay_node", options)
  {
    max_linear_mps_ = declare_parameter<double>("max_linear_mps", 0.35);
    max_lateral_mps_ = declare_parameter<double>("max_lateral_mps", 0.25);
    max_angular_radps_ = declare_parameter<double>("max_angular_radps", 0.80);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.30);

    const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel/teleop");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/cmd_vel");

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic,
      10,
      std::bind(&TwistSafetyRelayNode::cmd_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TwistSafetyRelayNode::watchdog, this));
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    auto out = *msg;
    out.linear.x = std::clamp(out.linear.x, -max_linear_mps_, max_linear_mps_);
    out.linear.y = std::clamp(out.linear.y, -max_lateral_mps_, max_lateral_mps_);
    out.angular.z = std::clamp(out.angular.z, -max_angular_radps_, max_angular_radps_);
    last_cmd_time_ = now();
    pub_->publish(std::move(out));
  }

  void watchdog()
  {
    if (last_cmd_time_.nanoseconds() == 0) {
      return;
    }
    if ((now() - last_cmd_time_).seconds() > stale_timeout_s_) {
      pub_->publish(geometry_msgs::msg::Twist{});
      last_cmd_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    }
  }

  double max_linear_mps_{0.35};
  double max_lateral_mps_{0.25};
  double max_angular_radps_{0.80};
  double stale_timeout_s_{0.30};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace teleop_tools

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<teleop_tools::TwistSafetyRelayNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
