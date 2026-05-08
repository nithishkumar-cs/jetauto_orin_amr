#include "safety_layer/safety_policy.hpp"

#include <amr_interfaces/msg/obstacle_array.hpp>
#include <amr_interfaces/msg/safety_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace safety_layer
{

class SafetyMonitorNode final : public rclcpp::Node
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions& options)
  : Node("safety_monitor_node", options)
  {
    config_.stop_distance_m = declare_parameter<double>("stop_distance_m", config_.stop_distance_m);
    config_.slow_distance_m = declare_parameter<double>("slow_distance_m", config_.slow_distance_m);
    config_.lateral_half_width_m = declare_parameter<double>("lateral_half_width_m", config_.lateral_half_width_m);
    config_.stale_input_timeout_s = declare_parameter<double>("stale_input_timeout_s", config_.stale_input_timeout_s);
    config_.slow_scale = declare_parameter<double>("slow_scale", config_.slow_scale);

    const auto obstacles_topic = declare_parameter<std::string>("obstacles_topic", "/perception/fused_obstacles");
    const auto cmd_in_topic = declare_parameter<std::string>("cmd_vel_in", "/cmd_vel");
    const auto cmd_out_topic = declare_parameter<std::string>("cmd_vel_out", "/cmd_vel/safety_limited");
    const auto safety_topic = declare_parameter<std::string>("safety_state_topic", "/safety/state");

    obstacle_sub_ = create_subscription<amr_interfaces::msg::ObstacleArray>(
      obstacles_topic,
      10,
      std::bind(&SafetyMonitorNode::obstacles_callback, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_in_topic,
      10,
      std::bind(&SafetyMonitorNode::cmd_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic, 10);
    safety_pub_ = create_publisher<amr_interfaces::msg::SafetyState>(safety_topic, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SafetyMonitorNode::publish_state, this));
  }

private:
  void obstacles_callback(const amr_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_obstacles_ = msg->obstacles;
    last_obstacle_time_ = now();
  }

  void cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    const auto decision = current_decision();
    geometry_msgs::msg::Twist gated = *msg;

    gated.linear.x *= decision.command_scale;
    gated.linear.y *= decision.command_scale;
    if (decision.code == SafetyCode::Stop || decision.code == SafetyCode::SensorDegraded) {
      gated.angular.z = 0.0;
    }

    cmd_pub_->publish(std::move(gated));
  }

  SafetyDecision current_decision()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double age = last_obstacle_time_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() :
      (now() - last_obstacle_time_).seconds();
    latest_age_s_ = age;
    return evaluate_obstacles(latest_obstacles_, config_, age);
  }

  void publish_state()
  {
    const auto decision = current_decision();

    amr_interfaces::msg::SafetyState state;
    state.header.stamp = now();
    state.state = static_cast<uint8_t>(decision.code);
    state.reason = decision.reason;
    state.nearest_obstacle_m = static_cast<float>(decision.nearest_obstacle_m);
    const auto bounded_age = std::isfinite(latest_age_s_) ? latest_age_s_ : 999.0;
    const auto age_ns = static_cast<int64_t>(bounded_age * 1e9);
    state.input_age.sec = static_cast<int32_t>(age_ns / 1000000000);
    state.input_age.nanosec = static_cast<uint32_t>(age_ns % 1000000000);
    safety_pub_->publish(std::move(state));
  }

  SafetyConfig config_;
  std::mutex mutex_;
  std::vector<amr_interfaces::msg::Obstacle> latest_obstacles_;
  rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};
  double latest_age_s_{0.0};

  rclcpp::Subscription<amr_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<amr_interfaces::msg::SafetyState>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace safety_layer

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_layer::SafetyMonitorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
