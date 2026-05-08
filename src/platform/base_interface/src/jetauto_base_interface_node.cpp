#include <amr_interfaces/msg/node_health.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace base_interface
{

class JetAutoBaseInterfaceNode final : public rclcpp::Node
{
public:
  explicit JetAutoBaseInterfaceNode(const rclcpp::NodeOptions& options)
  : Node("jetauto_base_interface_node", options)
  {
    transport_mode_ = declare_parameter<std::string>("transport_mode", "stub");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.25);
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);

    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel/safety_limited");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom/wheel");

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic,
      10,
      std::bind(&JetAutoBaseInterfaceNode::cmd_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    health_pub_ = create_publisher<amr_interfaces::msg::NodeHealth>("/diagnostics/base/health", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&JetAutoBaseInterfaceNode::update, this));

    RCLCPP_WARN_EXPRESSION(
      get_logger(),
      transport_mode_ == "stub",
      "Base interface is running in stub mode; replace with JetAuto controller transport before robot motion.");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = now();
  }

  void update()
  {
    const auto stamp = now();
    const double dt = last_update_time_.nanoseconds() == 0 ?
      1.0 / update_rate_hz_ :
      (stamp - last_update_time_).seconds();
    last_update_time_ = stamp;

    geometry_msgs::msg::Twist active_cmd = last_cmd_;
    if (last_cmd_time_.nanoseconds() == 0 || (stamp - last_cmd_time_).seconds() > cmd_timeout_s_) {
      active_cmd = geometry_msgs::msg::Twist{};
    }

    if (transport_mode_ == "stub") {
      integrate_stub_odom(active_cmd, dt);
    }

    publish_odom(stamp, active_cmd);
    publish_health(
      transport_mode_ == "stub" ? amr_interfaces::msg::NodeHealth::WARN : amr_interfaces::msg::NodeHealth::OK,
      transport_mode_ == "stub" ? "stub odometry mode" : "base transport active");
  }

  void integrate_stub_odom(const geometry_msgs::msg::Twist& cmd, double dt)
  {
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    x_ += (cmd.linear.x * cos_yaw - cmd.linear.y * sin_yaw) * dt;
    y_ += (cmd.linear.x * sin_yaw + cmd.linear.y * cos_yaw) * dt;
    yaw_ += cmd.angular.z * dt;
  }

  void publish_odom(const rclcpp::Time& stamp, const geometry_msgs::msg::Twist& twist)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    odom.twist.twist = twist;
    odom_pub_->publish(std::move(odom));
  }

  void publish_health(uint8_t status, const std::string& message)
  {
    amr_interfaces::msg::NodeHealth health;
    health.header.stamp = now();
    health.node_name = get_name();
    health.status = status;
    health.message = message;
    health.frequency_hz = static_cast<float>(update_rate_hz_);
    health_pub_->publish(std::move(health));
  }

  std::string transport_mode_;
  std::string odom_frame_;
  std::string base_frame_;
  double cmd_timeout_s_{0.25};
  double update_rate_hz_{50.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<amr_interfaces::msg::NodeHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace base_interface

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<base_interface::JetAutoBaseInterfaceNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
