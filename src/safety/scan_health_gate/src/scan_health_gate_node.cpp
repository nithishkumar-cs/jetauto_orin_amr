#include "scan_health_gate/scan_health_gate_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace scan_health_gate
{

ScanHealthGateNode::ScanHealthGateNode(const rclcpp::NodeOptions & options)
: Node("scan_health_gate", options)
{
  const auto input_topic =
    declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel/recovery_limited");
  const auto output_topic =
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel/scan_health_limited");
  const auto scan_topic = declare_parameter<std::string>("scan_in_topic", "/scan");
  base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
  odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
  const auto timeout_sec = declare_parameter<double>("scan_timeout_sec", 0.3);
  const auto future_tolerance_sec = declare_parameter<double>("future_stamp_tolerance_sec", 0.1);
  const auto stop_rate_hz = declare_parameter<double>("stop_publish_rate_hz", 20.0);
  if (
    !std::isfinite(timeout_sec) || timeout_sec <= 0.0 || !std::isfinite(future_tolerance_sec) ||
    future_tolerance_sec < 0.0 || !std::isfinite(stop_rate_hz) || stop_rate_hz <= 0.0) {
    throw std::invalid_argument("scan timeout, timestamp tolerance, or stop rate is invalid");
  }
  if (base_frame_.empty() || odom_frame_.empty()) {
    throw std::invalid_argument("base and odom frame IDs must not be empty");
  }

  scan_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);
  future_stamp_tolerance_ = rclcpp::Duration::from_seconds(future_tolerance_sec);
  receipt_timeout_ = std::chrono::duration<double>(timeout_sec);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(10));
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic, rclcpp::QoS(10),
    std::bind(&ScanHealthGateNode::on_command, this, std::placeholders::_1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&ScanHealthGateNode::on_scan, this, std::placeholders::_1));
  const auto period = std::chrono::duration<double>(1.0 / stop_rate_hz);
  stop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ScanHealthGateNode::publish_stop_if_unhealthy, this));
}

void ScanHealthGateNode::on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  const auto stamp = rclcpp::Time(scan->header.stamp, RCL_ROS_TIME);
  const auto now = this->now();
  const bool has_usable_return =
    std::any_of(scan->ranges.begin(), scan->ranges.end(), [scan](float range) {
      return std::isfinite(range) && range >= scan->range_min && range <= scan->range_max;
    });
  const bool valid = !scan->header.frame_id.empty() && stamp.nanoseconds() != 0 &&
                     std::isfinite(scan->angle_min) && std::isfinite(scan->angle_increment) &&
                     scan->angle_increment > 0.0F && std::isfinite(scan->range_min) &&
                     std::isfinite(scan->range_max) && scan->range_min >= 0.0F &&
                     scan->range_max > scan->range_min && has_usable_return &&
                     now - stamp <= scan_timeout_ && stamp - now <= future_stamp_tolerance_;

  std::scoped_lock lock(mutex_);
  scan_valid_ = valid;
  if (valid) {
    scan_frame_ = scan->header.frame_id;
    last_scan_stamp_ = stamp;
    last_scan_receipt_ = std::chrono::steady_clock::now();
  } else {
    publish_stop();
  }
}

void ScanHealthGateNode::on_command(const geometry_msgs::msg::Twist::ConstSharedPtr command)
{
  std::scoped_lock lock(mutex_);
  if (scan_is_fresh()) {
    publisher_->publish(*command);
  } else {
    publish_stop();
  }
}

void ScanHealthGateNode::publish_stop_if_unhealthy()
{
  std::scoped_lock lock(mutex_);
  if (!scan_is_fresh()) {
    publish_stop();
  }
}

bool ScanHealthGateNode::scan_is_fresh() const
{
  if (!scan_valid_) {
    return false;
  }
  const auto now = this->now();
  const auto stamp_age = now - last_scan_stamp_;
  const auto receipt_age = std::chrono::steady_clock::now() - last_scan_receipt_;
  // Match collision_monitor's base-shift frame/time request. A zero-wait
  // lookup deliberately closes the gate until this listener has the TF.
  return stamp_age <= scan_timeout_ && last_scan_stamp_ - now <= future_stamp_tolerance_ &&
         receipt_age <= receipt_timeout_ &&
         tf_buffer_->canTransform(base_frame_, now, scan_frame_, last_scan_stamp_, odom_frame_);
}

void ScanHealthGateNode::publish_stop()
{
  publisher_->publish(geometry_msgs::msg::Twist{});
}

}  // namespace scan_health_gate
