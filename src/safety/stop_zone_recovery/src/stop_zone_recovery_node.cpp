#include "stop_zone_recovery/stop_zone_recovery_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace stop_zone_recovery
{

std::size_t count_returns_in_box(
  const sensor_msgs::msg::LaserScan & scan, const PlanarTransform & to_base, const RearBox & box)
{
  const double c = std::cos(to_base.yaw);
  const double s = std::sin(to_base.yaw);
  std::size_t count = 0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double px = r * std::cos(a);
    const double py = r * std::sin(a);
    const double bx = to_base.tx + c * px - s * py;
    const double by = to_base.ty + s * px + c * py;
    if (bx >= box.x_min && bx <= box.x_max && std::abs(by) <= box.half_width) {
      ++count;
    }
  }
  return count;
}

StopZoneRecoveryNode::StopZoneRecoveryNode(const rclcpp::NodeOptions & options)
: Node("stop_zone_recovery", options)
{
  const auto raw_topic = declare_parameter<std::string>("raw_cmd_vel_in_topic", "/cmd_vel");
  const auto limited_topic =
    declare_parameter<std::string>("limited_cmd_vel_in_topic", "/cmd_vel/collision_limited");
  const auto output_topic =
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel/recovery_limited");
  const auto scan_topic = declare_parameter<std::string>("scan_in_topic", "/scan");
  base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
  max_reverse_speed_ = declare_parameter<double>("max_reverse_speed", 0.05);
  axis_epsilon_ = declare_parameter<double>("off_axis_epsilon", 1e-3);
  box_.x_min = declare_parameter<double>("rear_box_x_min", -1.0);
  box_.x_max = declare_parameter<double>("rear_box_x_max", 0.0);
  box_.half_width = declare_parameter<double>("rear_box_half_width", 0.65);
  const auto min_points = declare_parameter<int>("min_blocking_points", 1);
  const auto scan_timeout = declare_parameter<double>("scan_timeout_sec", 0.3);
  const auto command_timeout = declare_parameter<double>("command_timeout_sec", 0.3);
  const auto future_tolerance = declare_parameter<double>("future_stamp_tolerance_sec", 0.1);
  const auto stop_hold = declare_parameter<double>("stop_hold_sec", 0.5);
  if (
    !std::isfinite(max_reverse_speed_) || max_reverse_speed_ <= 0.0 ||
    !std::isfinite(axis_epsilon_) || axis_epsilon_ < 0.0 || !std::isfinite(box_.x_min) ||
    !std::isfinite(box_.x_max) || box_.x_min >= box_.x_max || !std::isfinite(box_.half_width) ||
    box_.half_width <= 0.0 || min_points < 1 || !std::isfinite(scan_timeout) ||
    scan_timeout <= 0.0 || !std::isfinite(command_timeout) || command_timeout <= 0.0 ||
    !std::isfinite(future_tolerance) || future_tolerance < 0.0 || !std::isfinite(stop_hold) ||
    stop_hold < 0.0 || base_frame_.empty()) {
    throw std::invalid_argument("stop_zone_recovery parameters are invalid");
  }
  min_blocking_points_ = static_cast<std::size_t>(min_points);
  scan_timeout_ = rclcpp::Duration::from_seconds(scan_timeout);
  command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);
  future_tolerance_ = rclcpp::Duration::from_seconds(future_tolerance);
  stop_hold_ = rclcpp::Duration::from_seconds(stop_hold);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(10));
  raw_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    raw_topic, rclcpp::QoS(10),
    std::bind(&StopZoneRecoveryNode::on_raw_command, this, std::placeholders::_1));
  limited_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    limited_topic, rclcpp::QoS(10),
    std::bind(&StopZoneRecoveryNode::on_limited_command, this, std::placeholders::_1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&StopZoneRecoveryNode::on_scan, this, std::placeholders::_1));
}

void StopZoneRecoveryNode::on_raw_command(const geometry_msgs::msg::Twist::ConstSharedPtr command)
{
  std::scoped_lock lock(mutex_);
  last_raw_ = *command;
  last_raw_receipt_ = now();
}

void StopZoneRecoveryNode::on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  const auto stamp = rclcpp::Time(scan->header.stamp, RCL_ROS_TIME);
  const auto t = now();
  bool ok = !scan->header.frame_id.empty() && stamp.nanoseconds() != 0 &&
            std::isfinite(scan->angle_min) && std::isfinite(scan->angle_increment) &&
            scan->angle_increment > 0.0F && std::isfinite(scan->range_min) &&
            std::isfinite(scan->range_max) && scan->range_max > scan->range_min &&
            t - stamp <= scan_timeout_ && stamp - t <= future_tolerance_;
  bool clear = false;
  if (ok) {
    try {
      // Lidar-to-base is a rigid mount, so the latest transform is the right one.
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, scan->header.frame_id, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const PlanarTransform to_base{
        tf.transform.translation.x, tf.transform.translation.y,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
      clear = count_returns_in_box(*scan, to_base, box_) < min_blocking_points_;
    } catch (const tf2::TransformException &) {
      ok = false;
    }
  }
  std::scoped_lock lock(mutex_);
  scan_ok_ = ok;
  rear_clear_ = ok && clear;
  last_scan_receipt_ = t;
}

bool StopZoneRecoveryNode::rear_is_clear() const
{
  return scan_ok_ && rear_clear_ && now() - last_scan_receipt_ <= scan_timeout_;
}

void StopZoneRecoveryNode::on_limited_command(
  const geometry_msgs::msg::Twist::ConstSharedPtr command)
{
  std::scoped_lock lock(mutex_);
  const bool monitor_zeroed =
    command->linear.x == 0.0 && command->linear.y == 0.0 && command->angular.z == 0.0;
  const auto & raw = last_raw_;
  const bool pure_reverse = std::isfinite(raw.linear.x) && raw.linear.x < 0.0 &&
                            std::abs(raw.linear.y) <= axis_epsilon_ &&
                            std::abs(raw.linear.z) <= axis_epsilon_ &&
                            std::abs(raw.angular.z) <= axis_epsilon_;
  const auto t = now();
  const bool raw_fresh = t - last_raw_receipt_ <= command_timeout_;
  const bool raw_nonzero = std::abs(raw.linear.x) > axis_epsilon_ ||
                           std::abs(raw.linear.y) > axis_epsilon_ ||
                           std::abs(raw.angular.z) > axis_epsilon_;
  if (monitor_zeroed && raw_fresh && raw_nonzero) {
    stop_armed_ = true;
    last_stop_ = t;
  }
  if (monitor_zeroed && pure_reverse && raw_fresh && rear_is_clear()) {
    geometry_msgs::msg::Twist reverse;
    reverse.linear.x = std::max(raw.linear.x, -max_reverse_speed_);
    publisher_->publish(reverse);
  } else if (!monitor_zeroed && stop_armed_ && t - last_stop_ <= stop_hold_) {
    // A nonzero monitor decision can be based on a scan received before the latest stop.
    publisher_->publish(geometry_msgs::msg::Twist{});
  } else {
    publisher_->publish(*command);
  }
}

}  // namespace stop_zone_recovery
