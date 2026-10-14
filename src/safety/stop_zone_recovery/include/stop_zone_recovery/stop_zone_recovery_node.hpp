#ifndef STOP_ZONE_RECOVERY__STOP_ZONE_RECOVERY_NODE_HPP_
#define STOP_ZONE_RECOVERY__STOP_ZONE_RECOVERY_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace stop_zone_recovery
{

/// Axis-aligned box in base_link that must be free of scan returns before reversing.
struct RearBox
{
  double x_min;
  double x_max;
  double half_width;
};

/// Planar rigid transform from the scan frame into base_link.
struct PlanarTransform
{
  double tx;
  double ty;
  double yaw;
};

/// Counts finite, in-range scan returns whose base_link position lies inside `box`.
std::size_t count_returns_in_box(
  const sensor_msgs::msg::LaserScan & scan, const PlanarTransform & to_base, const RearBox & box);

/**
 * Sits between collision_monitor and scan_health_gate. Forwards the monitor output
 * unchanged, except when the monitor has zeroed a pure-reverse command: then it emits
 * that reverse at a capped speed iff a fresh scan shows the rear box clear. Anything
 * else (stale scan, no TF, rear return, strafe/rotation, stale command) forwards the
 * monitor output, so the monitor's stop stays in force. Not safety-rated.
 */
class StopZoneRecoveryNode : public rclcpp::Node
{
public:
  explicit StopZoneRecoveryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_raw_command(const geometry_msgs::msg::Twist::ConstSharedPtr command);
  void on_limited_command(const geometry_msgs::msg::Twist::ConstSharedPtr command);
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  bool rear_is_clear() const;

  mutable std::mutex mutex_;
  std::string base_frame_;
  RearBox box_{};
  double max_reverse_speed_{0.0};
  double axis_epsilon_{0.0};
  std::size_t min_blocking_points_{1};
  rclcpp::Duration scan_timeout_{0, 0};
  rclcpp::Duration command_timeout_{0, 0};
  rclcpp::Duration future_tolerance_{0, 0};
  geometry_msgs::msg::Twist last_raw_;
  rclcpp::Time last_raw_receipt_{0, 0, RCL_ROS_TIME};
  bool scan_ok_{false};
  bool rear_clear_{false};
  rclcpp::Time last_scan_receipt_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr limited_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace stop_zone_recovery

#endif  // STOP_ZONE_RECOVERY__STOP_ZONE_RECOVERY_NODE_HPP_
