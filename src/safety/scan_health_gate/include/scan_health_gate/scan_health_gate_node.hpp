#ifndef SCAN_HEALTH_GATE__SCAN_HEALTH_GATE_NODE_HPP_
#define SCAN_HEALTH_GATE__SCAN_HEALTH_GATE_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace scan_health_gate
{

class ScanHealthGateNode : public rclcpp::Node
{
public:
  explicit ScanHealthGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  void on_command(const geometry_msgs::msg::Twist::ConstSharedPtr command);
  void publish_stop_if_unhealthy();
  bool scan_is_fresh() const;
  void publish_stop();

  mutable std::mutex mutex_;
  bool scan_valid_{false};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_scan_receipt_{};
  rclcpp::Duration scan_timeout_{0, 0};
  rclcpp::Duration future_stamp_tolerance_{0, 0};
  std::chrono::duration<double> receipt_timeout_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
};

}  // namespace scan_health_gate

#endif  // SCAN_HEALTH_GATE__SCAN_HEALTH_GATE_NODE_HPP_
