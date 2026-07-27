#ifndef ESTOP_GATE__ESTOP_GATE_NODE_HPP_
#define ESTOP_GATE__ESTOP_GATE_NODE_HPP_

#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace estop_gate
{

class EstopGateNode : public rclcpp::Node
{
public:
  explicit EstopGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_command(const geometry_msgs::msg::Twist::SharedPtr command);
  void on_estop(const std_msgs::msg::Bool::SharedPtr message);
  void on_reset(const std_msgs::msg::Bool::SharedPtr message);
  void publish_stop();

  std::mutex mutex_;
  bool estop_latched_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_subscription_;
};

}  // namespace estop_gate

#endif  // ESTOP_GATE__ESTOP_GATE_NODE_HPP_
