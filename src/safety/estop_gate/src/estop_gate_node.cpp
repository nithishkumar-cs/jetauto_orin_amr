#include "estop_gate/estop_gate_node.hpp"

#include <functional>

namespace estop_gate
{

EstopGateNode::EstopGateNode(const rclcpp::NodeOptions & options) : Node("estop_gate", options)
{
  const auto input_topic = declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel");
  const auto output_topic =
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel/safety_limited");
  const auto estop_in_topic = declare_parameter<std::string>("estop_in_topic", "/estop/engaged");
  const auto reset_in_topic = declare_parameter<std::string>("reset_in_topic", "/estop/reset");
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(10));
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic, rclcpp::QoS(10),
    std::bind(&EstopGateNode::on_command, this, std::placeholders::_1));
  estop_subscription_ = create_subscription<std_msgs::msg::Bool>(
    estop_in_topic, rclcpp::QoS(10),
    std::bind(&EstopGateNode::on_estop, this, std::placeholders::_1));
  reset_subscription_ = create_subscription<std_msgs::msg::Bool>(
    reset_in_topic, rclcpp::QoS(10),
    std::bind(&EstopGateNode::on_reset, this, std::placeholders::_1));
}

void EstopGateNode::on_command(const geometry_msgs::msg::Twist::SharedPtr command)
{
  std::scoped_lock lock(mutex_);
  if (estop_latched_) {
    publish_stop();
    return;
  }
  publisher_->publish(*command);
}

void EstopGateNode::on_estop(const std_msgs::msg::Bool::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  if (message->data) {
    estop_latched_ = true;
    publish_stop();
  }
}

void EstopGateNode::on_reset(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message->data) {
    return;
  }

  std::scoped_lock lock(mutex_);
  estop_latched_ = false;
}

void EstopGateNode::publish_stop()
{
  publisher_->publish(geometry_msgs::msg::Twist{});
}

}  // namespace estop_gate
