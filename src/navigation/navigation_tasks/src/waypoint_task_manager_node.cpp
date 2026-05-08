#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <functional>
#include <sstream>
#include <string>

namespace navigation_tasks
{

class WaypointTaskManagerNode final : public rclcpp::Node
{
public:
  explicit WaypointTaskManagerNode(const rclcpp::NodeOptions& options)
  : Node("waypoint_task_manager_node", options)
  {
    task_name_ = declare_parameter<std::string>("task_name", "manual_goal");
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/navigation/task_status");

    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic,
      10,
      std::bind(&WaypointTaskManagerNode::goal_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&WaypointTaskManagerNode::publish_status, this));
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    latest_goal_ = *msg;
    has_goal_ = true;
    state_ = "goal_received";
  }

  void publish_status()
  {
    std::ostringstream out;
    out << task_name_ << ":" << state_;
    if (has_goal_) {
      out << " frame=" << latest_goal_.header.frame_id
          << " x=" << latest_goal_.pose.position.x
          << " y=" << latest_goal_.pose.position.y;
    }
    std_msgs::msg::String msg;
    msg.data = out.str();
    status_pub_->publish(std::move(msg));
  }

  std::string task_name_;
  std::string state_{"idle"};
  bool has_goal_{false};
  geometry_msgs::msg::PoseStamped latest_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation_tasks

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_tasks::WaypointTaskManagerNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
