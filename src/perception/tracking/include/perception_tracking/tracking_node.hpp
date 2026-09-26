#ifndef PERCEPTION_TRACKING__TRACKING_NODE_HPP_
#define PERCEPTION_TRACKING__TRACKING_NODE_HPP_

#include <memory>
#include <string>

#include "amr_interfaces/msg/tracked_obstacle_array.hpp"
#include "perception_tracking/multi_object_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace perception_tracking
{

class TrackingNode : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_observations(const vision_msgs::msg::Detection3DArray::ConstSharedPtr & message);

  MultiObjectTracker tracker_;
  std::string source_;
  std::string tracking_frame_;
  std::string output_frame_;
  double transform_timeout_s_{0.05};
  double unobserved_state_variance_{1e6};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<amr_interfaces::msg::TrackedObstacleArray>::SharedPtr tracks_publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr observations_subscription_;
};

}  // namespace perception_tracking

#endif  // PERCEPTION_TRACKING__TRACKING_NODE_HPP_
