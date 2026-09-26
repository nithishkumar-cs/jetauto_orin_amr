#ifndef PERCEPTION_FUSION__FUSION_NODE_HPP_
#define PERCEPTION_FUSION__FUSION_NODE_HPP_

#include <memory>
#include <string>

#include "amr_interfaces/msg/tracked_obstacle_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "perception_fusion/multi_sensor_fusion.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_fusion
{

class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    amr_interfaces::msg::TrackedObstacleArray, amr_interfaces::msg::TrackedObstacleArray>;

  void on_synchronized_tracks(
    const amr_interfaces::msg::TrackedObstacleArray::ConstSharedPtr & camera_tracks,
    const amr_interfaces::msg::TrackedObstacleArray::ConstSharedPtr & lidar_tracks);

  MultiSensorFusion fusion_;
  std::string fusion_frame_;
  double unobserved_state_variance_{1e6};
  rclcpp::Publisher<amr_interfaces::msg::TrackedObstacleArray>::SharedPtr publisher_;
  message_filters::Subscriber<amr_interfaces::msg::TrackedObstacleArray> camera_subscription_;
  message_filters::Subscriber<amr_interfaces::msg::TrackedObstacleArray> lidar_subscription_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
};

}  // namespace perception_fusion

#endif  // PERCEPTION_FUSION__FUSION_NODE_HPP_
