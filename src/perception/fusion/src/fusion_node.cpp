#include "perception_fusion/fusion_node.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace perception_fusion
{
namespace
{

constexpr std::size_t kCovarianceWidth = 6U;

std::optional<TrackObservation> observation_from_message(
  const amr_interfaces::msg::TrackedObstacle & obstacle)
{
  const auto & orientation = obstacle.pose.pose.orientation;
  const auto orientation_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z +
    orientation.w * orientation.w;
  if (!std::isfinite(orientation_norm_squared) || orientation_norm_squared <= 1e-12) {
    return std::nullopt;
  }
  tf2::Quaternion normalized_orientation;
  tf2::fromMsg(orientation, normalized_orientation);
  normalized_orientation.normalize();

  Eigen::Matrix2d position_covariance;
  position_covariance << obstacle.pose.covariance[0], obstacle.pose.covariance[1],
    obstacle.pose.covariance[6], obstacle.pose.covariance[7];
  Eigen::Matrix2d velocity_covariance;
  velocity_covariance << obstacle.velocity.covariance[0], obstacle.velocity.covariance[1],
    obstacle.velocity.covariance[6], obstacle.velocity.covariance[7];

  TrackObservation observation{
    obstacle.track_id,
    obstacle.pose.pose.position.x,
    obstacle.pose.pose.position.y,
    obstacle.pose.pose.position.z,
    tf2::getYaw(normalized_orientation),
    obstacle.size.x,
    obstacle.size.y,
    obstacle.size.z,
    obstacle.velocity.twist.linear.x,
    obstacle.velocity.twist.linear.y,
    obstacle.classification.class_id,
    obstacle.classification.score,
    position_covariance,
    velocity_covariance,
    obstacle.is_dynamic,
  };
  if (!valid_track_observation(observation)) {
    return std::nullopt;
  }
  return observation;
}

void set_unknown_pose_covariance(
  geometry_msgs::msg::PoseWithCovariance & pose, double unknown_variance)
{
  pose.covariance[2U * kCovarianceWidth + 2U] = unknown_variance;
  pose.covariance[3U * kCovarianceWidth + 3U] = unknown_variance;
  pose.covariance[4U * kCovarianceWidth + 4U] = unknown_variance;
  pose.covariance[5U * kCovarianceWidth + 5U] = unknown_variance;
}

void set_unknown_twist_covariance(
  geometry_msgs::msg::TwistWithCovariance & velocity, double unknown_variance)
{
  velocity.covariance[2U * kCovarianceWidth + 2U] = unknown_variance;
  velocity.covariance[3U * kCovarianceWidth + 3U] = unknown_variance;
  velocity.covariance[4U * kCovarianceWidth + 4U] = unknown_variance;
  velocity.covariance[5U * kCovarianceWidth + 5U] = unknown_variance;
}

}  // namespace

FusionNode::FusionNode(const rclcpp::NodeOptions & options) : Node("fusion", options), fusion_()
{
  auto config = FusionConfig{};
  config.cross_sensor_association_distance_m = declare_parameter<double>(
    "cross_sensor_association_distance_m", config.cross_sensor_association_distance_m);
  config.identity_reassociation_distance_m = declare_parameter<double>(
    "identity_reassociation_distance_m", config.identity_reassociation_distance_m);
  config.max_identity_age_s =
    declare_parameter<double>("max_identity_age_s", config.max_identity_age_s);
  fusion_ = MultiSensorFusion(config);

  fusion_frame_ = declare_parameter<std::string>("fusion_frame", "base_link");
  unobserved_state_variance_ = declare_parameter<double>("unobserved_state_variance", 1e6);
  const auto camera_tracks_in_topic =
    declare_parameter<std::string>("camera_tracks_in_topic", "/perception/camera/tracks");
  const auto lidar_tracks_in_topic =
    declare_parameter<std::string>("lidar_tracks_in_topic", "/perception/lidar/tracks");
  const auto fused_obstacles_out_topic =
    declare_parameter<std::string>("fused_obstacles_out_topic", "/perception/fused_obstacles");
  const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 10);
  const auto sync_max_interval_ms = declare_parameter<int>("sync_max_interval_ms", 50);
  if (
    fusion_frame_.empty() || !std::isfinite(unobserved_state_variance_) ||
    unobserved_state_variance_ <= 0.0 || camera_tracks_in_topic.empty() ||
    lidar_tracks_in_topic.empty() || fused_obstacles_out_topic.empty() || sync_queue_size <= 0 ||
    sync_max_interval_ms < 0) {
    throw std::invalid_argument(
      "Fusion frame and topics must be non-empty; unobserved variance and queue size must be "
      "positive; sync max interval must be non-negative.");
  }

  publisher_ = create_publisher<amr_interfaces::msg::TrackedObstacleArray>(
    fused_obstacles_out_topic, rclcpp::QoS(10));
  const auto input_qos = rclcpp::QoS(10).get_rmw_qos_profile();
  camera_subscription_.subscribe(this, camera_tracks_in_topic, input_qos);
  lidar_subscription_.subscribe(this, lidar_tracks_in_topic, input_qos);
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(sync_queue_size), camera_subscription_, lidar_subscription_);
  synchronizer_->setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(static_cast<double>(sync_max_interval_ms) / 1000.0));
  synchronizer_->registerCallback(std::bind(
    &FusionNode::on_synchronized_tracks, this, std::placeholders::_1, std::placeholders::_2));
}

void FusionNode::on_synchronized_tracks(
  const amr_interfaces::msg::TrackedObstacleArray::ConstSharedPtr & camera_tracks,
  const amr_interfaces::msg::TrackedObstacleArray::ConstSharedPtr & lidar_tracks)
{
  if (
    camera_tracks->header.frame_id != fusion_frame_ ||
    lidar_tracks->header.frame_id != fusion_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Camera and LiDAR tracks must both use the configured fusion frame '%s'.",
      fusion_frame_.c_str());
    return;
  }

  std::vector<TrackObservation> camera_observations;
  camera_observations.reserve(camera_tracks->obstacles.size());
  std::vector<TrackObservation> lidar_observations;
  lidar_observations.reserve(lidar_tracks->obstacles.size());
  std::size_t rejected_count = 0U;
  for (const auto & obstacle : camera_tracks->obstacles) {
    const auto observation = observation_from_message(obstacle);
    if (observation) {
      camera_observations.push_back(*observation);
    } else {
      ++rejected_count;
    }
  }
  for (const auto & obstacle : lidar_tracks->obstacles) {
    const auto observation = observation_from_message(obstacle);
    if (observation) {
      lidar_observations.push_back(*observation);
    } else {
      ++rejected_count;
    }
  }
  if (rejected_count > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%zu invalid input tracks were skipped.", rejected_count);
  }

  const auto camera_stamp = rclcpp::Time(camera_tracks->header.stamp);
  const auto lidar_stamp = rclcpp::Time(lidar_tracks->header.stamp);
  const auto output_stamp = camera_stamp >= lidar_stamp ? camera_stamp : lidar_stamp;
  std::vector<FusedObstacle> fused_obstacles;
  try {
    fused_obstacles = fusion_.fuse(output_stamp.seconds(), camera_observations, lidar_observations);
  } catch (const std::invalid_argument & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Fusion update rejected: %s", error.what());
    return;
  }

  amr_interfaces::msg::TrackedObstacleArray output;
  output.header.stamp = static_cast<builtin_interfaces::msg::Time>(output_stamp);
  output.header.frame_id = fusion_frame_;
  output.obstacles.reserve(fused_obstacles.size());
  for (const auto & fused : fused_obstacles) {
    auto & obstacle = output.obstacles.emplace_back();
    obstacle.track_id = fused.track_id;
    obstacle.classification.class_id = fused.class_id;
    obstacle.classification.score = fused.class_score;
    obstacle.pose.pose.position.x = fused.x_m;
    obstacle.pose.pose.position.y = fused.y_m;
    obstacle.pose.pose.position.z = fused.z_m;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, fused.yaw_rad);
    obstacle.pose.pose.orientation = tf2::toMsg(orientation);
    obstacle.pose.covariance[0] = fused.position_covariance(0, 0);
    obstacle.pose.covariance[1] = fused.position_covariance(0, 1);
    obstacle.pose.covariance[6] = fused.position_covariance(1, 0);
    obstacle.pose.covariance[7] = fused.position_covariance(1, 1);
    set_unknown_pose_covariance(obstacle.pose, unobserved_state_variance_);
    obstacle.size.x = fused.size_x_m;
    obstacle.size.y = fused.size_y_m;
    obstacle.size.z = fused.size_z_m;
    obstacle.velocity.twist.linear.x = fused.velocity_x_mps;
    obstacle.velocity.twist.linear.y = fused.velocity_y_mps;
    obstacle.velocity.covariance[0] = fused.velocity_covariance(0, 0);
    obstacle.velocity.covariance[1] = fused.velocity_covariance(0, 1);
    obstacle.velocity.covariance[6] = fused.velocity_covariance(1, 0);
    obstacle.velocity.covariance[7] = fused.velocity_covariance(1, 1);
    set_unknown_twist_covariance(obstacle.velocity, unobserved_state_variance_);
    obstacle.is_dynamic = fused.is_dynamic;
    obstacle.source = fused.source;
  }
  publisher_->publish(output);
}

}  // namespace perception_fusion
