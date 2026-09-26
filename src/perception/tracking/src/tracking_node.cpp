#include "perception_tracking/tracking_node.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"

namespace perception_tracking
{
namespace
{

constexpr std::size_t kCovarianceWidth = 6U;

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool valid_box(const vision_msgs::msg::Detection3D & detection)
{
  const auto & position = detection.bbox.center.position;
  const auto & orientation = detection.bbox.center.orientation;
  const auto & size = detection.bbox.size;
  const auto orientation_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z +
    orientation.w * orientation.w;
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
         std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
         orientation_norm_squared > 1e-12 && finite_non_negative(size.x) &&
         finite_non_negative(size.y) && finite_non_negative(size.z);
}

std::optional<vision_msgs::msg::ObjectHypothesis> best_hypothesis(
  const vision_msgs::msg::Detection3D & detection)
{
  std::optional<vision_msgs::msg::ObjectHypothesis> best;
  for (const auto & result : detection.results) {
    const auto & hypothesis = result.hypothesis;
    if (
      hypothesis.class_id.empty() || !std::isfinite(hypothesis.score) || hypothesis.score < 0.0 ||
      hypothesis.score > 1.0) {
      continue;
    }
    if (!best || hypothesis.score > best->score) {
      best = hypothesis;
    }
  }
  return best;
}

Eigen::Matrix2d rotation_matrix(double yaw_rad)
{
  const auto cosine = std::cos(yaw_rad);
  const auto sine = std::sin(yaw_rad);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;
  return rotation;
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

TrackingNode::TrackingNode(const rclcpp::NodeOptions & options)
: Node("tracking", options),
  tracker_(),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_, this, true)
{
  auto config = TrackerConfig{};
  config.association_distance_m =
    declare_parameter<double>("association_distance_m", config.association_distance_m);
  config.process_noise_acceleration_stddev_mps2 = declare_parameter<double>(
    "process_noise_acceleration_stddev_mps2", config.process_noise_acceleration_stddev_mps2);
  config.measurement_noise_position_stddev_m = declare_parameter<double>(
    "measurement_noise_position_stddev_m", config.measurement_noise_position_stddev_m);
  config.initial_position_stddev_m =
    declare_parameter<double>("initial_position_stddev_m", config.initial_position_stddev_m);
  config.initial_velocity_stddev_mps =
    declare_parameter<double>("initial_velocity_stddev_mps", config.initial_velocity_stddev_mps);
  config.max_unobserved_duration_s =
    declare_parameter<double>("max_unobserved_duration_s", config.max_unobserved_duration_s);
  const auto min_confirmed_hits =
    declare_parameter<int>("min_confirmed_hits", static_cast<int>(config.min_confirmed_hits));
  if (min_confirmed_hits <= 0) {
    throw std::invalid_argument("min_confirmed_hits must be positive.");
  }
  config.min_confirmed_hits = static_cast<std::size_t>(min_confirmed_hits);
  config.dynamic_speed_threshold_mps =
    declare_parameter<double>("dynamic_speed_threshold_mps", config.dynamic_speed_threshold_mps);
  tracker_ = MultiObjectTracker(config);

  source_ = declare_parameter<std::string>("source", "");
  tracking_frame_ = declare_parameter<std::string>("tracking_frame", "odom");
  output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
  const auto transform_timeout_ms = declare_parameter<int>("transform_timeout_ms", 50);
  unobserved_state_variance_ = declare_parameter<double>("unobserved_state_variance", 1e6);
  if (
    source_.empty() || tracking_frame_.empty() || output_frame_.empty() ||
    transform_timeout_ms < 0 || !std::isfinite(unobserved_state_variance_) ||
    unobserved_state_variance_ <= 0.0) {
    throw std::invalid_argument(
      "Tracking source and frames must be non-empty; transform timeout must be "
      "non-negative; unobserved variance must be positive.");
  }
  transform_timeout_s_ = static_cast<double>(transform_timeout_ms) / 1000.0;

  const auto observations_in_topic =
    declare_parameter<std::string>("observations_in_topic", "/perception/detections_3d");
  const auto tracks_out_topic =
    declare_parameter<std::string>("tracks_out_topic", "/perception/tracks");
  const auto observations_queue_size = declare_parameter<int>("observations_queue_size", 10);
  if (observations_queue_size <= 0) {
    throw std::invalid_argument("observations_queue_size must be positive.");
  }

  tracks_publisher_ =
    create_publisher<amr_interfaces::msg::TrackedObstacleArray>(tracks_out_topic, rclcpp::QoS(10));
  observations_subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    observations_in_topic, rclcpp::QoS(observations_queue_size),
    std::bind(&TrackingNode::on_observations, this, std::placeholders::_1));
}

void TrackingNode::on_observations(
  const vision_msgs::msg::Detection3DArray::ConstSharedPtr & message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Observation frame_id is empty; detections were not tracked.");
    return;
  }

  const auto stamp = rclcpp::Time(message->header.stamp);
  const auto timeout = rclcpp::Duration::from_seconds(transform_timeout_s_);
  geometry_msgs::msg::TransformStamped observation_to_tracking;
  geometry_msgs::msg::TransformStamped tracking_to_output;
  try {
    observation_to_tracking =
      tf_buffer_.lookupTransform(tracking_frame_, message->header.frame_id, stamp, timeout);
    tracking_to_output = tf_buffer_.lookupTransform(output_frame_, tracking_frame_, stamp, timeout);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Tracking transform unavailable: %s", error.what());
    return;
  }

  std::vector<Observation> observations;
  observations.reserve(message->detections.size());
  std::size_t rejected_count = 0U;
  for (const auto & detection : message->detections) {
    if (
      (!detection.header.frame_id.empty() &&
       detection.header.frame_id != message->header.frame_id) ||
      !valid_box(detection)) {
      ++rejected_count;
      continue;
    }

    geometry_msgs::msg::PoseStamped input_pose;
    input_pose.header = message->header;
    input_pose.pose = detection.bbox.center;
    tf2::Quaternion input_orientation;
    tf2::fromMsg(input_pose.pose.orientation, input_orientation);
    input_orientation.normalize();
    input_pose.pose.orientation = tf2::toMsg(input_orientation);
    geometry_msgs::msg::PoseStamped tracking_pose;
    tf2::doTransform(input_pose, tracking_pose, observation_to_tracking);

    const auto classification = best_hypothesis(detection);
    observations.push_back(Observation{
      tracking_pose.pose.position.x,
      tracking_pose.pose.position.y,
      tracking_pose.pose.position.z,
      tf2::getYaw(tracking_pose.pose.orientation),
      detection.bbox.size.x,
      detection.bbox.size.y,
      detection.bbox.size.z,
      classification ? classification->class_id : "",
      classification ? classification->score : 0.0,
    });
  }
  if (rejected_count > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%zu invalid detections were skipped.", rejected_count);
  }

  std::vector<TrackEstimate> tracks;
  try {
    tracks = tracker_.update(stamp.seconds(), observations);
  } catch (const std::invalid_argument & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Tracking update rejected: %s", error.what());
    return;
  }

  const auto transform_yaw = tf2::getYaw(tracking_to_output.transform.rotation);
  const auto covariance_rotation = rotation_matrix(transform_yaw);
  tf2::Quaternion output_rotation;
  tf2::fromMsg(tracking_to_output.transform.rotation, output_rotation);

  amr_interfaces::msg::TrackedObstacleArray output;
  output.header.stamp = message->header.stamp;
  output.header.frame_id = output_frame_;
  output.obstacles.reserve(tracks.size());
  for (const auto & track : tracks) {
    geometry_msgs::msg::PoseStamped tracking_pose;
    tracking_pose.header.stamp = message->header.stamp;
    tracking_pose.header.frame_id = tracking_frame_;
    tracking_pose.pose.position.x = track.x_m;
    tracking_pose.pose.position.y = track.y_m;
    tracking_pose.pose.position.z = track.z_m;
    tf2::Quaternion track_orientation;
    track_orientation.setRPY(0.0, 0.0, track.yaw_rad);
    tracking_pose.pose.orientation = tf2::toMsg(track_orientation);
    geometry_msgs::msg::PoseStamped output_pose;
    tf2::doTransform(tracking_pose, output_pose, tracking_to_output);

    const tf2::Vector3 tracking_velocity{track.velocity_x_mps, track.velocity_y_mps, 0.0};
    const auto output_velocity = tf2::quatRotate(output_rotation, tracking_velocity);
    const Eigen::Matrix2d position_covariance = covariance_rotation *
                                                track.state_covariance.block<2, 2>(0, 0) *
                                                covariance_rotation.transpose();
    const Eigen::Matrix2d velocity_covariance = covariance_rotation *
                                                track.state_covariance.block<2, 2>(2, 2) *
                                                covariance_rotation.transpose();

    auto & obstacle = output.obstacles.emplace_back();
    obstacle.track_id = track.track_id;
    obstacle.classification.class_id = track.class_id;
    obstacle.classification.score = track.class_score;
    obstacle.pose.pose = output_pose.pose;
    obstacle.pose.covariance[0] = position_covariance(0, 0);
    obstacle.pose.covariance[1] = position_covariance(0, 1);
    obstacle.pose.covariance[6] = position_covariance(1, 0);
    obstacle.pose.covariance[7] = position_covariance(1, 1);
    set_unknown_pose_covariance(obstacle.pose, unobserved_state_variance_);
    obstacle.size.x = track.size_x_m;
    obstacle.size.y = track.size_y_m;
    obstacle.size.z = track.size_z_m;
    obstacle.velocity.twist.linear.x = output_velocity.x();
    obstacle.velocity.twist.linear.y = output_velocity.y();
    obstacle.velocity.twist.linear.z = output_velocity.z();
    obstacle.velocity.covariance[0] = velocity_covariance(0, 0);
    obstacle.velocity.covariance[1] = velocity_covariance(0, 1);
    obstacle.velocity.covariance[6] = velocity_covariance(1, 0);
    obstacle.velocity.covariance[7] = velocity_covariance(1, 1);
    set_unknown_twist_covariance(obstacle.velocity, unobserved_state_variance_);
    obstacle.is_dynamic = track.is_dynamic;
    obstacle.source = source_;
  }
  tracks_publisher_->publish(output);
}

}  // namespace perception_tracking
