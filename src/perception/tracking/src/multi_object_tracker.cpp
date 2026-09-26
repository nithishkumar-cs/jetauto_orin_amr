#include "perception_tracking/multi_object_tracker.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace perception_tracking
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

double normalized_angle(double angle_rad)
{
  return std::remainder(angle_rad, kTwoPi);
}

struct AssociationCandidate
{
  std::size_t track_index{0U};
  std::size_t observation_index{0U};
  double squared_distance_m2{0.0};
  std::uint32_t track_id{0U};
};

}  // namespace

MultiObjectTracker::MultiObjectTracker(TrackerConfig config) : config_(config)
{
  if (
    !std::isfinite(config_.association_distance_m) || config_.association_distance_m <= 0.0 ||
    !finite_non_negative(config_.process_noise_acceleration_stddev_mps2) ||
    !std::isfinite(config_.measurement_noise_position_stddev_m) ||
    config_.measurement_noise_position_stddev_m <= 0.0 ||
    !std::isfinite(config_.initial_position_stddev_m) || config_.initial_position_stddev_m <= 0.0 ||
    !std::isfinite(config_.initial_velocity_stddev_mps) ||
    config_.initial_velocity_stddev_mps <= 0.0 ||
    !finite_non_negative(config_.max_unobserved_duration_s) || config_.min_confirmed_hits == 0U ||
    !finite_non_negative(config_.dynamic_speed_threshold_mps)) {
    throw std::invalid_argument("Tracker configuration is invalid.");
  }
}

void MultiObjectTracker::validate_observation(const Observation & observation) const
{
  if (
    !std::isfinite(observation.x_m) || !std::isfinite(observation.y_m) ||
    !std::isfinite(observation.z_m) || !std::isfinite(observation.yaw_rad) ||
    !finite_non_negative(observation.size_x_m) || !finite_non_negative(observation.size_y_m) ||
    !finite_non_negative(observation.size_z_m) || !std::isfinite(observation.class_score) ||
    observation.class_score < 0.0 || observation.class_score > 1.0) {
    throw std::invalid_argument("Tracker observation is invalid.");
  }
}

void MultiObjectTracker::predict(Track & track, double dt_s) const
{
  Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
  transition(0, 2) = dt_s;
  transition(1, 3) = dt_s;

  const auto dt2 = dt_s * dt_s;
  const auto dt3 = dt2 * dt_s;
  const auto dt4 = dt2 * dt2;
  const auto acceleration_variance =
    config_.process_noise_acceleration_stddev_mps2 * config_.process_noise_acceleration_stddev_mps2;
  Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
  process_noise(0, 0) = dt4 / 4.0;
  process_noise(0, 2) = dt3 / 2.0;
  process_noise(1, 1) = dt4 / 4.0;
  process_noise(1, 3) = dt3 / 2.0;
  process_noise(2, 0) = dt3 / 2.0;
  process_noise(2, 2) = dt2;
  process_noise(3, 1) = dt3 / 2.0;
  process_noise(3, 3) = dt2;
  process_noise *= acceleration_variance;

  track.state = transition * track.state;
  track.covariance = transition * track.covariance * transition.transpose() + process_noise;
}

void MultiObjectTracker::correct(
  Track & track, const Observation & observation, double timestamp_s) const
{
  Eigen::Matrix<double, 2, 4> observation_model = Eigen::Matrix<double, 2, 4>::Zero();
  observation_model(0, 0) = 1.0;
  observation_model(1, 1) = 1.0;
  const auto measurement_variance =
    config_.measurement_noise_position_stddev_m * config_.measurement_noise_position_stddev_m;
  const auto measurement_noise = Eigen::Matrix2d::Identity() * measurement_variance;
  const Eigen::Vector2d measurement{observation.x_m, observation.y_m};
  const auto innovation = measurement - observation_model * track.state;
  const Eigen::Matrix2d innovation_covariance =
    observation_model * track.covariance * observation_model.transpose() + measurement_noise;
  const Eigen::Matrix<double, 4, 2> kalman_gain =
    innovation_covariance.ldlt().solve(observation_model * track.covariance).transpose();

  track.state += kalman_gain * innovation;
  const auto identity = Eigen::Matrix4d::Identity();
  const auto residual_transform = identity - kalman_gain * observation_model;
  track.covariance = residual_transform * track.covariance * residual_transform.transpose() +
                     kalman_gain * measurement_noise * kalman_gain.transpose();
  track.z_m = observation.z_m;
  track.yaw_rad = normalized_angle(observation.yaw_rad);
  track.size_x_m = observation.size_x_m;
  track.size_y_m = observation.size_y_m;
  track.size_z_m = observation.size_z_m;
  if (!observation.class_id.empty()) {
    track.class_id = observation.class_id;
    track.class_score = observation.class_score;
  }
  track.last_observed_timestamp_s = timestamp_s;
  ++track.hit_count;
}

MultiObjectTracker::Track MultiObjectTracker::make_track(
  const Observation & observation, double timestamp_s)
{
  Track track;
  track.id = next_track_id_++;
  if (next_track_id_ == 0U) {
    next_track_id_ = 1U;
  }
  track.state << observation.x_m, observation.y_m, 0.0, 0.0;
  const auto position_variance =
    config_.initial_position_stddev_m * config_.initial_position_stddev_m;
  const auto velocity_variance =
    config_.initial_velocity_stddev_mps * config_.initial_velocity_stddev_mps;
  track.covariance.diagonal() << position_variance, position_variance, velocity_variance,
    velocity_variance;
  track.z_m = observation.z_m;
  track.yaw_rad = normalized_angle(observation.yaw_rad);
  track.size_x_m = observation.size_x_m;
  track.size_y_m = observation.size_y_m;
  track.size_z_m = observation.size_z_m;
  track.class_id = observation.class_id;
  track.class_score = observation.class_score;
  track.last_observed_timestamp_s = timestamp_s;
  track.hit_count = 1U;
  return track;
}

TrackEstimate MultiObjectTracker::estimate(const Track & track) const
{
  return TrackEstimate{
    track.id,
    track.state(0),
    track.state(1),
    track.z_m,
    track.yaw_rad,
    track.size_x_m,
    track.size_y_m,
    track.size_z_m,
    track.state(2),
    track.state(3),
    track.class_id,
    track.class_score,
    track.covariance,
    std::hypot(track.state(2), track.state(3)) >= config_.dynamic_speed_threshold_mps,
  };
}

std::vector<TrackEstimate> MultiObjectTracker::update(
  double timestamp_s, const std::vector<Observation> & observations)
{
  if (!std::isfinite(timestamp_s)) {
    throw std::invalid_argument("Tracker timestamp must be finite.");
  }
  if (last_update_timestamp_s_ && timestamp_s < *last_update_timestamp_s_) {
    throw std::invalid_argument("Tracker timestamps must not move backwards.");
  }
  for (const auto & observation : observations) {
    validate_observation(observation);
  }

  const auto dt_s = last_update_timestamp_s_ ? timestamp_s - *last_update_timestamp_s_ : 0.0;
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this, timestamp_s](const Track & track) {
        return timestamp_s - track.last_observed_timestamp_s > config_.max_unobserved_duration_s;
      }),
    tracks_.end());
  for (auto & track : tracks_) {
    predict(track, dt_s);
  }

  const auto maximum_distance_squared =
    config_.association_distance_m * config_.association_distance_m;
  std::vector<AssociationCandidate> candidates;
  candidates.reserve(tracks_.size() * observations.size());
  for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
    for (std::size_t observation_index = 0U; observation_index < observations.size();
         ++observation_index) {
      const auto dx = tracks_[track_index].state(0) - observations[observation_index].x_m;
      const auto dy = tracks_[track_index].state(1) - observations[observation_index].y_m;
      const auto squared_distance = dx * dx + dy * dy;
      if (squared_distance <= maximum_distance_squared) {
        candidates.push_back(AssociationCandidate{
          track_index, observation_index, squared_distance, tracks_[track_index].id});
      }
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const AssociationCandidate & first, const AssociationCandidate & second) {
      if (first.squared_distance_m2 != second.squared_distance_m2) {
        return first.squared_distance_m2 < second.squared_distance_m2;
      }
      if (first.track_id != second.track_id) {
        return first.track_id < second.track_id;
      }
      return first.observation_index < second.observation_index;
    });

  std::vector<bool> matched_tracks(tracks_.size(), false);
  std::vector<bool> matched_observations(observations.size(), false);
  for (const auto & candidate : candidates) {
    if (
      matched_tracks[candidate.track_index] || matched_observations[candidate.observation_index]) {
      continue;
    }
    correct(tracks_[candidate.track_index], observations[candidate.observation_index], timestamp_s);
    matched_tracks[candidate.track_index] = true;
    matched_observations[candidate.observation_index] = true;
  }

  for (std::size_t index = 0U; index < observations.size(); ++index) {
    if (!matched_observations[index]) {
      tracks_.push_back(make_track(observations[index], timestamp_s));
    }
  }

  last_update_timestamp_s_ = timestamp_s;

  std::vector<TrackEstimate> estimates;
  estimates.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    if (track.hit_count >= config_.min_confirmed_hits) {
      estimates.push_back(estimate(track));
    }
  }
  return estimates;
}

}  // namespace perception_tracking
