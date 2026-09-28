#include "perception_fusion/multi_sensor_fusion.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace perception_fusion
{
namespace
{

constexpr double kCovarianceSymmetryTolerance = 1e-9;

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool positive_definite(const Eigen::Matrix2d & covariance)
{
  if (
    !covariance.allFinite() ||
    !covariance.isApprox(covariance.transpose(), kCovarianceSymmetryTolerance)) {
    return false;
  }
  Eigen::LDLT<Eigen::Matrix2d> decomposition(covariance);
  return decomposition.info() == Eigen::Success && decomposition.isPositive();
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> fuse_gaussians(
  const Eigen::Vector2d & first_mean, const Eigen::Matrix2d & first_covariance,
  const Eigen::Vector2d & second_mean, const Eigen::Matrix2d & second_covariance)
{
  const auto identity = Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d first_information = first_covariance.ldlt().solve(identity);
  const Eigen::Matrix2d second_information = second_covariance.ldlt().solve(identity);
  const Eigen::Matrix2d information_sum = first_information + second_information;
  const Eigen::Matrix2d covariance = information_sum.ldlt().solve(identity);
  const Eigen::Vector2d mean =
    covariance * (first_information * first_mean + second_information * second_mean);
  return {mean, covariance};
}

double squared_distance(double first_x, double first_y, double second_x, double second_y)
{
  const auto dx = first_x - second_x;
  const auto dy = first_y - second_y;
  return dx * dx + dy * dy;
}

bool unique_track_ids(const std::vector<TrackObservation> & tracks)
{
  std::unordered_set<std::uint32_t> ids;
  for (const auto & track : tracks) {
    if (!ids.insert(track.source_track_id).second) {
      return false;
    }
  }
  return true;
}

struct AssociationCandidate
{
  std::size_t camera_index{0U};
  std::size_t lidar_index{0U};
  double squared_distance_m2{0.0};
  std::uint32_t camera_track_id{0U};
  std::uint32_t lidar_track_id{0U};
};

}  // namespace

bool valid_track_observation(const TrackObservation & observation)
{
  return observation.source_track_id != 0U && std::isfinite(observation.x_m) &&
         std::isfinite(observation.y_m) && std::isfinite(observation.z_m) &&
         std::isfinite(observation.yaw_rad) && finite_non_negative(observation.size_x_m) &&
         finite_non_negative(observation.size_y_m) && finite_non_negative(observation.size_z_m) &&
         std::isfinite(observation.velocity_x_mps) && std::isfinite(observation.velocity_y_mps) &&
         std::isfinite(observation.class_score) && observation.class_score >= 0.0 &&
         observation.class_score <= 1.0 && positive_definite(observation.position_covariance) &&
         positive_definite(observation.velocity_covariance);
}

MultiSensorFusion::MultiSensorFusion(FusionConfig config) : config_(config)
{
  if (
    !std::isfinite(config_.cross_sensor_association_distance_m) ||
    config_.cross_sensor_association_distance_m <= 0.0 ||
    !std::isfinite(config_.identity_reassociation_distance_m) ||
    config_.identity_reassociation_distance_m <= 0.0 ||
    !finite_non_negative(config_.max_identity_age_s)) {
    throw std::invalid_argument("Fusion configuration is invalid.");
  }
}

FusedObstacle MultiSensorFusion::fuse_pair(
  const TrackObservation & camera, const TrackObservation & lidar) const
{
  const auto [position, position_covariance] = fuse_gaussians(
    Eigen::Vector2d{camera.x_m, camera.y_m}, camera.position_covariance,
    Eigen::Vector2d{lidar.x_m, lidar.y_m}, lidar.position_covariance);
  const auto [velocity, velocity_covariance] = fuse_gaussians(
    Eigen::Vector2d{camera.velocity_x_mps, camera.velocity_y_mps}, camera.velocity_covariance,
    Eigen::Vector2d{lidar.velocity_x_mps, lidar.velocity_y_mps}, lidar.velocity_covariance);

  const auto * classification = &camera;
  if (
    classification->class_id.empty() ||
    (!lidar.class_id.empty() && lidar.class_score > classification->class_score)) {
    classification = &lidar;
  }

  return FusedObstacle{
    0U,
    position.x(),
    position.y(),
    camera.z_m,
    camera.yaw_rad,
    std::max(camera.size_x_m, lidar.size_x_m),
    std::max(camera.size_y_m, lidar.size_y_m),
    std::max(camera.size_z_m, lidar.size_z_m),
    velocity.x(),
    velocity.y(),
    classification->class_id,
    classification->class_score,
    position_covariance,
    velocity_covariance,
    camera.is_dynamic || lidar.is_dynamic,
    "camera+lidar",
  };
}

FusedObstacle MultiSensorFusion::copy_single(
  const TrackObservation & track, const std::string & source) const
{
  return FusedObstacle{
    0U,
    track.x_m,
    track.y_m,
    track.z_m,
    track.yaw_rad,
    track.size_x_m,
    track.size_y_m,
    track.size_z_m,
    track.velocity_x_mps,
    track.velocity_y_mps,
    track.class_id,
    track.class_score,
    track.position_covariance,
    track.velocity_covariance,
    track.is_dynamic,
    source,
  };
}

std::size_t MultiSensorFusion::resolve_identity(
  const Candidate & candidate, double timestamp_s, const std::vector<bool> & identity_used)
{
  std::vector<std::size_t> camera_matches;
  std::vector<std::size_t> lidar_matches;
  for (std::size_t index = 0U; index < identities_.size(); ++index) {
    if (identity_used[index]) {
      continue;
    }
    const auto & identity = identities_[index];
    const auto same_camera_id = candidate.camera_track_id && identity.camera_track_id &&
                                *candidate.camera_track_id == *identity.camera_track_id;
    const auto same_lidar_id = candidate.lidar_track_id && identity.lidar_track_id &&
                               *candidate.lidar_track_id == *identity.lidar_track_id;
    if (same_camera_id) {
      camera_matches.push_back(index);
    }
    if (same_lidar_id) {
      lidar_matches.push_back(index);
    }
  }

  auto better_identity = [this, &candidate](std::size_t first, std::size_t second) {
    const auto first_distance = squared_distance(
      identities_[first].x_m, identities_[first].y_m, candidate.obstacle.x_m,
      candidate.obstacle.y_m);
    const auto second_distance = squared_distance(
      identities_[second].x_m, identities_[second].y_m, candidate.obstacle.x_m,
      candidate.obstacle.y_m);
    if (first_distance != second_distance) {
      return first_distance < second_distance;
    }
    return identities_[first].fused_id < identities_[second].fused_id;
  };

  std::size_t selected_index = identities_.size();
  const auto & exact_matches = camera_matches.empty() ? lidar_matches : camera_matches;
  if (!exact_matches.empty()) {
    selected_index = *std::min_element(
      exact_matches.begin(), exact_matches.end(),
      [&better_identity](std::size_t first, std::size_t second) {
        return better_identity(first, second);
      });
  } else {
    const auto maximum_distance_squared =
      config_.identity_reassociation_distance_m * config_.identity_reassociation_distance_m;
    auto best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < identities_.size(); ++index) {
      if (identity_used[index]) {
        continue;
      }
      const auto distance = squared_distance(
        identities_[index].x_m, identities_[index].y_m, candidate.obstacle.x_m,
        candidate.obstacle.y_m);
      if (
        distance <= maximum_distance_squared &&
        (distance < best_distance ||
         (distance == best_distance && selected_index < identities_.size() &&
          identities_[index].fused_id < identities_[selected_index].fused_id))) {
        selected_index = index;
        best_distance = distance;
      }
    }
  }

  if (selected_index == identities_.size()) {
    selected_index = identities_.size();
    identities_.push_back(Identity{
      next_fused_id(), candidate.camera_track_id, candidate.lidar_track_id, candidate.obstacle.x_m,
      candidate.obstacle.y_m, timestamp_s});
  }
  return selected_index;
}

void MultiSensorFusion::attach_source_ids(std::size_t identity_index, const Candidate & candidate)
{
  if (candidate.camera_track_id) {
    for (std::size_t index = 0U; index < identities_.size(); ++index) {
      if (
        index != identity_index && identities_[index].camera_track_id &&
        *identities_[index].camera_track_id == *candidate.camera_track_id) {
        identities_[index].camera_track_id.reset();
      }
    }
    identities_[identity_index].camera_track_id = candidate.camera_track_id;
  }
  if (candidate.lidar_track_id) {
    for (std::size_t index = 0U; index < identities_.size(); ++index) {
      if (
        index != identity_index && identities_[index].lidar_track_id &&
        *identities_[index].lidar_track_id == *candidate.lidar_track_id) {
        identities_[index].lidar_track_id.reset();
      }
    }
    identities_[identity_index].lidar_track_id = candidate.lidar_track_id;
  }
}

std::uint32_t MultiSensorFusion::next_fused_id()
{
  const auto id = next_fused_id_++;
  if (next_fused_id_ == 0U) {
    next_fused_id_ = 1U;
  }
  return id;
}

std::vector<FusedObstacle> MultiSensorFusion::fuse(
  double timestamp_s, const std::vector<TrackObservation> & camera_tracks,
  const std::vector<TrackObservation> & lidar_tracks)
{
  if (!std::isfinite(timestamp_s)) {
    throw std::invalid_argument("Fusion timestamp must be finite.");
  }
  if (last_timestamp_s_ && timestamp_s < *last_timestamp_s_) {
    throw std::invalid_argument("Fusion timestamps must not move backwards.");
  }
  if (
    std::any_of(
      camera_tracks.begin(), camera_tracks.end(),
      [](const auto & track) { return !valid_track_observation(track); }) ||
    std::any_of(
      lidar_tracks.begin(), lidar_tracks.end(),
      [](const auto & track) { return !valid_track_observation(track); }) ||
    !unique_track_ids(camera_tracks) || !unique_track_ids(lidar_tracks)) {
    throw std::invalid_argument("Fusion track observation is invalid.");
  }

  identities_.erase(
    std::remove_if(
      identities_.begin(), identities_.end(),
      [this, timestamp_s](const Identity & identity) {
        return timestamp_s - identity.last_seen_timestamp_s > config_.max_identity_age_s;
      }),
    identities_.end());

  const auto maximum_distance_squared =
    config_.cross_sensor_association_distance_m * config_.cross_sensor_association_distance_m;
  std::vector<AssociationCandidate> associations;
  associations.reserve(camera_tracks.size() * lidar_tracks.size());
  for (std::size_t camera_index = 0U; camera_index < camera_tracks.size(); ++camera_index) {
    for (std::size_t lidar_index = 0U; lidar_index < lidar_tracks.size(); ++lidar_index) {
      const auto distance = squared_distance(
        camera_tracks[camera_index].x_m, camera_tracks[camera_index].y_m,
        lidar_tracks[lidar_index].x_m, lidar_tracks[lidar_index].y_m);
      if (distance <= maximum_distance_squared) {
        associations.push_back(AssociationCandidate{
          camera_index, lidar_index, distance, camera_tracks[camera_index].source_track_id,
          lidar_tracks[lidar_index].source_track_id});
      }
    }
  }
  std::sort(
    associations.begin(), associations.end(),
    [](const AssociationCandidate & first, const AssociationCandidate & second) {
      if (first.squared_distance_m2 != second.squared_distance_m2) {
        return first.squared_distance_m2 < second.squared_distance_m2;
      }
      if (first.camera_track_id != second.camera_track_id) {
        return first.camera_track_id < second.camera_track_id;
      }
      return first.lidar_track_id < second.lidar_track_id;
    });

  std::vector<bool> matched_camera(camera_tracks.size(), false);
  std::vector<bool> matched_lidar(lidar_tracks.size(), false);
  std::vector<Candidate> candidates;
  candidates.reserve(camera_tracks.size() + lidar_tracks.size());
  for (const auto & association : associations) {
    if (matched_camera[association.camera_index] || matched_lidar[association.lidar_index]) {
      continue;
    }
    candidates.push_back(Candidate{
      camera_tracks[association.camera_index].source_track_id,
      lidar_tracks[association.lidar_index].source_track_id,
      fuse_pair(camera_tracks[association.camera_index], lidar_tracks[association.lidar_index])});
    matched_camera[association.camera_index] = true;
    matched_lidar[association.lidar_index] = true;
  }
  for (std::size_t index = 0U; index < camera_tracks.size(); ++index) {
    if (!matched_camera[index]) {
      candidates.push_back(Candidate{
        camera_tracks[index].source_track_id, std::nullopt,
        copy_single(camera_tracks[index], "camera")});
    }
  }
  for (std::size_t index = 0U; index < lidar_tracks.size(); ++index) {
    if (!matched_lidar[index]) {
      candidates.push_back(Candidate{
        std::nullopt, lidar_tracks[index].source_track_id,
        copy_single(lidar_tracks[index], "lidar")});
    }
  }

  std::vector<bool> identity_used(identities_.size() + candidates.size(), false);
  std::vector<FusedObstacle> output;
  output.reserve(candidates.size());
  for (auto & candidate : candidates) {
    const auto identity_index = resolve_identity(candidate, timestamp_s, identity_used);
    if (identity_index >= identity_used.size()) {
      identity_used.resize(identity_index + 1U, false);
    }
    identity_used[identity_index] = true;
    attach_source_ids(identity_index, candidate);
    auto & identity = identities_[identity_index];
    identity.x_m = candidate.obstacle.x_m;
    identity.y_m = candidate.obstacle.y_m;
    identity.last_seen_timestamp_s = timestamp_s;
    candidate.obstacle.track_id = identity.fused_id;
    output.push_back(std::move(candidate.obstacle));
  }
  std::sort(output.begin(), output.end(), [](const auto & first, const auto & second) {
    return first.track_id < second.track_id;
  });
  last_timestamp_s_ = timestamp_s;
  return output;
}

}  // namespace perception_fusion
