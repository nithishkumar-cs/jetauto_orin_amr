#ifndef PERCEPTION_FUSION__MULTI_SENSOR_FUSION_HPP_
#define PERCEPTION_FUSION__MULTI_SENSOR_FUSION_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace perception_fusion
{

struct FusionConfig
{
  double cross_sensor_association_distance_m{0.75};
  double identity_reassociation_distance_m{1.00};
  double max_identity_age_s{2.00};
};

struct TrackObservation
{
  std::uint32_t source_track_id{0U};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double size_x_m{0.0};
  double size_y_m{0.0};
  double size_z_m{0.0};
  double velocity_x_mps{0.0};
  double velocity_y_mps{0.0};
  std::string class_id;
  double class_score{0.0};
  Eigen::Matrix2d position_covariance{Eigen::Matrix2d::Identity()};
  Eigen::Matrix2d velocity_covariance{Eigen::Matrix2d::Identity()};
  bool is_dynamic{false};
};

struct FusedObstacle
{
  std::uint32_t track_id{0U};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double size_x_m{0.0};
  double size_y_m{0.0};
  double size_z_m{0.0};
  double velocity_x_mps{0.0};
  double velocity_y_mps{0.0};
  std::string class_id;
  double class_score{0.0};
  Eigen::Matrix2d position_covariance{Eigen::Matrix2d::Identity()};
  Eigen::Matrix2d velocity_covariance{Eigen::Matrix2d::Identity()};
  bool is_dynamic{false};
  std::string source;
};

bool valid_track_observation(const TrackObservation & observation);

class MultiSensorFusion
{
public:
  explicit MultiSensorFusion(FusionConfig config = {});

  std::vector<FusedObstacle> fuse(
    double timestamp_s, const std::vector<TrackObservation> & camera_tracks,
    const std::vector<TrackObservation> & lidar_tracks);

private:
  struct Identity
  {
    std::uint32_t fused_id{0U};
    std::optional<std::uint32_t> camera_track_id;
    std::optional<std::uint32_t> lidar_track_id;
    double x_m{0.0};
    double y_m{0.0};
    double last_seen_timestamp_s{0.0};
  };

  struct Candidate
  {
    std::optional<std::uint32_t> camera_track_id;
    std::optional<std::uint32_t> lidar_track_id;
    FusedObstacle obstacle;
  };

  FusedObstacle fuse_pair(const TrackObservation & camera, const TrackObservation & lidar) const;
  FusedObstacle copy_single(const TrackObservation & track, const std::string & source) const;
  std::size_t resolve_identity(
    const Candidate & candidate, double timestamp_s, const std::vector<bool> & identity_used);
  void attach_source_ids(std::size_t identity_index, const Candidate & candidate);
  std::uint32_t next_fused_id();

  FusionConfig config_;
  std::uint32_t next_fused_id_{1U};
  std::optional<double> last_timestamp_s_;
  std::vector<Identity> identities_;
};

}  // namespace perception_fusion

#endif  // PERCEPTION_FUSION__MULTI_SENSOR_FUSION_HPP_
