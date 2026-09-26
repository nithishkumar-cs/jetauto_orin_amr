#ifndef PERCEPTION_TRACKING__MULTI_OBJECT_TRACKER_HPP_
#define PERCEPTION_TRACKING__MULTI_OBJECT_TRACKER_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace perception_tracking
{

struct TrackerConfig
{
  double association_distance_m{0.75};
  double process_noise_acceleration_stddev_mps2{1.0};
  double measurement_noise_position_stddev_m{0.15};
  double initial_position_stddev_m{0.15};
  double initial_velocity_stddev_mps{1.0};
  double max_unobserved_duration_s{0.50};
  std::size_t min_confirmed_hits{2U};
  double dynamic_speed_threshold_mps{0.20};
};

struct Observation
{
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  double size_x_m{0.0};
  double size_y_m{0.0};
  double size_z_m{0.0};
  std::string class_id;
  double class_score{0.0};
};

struct TrackEstimate
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
  Eigen::Matrix4d state_covariance{Eigen::Matrix4d::Zero()};
  bool is_dynamic{false};
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(TrackerConfig config = {});

  std::vector<TrackEstimate> update(
    double timestamp_s, const std::vector<Observation> & observations);

private:
  struct Track
  {
    std::uint32_t id{0U};
    Eigen::Vector4d state{Eigen::Vector4d::Zero()};
    Eigen::Matrix4d covariance{Eigen::Matrix4d::Zero()};
    double z_m{0.0};
    double yaw_rad{0.0};
    double size_x_m{0.0};
    double size_y_m{0.0};
    double size_z_m{0.0};
    std::string class_id;
    double class_score{0.0};
    double last_observed_timestamp_s{0.0};
    std::size_t hit_count{0U};
  };

  void validate_observation(const Observation & observation) const;
  void predict(Track & track, double dt_s) const;
  void correct(Track & track, const Observation & observation, double timestamp_s) const;
  Track make_track(const Observation & observation, double timestamp_s);
  TrackEstimate estimate(const Track & track) const;

  TrackerConfig config_;
  std::uint32_t next_track_id_{1U};
  std::optional<double> last_update_timestamp_s_;
  std::vector<Track> tracks_;
};

}  // namespace perception_tracking

#endif  // PERCEPTION_TRACKING__MULTI_OBJECT_TRACKER_HPP_
