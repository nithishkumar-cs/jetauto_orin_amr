#ifndef PERCEPTION_LIDAR_CLUSTERING__LASER_SCAN_CLUSTERER_HPP_
#define PERCEPTION_LIDAR_CLUSTERING__LASER_SCAN_CLUSTERER_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace perception_lidar_clustering
{

struct Point2D
{
  double x_m{0.0};
  double y_m{0.0};
};

struct Cluster2D
{
  double center_x_m{0.0};
  double center_y_m{0.0};
  double size_x_m{0.0};
  double size_y_m{0.0};
  std::size_t point_count{0U};
};

struct LidarClustererConfig
{
  double max_adjacent_point_distance_m{0.20};
  std::size_t min_cluster_points{3U};
};

class LaserScanView
{
public:
  static std::optional<LaserScanView> create(
    std::size_t range_count, double angle_min_rad, double angle_max_rad, double angle_increment_rad,
    double range_min_m, double range_max_m, const float * ranges);

  std::size_t size() const;
  std::optional<Point2D> point_m(std::size_t index) const;
  bool covers_full_circle() const;

private:
  LaserScanView(
    std::size_t range_count, double angle_min_rad, double angle_max_rad, double angle_increment_rad,
    double range_min_m, double range_max_m, const float * ranges);

  std::size_t range_count_;
  double angle_min_rad_;
  double angle_max_rad_;
  double angle_increment_rad_;
  double range_min_m_;
  double range_max_m_;
  const float * ranges_;
};

class LidarClusterer
{
public:
  explicit LidarClusterer(LidarClustererConfig config = {});

  std::vector<Cluster2D> cluster(const LaserScanView & scan) const;

private:
  LidarClustererConfig config_;
};

}  // namespace perception_lidar_clustering

#endif  // PERCEPTION_LIDAR_CLUSTERING__LASER_SCAN_CLUSTERER_HPP_
