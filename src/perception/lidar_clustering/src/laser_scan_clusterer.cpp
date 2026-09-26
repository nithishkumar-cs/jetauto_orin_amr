#include "perception_lidar_clustering/laser_scan_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace perception_lidar_clustering
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

double squared_distance(const Point2D & first, const Point2D & second)
{
  const auto dx = first.x_m - second.x_m;
  const auto dy = first.y_m - second.y_m;
  return dx * dx + dy * dy;
}

Cluster2D bounding_box(const std::vector<Point2D> & points)
{
  auto min_x = points.front().x_m;
  auto max_x = points.front().x_m;
  auto min_y = points.front().y_m;
  auto max_y = points.front().y_m;
  for (const auto & point : points) {
    min_x = std::min(min_x, point.x_m);
    max_x = std::max(max_x, point.x_m);
    min_y = std::min(min_y, point.y_m);
    max_y = std::max(max_y, point.y_m);
  }
  return Cluster2D{
    (min_x + max_x) / 2.0, (min_y + max_y) / 2.0, max_x - min_x, max_y - min_y, points.size(),
  };
}

}  // namespace

std::optional<LaserScanView> LaserScanView::create(
  std::size_t range_count, double angle_min_rad, double angle_max_rad, double angle_increment_rad,
  double range_min_m, double range_max_m, const float * ranges)
{
  if (
    range_count == 0U || ranges == nullptr || !std::isfinite(angle_min_rad) ||
    !std::isfinite(angle_max_rad) || !std::isfinite(angle_increment_rad) ||
    angle_increment_rad == 0.0 || !std::isfinite(range_min_m) || !std::isfinite(range_max_m) ||
    range_min_m < 0.0 || range_max_m <= range_min_m) {
    return std::nullopt;
  }

  const auto expected_angle_max =
    angle_min_rad + static_cast<double>(range_count - 1U) * angle_increment_rad;
  const auto angle_tolerance = std::max(std::abs(angle_increment_rad) * 0.51, 1e-6);
  if (std::abs(expected_angle_max - angle_max_rad) > angle_tolerance) {
    return std::nullopt;
  }

  return LaserScanView{range_count, angle_min_rad, angle_max_rad, angle_increment_rad,
                       range_min_m, range_max_m,   ranges};
}

LaserScanView::LaserScanView(
  std::size_t range_count, double angle_min_rad, double angle_max_rad, double angle_increment_rad,
  double range_min_m, double range_max_m, const float * ranges)
: range_count_(range_count),
  angle_min_rad_(angle_min_rad),
  angle_max_rad_(angle_max_rad),
  angle_increment_rad_(angle_increment_rad),
  range_min_m_(range_min_m),
  range_max_m_(range_max_m),
  ranges_(ranges)
{
}

std::size_t LaserScanView::size() const
{
  return range_count_;
}

std::optional<Point2D> LaserScanView::point_m(std::size_t index) const
{
  if (index >= range_count_) {
    return std::nullopt;
  }

  const auto range_m = static_cast<double>(ranges_[index]);
  if (!std::isfinite(range_m) || range_m < range_min_m_ || range_m > range_max_m_) {
    return std::nullopt;
  }

  const auto angle_rad = angle_min_rad_ + static_cast<double>(index) * angle_increment_rad_;
  return Point2D{range_m * std::cos(angle_rad), range_m * std::sin(angle_rad)};
}

bool LaserScanView::covers_full_circle() const
{
  const auto beam_width = std::abs(angle_increment_rad_);
  const auto angular_coverage = std::abs(angle_max_rad_ - angle_min_rad_) + beam_width;
  return angular_coverage >= kTwoPi - beam_width * 0.51;
}

LidarClusterer::LidarClusterer(LidarClustererConfig config) : config_(config)
{
  if (
    !std::isfinite(config_.max_adjacent_point_distance_m) ||
    config_.max_adjacent_point_distance_m <= 0.0 || config_.min_cluster_points == 0U) {
    throw std::invalid_argument(
      "Maximum adjacent-point distance and minimum cluster size must be positive.");
  }
}

std::vector<Cluster2D> LidarClusterer::cluster(const LaserScanView & scan) const
{
  std::vector<std::vector<Point2D>> candidates;
  std::vector<Point2D> current;
  const auto max_distance_squared =
    config_.max_adjacent_point_distance_m * config_.max_adjacent_point_distance_m;

  for (std::size_t index = 0U; index < scan.size(); ++index) {
    const auto point = scan.point_m(index);
    if (!point) {
      if (!current.empty()) {
        candidates.push_back(std::move(current));
        current.clear();
      }
      continue;
    }

    if (!current.empty() && squared_distance(current.back(), *point) > max_distance_squared) {
      candidates.push_back(std::move(current));
      current.clear();
    }
    current.push_back(*point);
  }
  if (!current.empty()) {
    candidates.push_back(std::move(current));
  }

  if (candidates.size() > 1U && scan.covers_full_circle()) {
    const auto first_point = scan.point_m(0U);
    const auto last_point = scan.point_m(scan.size() - 1U);
    if (
      first_point && last_point &&
      squared_distance(*first_point, *last_point) <= max_distance_squared) {
      auto trailing = std::move(candidates.back());
      candidates.pop_back();
      trailing.insert(trailing.end(), candidates.front().begin(), candidates.front().end());
      candidates.front() = std::move(trailing);
    }
  }

  std::vector<Cluster2D> clusters;
  clusters.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    if (candidate.size() >= config_.min_cluster_points) {
      clusters.push_back(bounding_box(candidate));
    }
  }
  return clusters;
}

}  // namespace perception_lidar_clustering
