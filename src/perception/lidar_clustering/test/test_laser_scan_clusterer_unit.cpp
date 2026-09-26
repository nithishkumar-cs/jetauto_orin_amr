#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "perception_lidar_clustering/laser_scan_clusterer.hpp"

namespace perception_lidar_clustering
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

LaserScanView make_scan(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min = 0.1, double range_max = 10.0)
{
  const auto angle_max = angle_min + static_cast<double>(ranges.size() - 1U) * angle_increment;
  return LaserScanView::create(
           ranges.size(), angle_min, angle_max, angle_increment, range_min, range_max,
           ranges.data())
    .value();
}

TEST(LaserScanViewUnit, ValidatesMetadataAndRangeStorage)
{
  const std::vector<float> ranges{1.0F, 1.0F, 1.0F};
  EXPECT_TRUE(LaserScanView::create(3U, -0.1, 0.1, 0.1, 0.1, 10.0, ranges.data()));
  EXPECT_FALSE(LaserScanView::create(0U, 0.0, 0.0, 0.1, 0.1, 10.0, ranges.data()));
  EXPECT_FALSE(LaserScanView::create(3U, -0.1, 0.1, 0.0, 0.1, 10.0, ranges.data()));
  EXPECT_FALSE(LaserScanView::create(3U, -0.1, 0.1, 0.1, 1.0, 1.0, ranges.data()));
  EXPECT_FALSE(LaserScanView::create(3U, -0.1, 0.1, 0.1, 0.1, 10.0, nullptr));
  EXPECT_FALSE(LaserScanView::create(3U, -0.1, 0.3, 0.1, 0.1, 10.0, ranges.data()));
}

TEST(LaserScanViewUnit, ConvertsOnlyFiniteInRangeReturnsToCartesianPoints)
{
  const std::vector<float> ranges{0.05F, 1.0F, std::numeric_limits<float>::infinity(), 11.0F};
  const auto scan = make_scan(ranges, 0.0, kPi / 2.0);

  EXPECT_FALSE(scan.point_m(0U));
  const auto point = scan.point_m(1U);
  ASSERT_TRUE(point);
  EXPECT_NEAR(point->x_m, 0.0, 1e-6);
  EXPECT_NEAR(point->y_m, 1.0, 1e-6);
  EXPECT_FALSE(scan.point_m(2U));
  EXPECT_FALSE(scan.point_m(3U));
  EXPECT_FALSE(scan.point_m(4U));
}

TEST(LidarClustererUnit, GroupsAdjacentReturnsAndBuildsBoundingBox)
{
  const std::vector<float> ranges{1.0F, 1.0F, 1.0F};
  const auto scan = make_scan(ranges, 0.0, 0.1);
  const LidarClusterer clusterer(LidarClustererConfig{0.11, 3U});

  const auto clusters = clusterer.cluster(scan);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().point_count, 3U);
  EXPECT_NEAR(clusters.front().center_x_m, (1.0 + std::cos(0.2)) / 2.0, 1e-6);
  EXPECT_NEAR(clusters.front().center_y_m, std::sin(0.2) / 2.0, 1e-6);
  EXPECT_NEAR(clusters.front().size_x_m, 1.0 - std::cos(0.2), 1e-6);
  EXPECT_NEAR(clusters.front().size_y_m, std::sin(0.2), 1e-6);
}

TEST(LidarClustererUnit, InvalidReturnsSplitCandidatesAndSmallCandidatesAreDropped)
{
  const std::vector<float> ranges{1.0F, 1.0F, std::numeric_limits<float>::quiet_NaN(), 1.0F};
  const auto scan = make_scan(ranges, 0.0, 0.1);
  const LidarClusterer clusterer(LidarClustererConfig{0.20, 2U});

  const auto clusters = clusterer.cluster(scan);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().point_count, 2U);
}

TEST(LidarClustererUnit, SeparatesAdjacentReturnsFartherThanConfiguredGap)
{
  const std::vector<float> ranges{1.0F, 1.0F, 2.0F, 2.0F};
  const auto scan = make_scan(ranges, 0.0, 0.01);
  const LidarClusterer clusterer(LidarClustererConfig{0.20, 2U});

  const auto clusters = clusterer.cluster(scan);

  ASSERT_EQ(clusters.size(), 2U);
  EXPECT_EQ(clusters[0].point_count, 2U);
  EXPECT_EQ(clusters[1].point_count, 2U);
}

TEST(LidarClustererUnit, JoinsMatchingEndpointsAcrossAFullCircleScanSeam)
{
  const std::vector<float> ranges{
    1.0F, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 1.0F};
  const auto scan = make_scan(ranges, -kPi, kPi / 2.0);
  ASSERT_TRUE(scan.covers_full_circle());
  const LidarClusterer clusterer(LidarClustererConfig{1.5, 2U});

  const auto clusters = clusterer.cluster(scan);

  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().point_count, 2U);
}

TEST(LidarClustererUnit, RejectsInvalidConfiguration)
{
  EXPECT_THROW(LidarClusterer(LidarClustererConfig{0.0, 3U}), std::invalid_argument);
  EXPECT_THROW(
    LidarClusterer(LidarClustererConfig{std::numeric_limits<double>::quiet_NaN(), 3U}),
    std::invalid_argument);
  EXPECT_THROW(LidarClusterer(LidarClustererConfig{0.2, 0U}), std::invalid_argument);
}

}  // namespace
}  // namespace perception_lidar_clustering
