#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "perception_fusion/multi_sensor_fusion.hpp"

namespace perception_fusion
{
namespace
{

TrackObservation track(
  std::uint32_t id, double x_m, double y_m, double position_variance = 0.04,
  std::string class_id = "person")
{
  TrackObservation observation;
  observation.source_track_id = id;
  observation.x_m = x_m;
  observation.y_m = y_m;
  observation.z_m = 0.8;
  observation.yaw_rad = 0.1;
  observation.size_x_m = 0.6;
  observation.size_y_m = 0.4;
  observation.size_z_m = 1.7;
  observation.velocity_x_mps = 0.4;
  observation.velocity_y_mps = -0.1;
  observation.class_id = std::move(class_id);
  observation.class_score = observation.class_id.empty() ? 0.0 : 0.9;
  observation.position_covariance = Eigen::Matrix2d::Identity() * position_variance;
  observation.velocity_covariance = Eigen::Matrix2d::Identity() * 0.25;
  observation.is_dynamic = true;
  return observation;
}

TEST(MultiSensorFusionUnit, CovarianceWeightsMatchedTracksAndCombinesMetadata)
{
  MultiSensorFusion fusion;
  auto camera = track(11U, 1.0, 2.0, 0.04, "person");
  auto lidar = track(21U, 1.2, 2.0, 0.01, "");
  lidar.z_m = 0.0;
  lidar.size_x_m = 0.8;
  lidar.size_y_m = 0.7;
  lidar.size_z_m = 0.0;
  lidar.velocity_x_mps = 0.0;
  lidar.is_dynamic = false;

  const auto output = fusion.fuse(1.0, {camera}, {lidar});

  ASSERT_EQ(output.size(), 1U);
  const auto & obstacle = output.front();
  EXPECT_EQ(obstacle.track_id, 1U);
  EXPECT_NEAR(obstacle.x_m, 1.16, 1e-12);
  EXPECT_NEAR(obstacle.y_m, 2.0, 1e-12);
  EXPECT_NEAR(obstacle.position_covariance(0, 0), 0.008, 1e-12);
  EXPECT_EQ(obstacle.class_id, "person");
  EXPECT_DOUBLE_EQ(obstacle.class_score, 0.9);
  EXPECT_DOUBLE_EQ(obstacle.z_m, camera.z_m);
  EXPECT_DOUBLE_EQ(obstacle.size_x_m, 0.8);
  EXPECT_DOUBLE_EQ(obstacle.size_y_m, 0.7);
  EXPECT_DOUBLE_EQ(obstacle.size_z_m, 1.7);
  EXPECT_TRUE(obstacle.is_dynamic);
  EXPECT_EQ(obstacle.source, "camera+lidar");
}

TEST(MultiSensorFusionUnit, KeepsUnmatchedSourcesAsSeparateGloballyUniqueTracks)
{
  auto config = FusionConfig{};
  config.cross_sensor_association_distance_m = 0.5;
  MultiSensorFusion fusion(config);

  const auto output = fusion.fuse(1.0, {track(1U, 0.0, 0.0)}, {track(1U, 3.0, 0.0)});

  ASSERT_EQ(output.size(), 2U);
  EXPECT_NE(output[0].track_id, output[1].track_id);
  EXPECT_EQ(output[0].source, "camera");
  EXPECT_EQ(output[1].source, "lidar");
}

TEST(MultiSensorFusionUnit, AssociatesOneToOneIndependentlyOfInputOrder)
{
  MultiSensorFusion fusion;
  const auto first = fusion.fuse(
    1.0, {track(10U, 0.0, 0.0), track(20U, 5.0, 0.0)},
    {track(100U, 0.1, 0.0), track(200U, 5.1, 0.0)});
  ASSERT_EQ(first.size(), 2U);
  const auto first_low = std::min_element(
    first.begin(), first.end(),
    [](const auto & left, const auto & right) { return left.x_m < right.x_m; });
  const auto first_high = std::max_element(
    first.begin(), first.end(),
    [](const auto & left, const auto & right) { return left.x_m < right.x_m; });

  const auto second = fusion.fuse(
    1.1, {track(20U, 5.2, 0.0), track(10U, 0.2, 0.0)},
    {track(200U, 5.1, 0.0), track(100U, 0.1, 0.0)});

  ASSERT_EQ(second.size(), 2U);
  const auto second_low = std::find_if(
    second.begin(), second.end(),
    [first_low](const auto & item) { return item.track_id == first_low->track_id; });
  const auto second_high = std::find_if(
    second.begin(), second.end(),
    [first_high](const auto & item) { return item.track_id == first_high->track_id; });
  ASSERT_NE(second_low, second.end());
  ASSERT_NE(second_high, second.end());
  EXPECT_LT(second_low->x_m, 1.0);
  EXPECT_GT(second_high->x_m, 4.0);
}

TEST(MultiSensorFusionUnit, PreservesFusedIdAsSourcesAppearAndDisappear)
{
  MultiSensorFusion fusion;
  const auto lidar_only = fusion.fuse(1.0, {}, {track(30U, 1.0, 0.0)});
  ASSERT_EQ(lidar_only.size(), 1U);
  const auto fused_id = lidar_only.front().track_id;

  const auto paired = fusion.fuse(1.1, {track(40U, 1.1, 0.0)}, {track(30U, 1.0, 0.0)});
  ASSERT_EQ(paired.size(), 1U);
  EXPECT_EQ(paired.front().track_id, fused_id);

  const auto camera_only = fusion.fuse(1.2, {track(40U, 1.2, 0.0)}, {});
  ASSERT_EQ(camera_only.size(), 1U);
  EXPECT_EQ(camera_only.front().track_id, fused_id);
}

TEST(MultiSensorFusionUnit, ExpiresIdentityBeforeReassociatingAReappearingTrack)
{
  auto config = FusionConfig{};
  config.max_identity_age_s = 0.5;
  MultiSensorFusion fusion(config);
  const auto first = fusion.fuse(1.0, {track(5U, 0.0, 0.0)}, {});
  ASSERT_EQ(first.size(), 1U);

  EXPECT_TRUE(fusion.fuse(1.6, {}, {}).empty());
  const auto replacement = fusion.fuse(1.7, {track(5U, 0.0, 0.0)}, {});

  ASSERT_EQ(replacement.size(), 1U);
  EXPECT_NE(replacement.front().track_id, first.front().track_id);
}

TEST(MultiSensorFusionUnit, ReassociatesAChangedSourceIdByPosition)
{
  MultiSensorFusion fusion;
  const auto first = fusion.fuse(1.0, {track(7U, 2.0, 0.0)}, {});
  ASSERT_EQ(first.size(), 1U);

  const auto restarted_source = fusion.fuse(1.1, {track(8U, 2.1, 0.0)}, {});

  ASSERT_EQ(restarted_source.size(), 1U);
  EXPECT_EQ(restarted_source.front().track_id, first.front().track_id);
}

TEST(MultiSensorFusionUnit, OutputCovariancesRemainFiniteSymmetricAndPositiveDefinite)
{
  MultiSensorFusion fusion;
  auto camera = track(1U, 0.0, 0.0);
  camera.position_covariance << 0.04, 0.01, 0.01, 0.09;
  auto lidar = track(2U, 0.1, 0.0);
  lidar.position_covariance << 0.01, -0.002, -0.002, 0.02;

  const auto output = fusion.fuse(1.0, {camera}, {lidar});

  ASSERT_EQ(output.size(), 1U);
  const auto & covariance = output.front().position_covariance;
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
  EXPECT_GT(covariance.determinant(), 0.0);
  EXPECT_GT(covariance(0, 0), 0.0);
  EXPECT_GT(covariance(1, 1), 0.0);
}

TEST(MultiSensorFusionUnit, RejectsInvalidConfigurationTracksDuplicatesAndTimeOrder)
{
  auto invalid_config = FusionConfig{};
  invalid_config.cross_sensor_association_distance_m = 0.0;
  EXPECT_THROW((MultiSensorFusion{invalid_config}), std::invalid_argument);

  MultiSensorFusion fusion;
  auto invalid_track = track(1U, 0.0, 0.0);
  invalid_track.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_track_observation(invalid_track));
  EXPECT_THROW(fusion.fuse(1.0, {invalid_track}, {}), std::invalid_argument);

  const auto duplicate = track(2U, 0.0, 0.0);
  EXPECT_THROW(fusion.fuse(1.0, {duplicate, duplicate}, {}), std::invalid_argument);

  ASSERT_EQ(fusion.fuse(2.0, {track(3U, 0.0, 0.0)}, {}).size(), 1U);
  EXPECT_THROW(fusion.fuse(1.9, {}, {}), std::invalid_argument);
}

}  // namespace
}  // namespace perception_fusion
