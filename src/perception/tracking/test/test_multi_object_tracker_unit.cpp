#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "perception_tracking/multi_object_tracker.hpp"

namespace perception_tracking
{
namespace
{

Observation observation(double x_m, double y_m, std::string class_id = "person")
{
  return Observation{x_m, y_m, 0.4, 0.1, 0.6, 0.5, 1.7, std::move(class_id), 0.9};
}

TrackerConfig immediate_config()
{
  auto config = TrackerConfig{};
  config.min_confirmed_hits = 1U;
  config.max_unobserved_duration_s = 0.5;
  return config;
}

TEST(MultiObjectTrackerUnit, ConfirmsTrackAndPreservesPersistentIdAndMetadata)
{
  MultiObjectTracker tracker;
  EXPECT_TRUE(tracker.update(0.0, {observation(1.0, 2.0)}).empty());

  const auto tracks = tracker.update(0.1, {observation(1.05, 2.0)});

  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().track_id, 1U);
  EXPECT_EQ(tracks.front().class_id, "person");
  EXPECT_DOUBLE_EQ(tracks.front().class_score, 0.9);
  EXPECT_DOUBLE_EQ(tracks.front().z_m, 0.4);
  EXPECT_DOUBLE_EQ(tracks.front().size_z_m, 1.7);
}

TEST(MultiObjectTrackerUnit, EstimatesVelocityAndMarksMovingTrackDynamic)
{
  auto config = immediate_config();
  config.association_distance_m = 1.5;
  config.max_unobserved_duration_s = 2.0;
  config.dynamic_speed_threshold_mps = 0.2;
  MultiObjectTracker tracker(config);
  tracker.update(0.0, {observation(0.0, 0.0)});

  const auto tracks = tracker.update(1.0, {observation(1.0, 0.0)});

  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_GT(tracks.front().velocity_x_mps, 0.2);
  EXPECT_NEAR(tracks.front().velocity_y_mps, 0.0, 1e-9);
  EXPECT_TRUE(tracks.front().is_dynamic);
}

TEST(MultiObjectTrackerUnit, MaintainsOneToOneIdsWhenObservationOrderChanges)
{
  auto config = immediate_config();
  config.association_distance_m = 1.0;
  MultiObjectTracker tracker(config);
  const auto first = tracker.update(0.0, {observation(0.0, 0.0), observation(10.0, 0.0)});
  ASSERT_EQ(first.size(), 2U);
  EXPECT_EQ(first[0].track_id, 1U);
  EXPECT_EQ(first[1].track_id, 2U);

  const auto second = tracker.update(0.1, {observation(9.9, 0.0), observation(0.1, 0.0)});

  ASSERT_EQ(second.size(), 2U);
  EXPECT_EQ(second[0].track_id, 1U);
  EXPECT_LT(second[0].x_m, 1.0);
  EXPECT_EQ(second[1].track_id, 2U);
  EXPECT_GT(second[1].x_m, 9.0);
}

TEST(MultiObjectTrackerUnit, CreatesNewIdOutsideAssociationGate)
{
  auto config = immediate_config();
  config.association_distance_m = 0.5;
  MultiObjectTracker tracker(config);
  tracker.update(0.0, {observation(0.0, 0.0)});

  const auto tracks = tracker.update(0.1, {observation(5.0, 0.0)});

  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_EQ(tracks[0].track_id, 1U);
  EXPECT_EQ(tracks[1].track_id, 2U);
}

TEST(MultiObjectTrackerUnit, PredictsThroughShortDropoutAndExpiresBeforeAssociation)
{
  auto config = immediate_config();
  config.association_distance_m = 1.0;
  config.max_unobserved_duration_s = 0.5;
  MultiObjectTracker tracker(config);
  tracker.update(0.0, {observation(0.0, 0.0)});
  const auto observed = tracker.update(0.1, {observation(0.1, 0.0)});
  ASSERT_EQ(observed.size(), 1U);
  const auto original_id = observed.front().track_id;

  const auto predicted = tracker.update(0.4, {});
  ASSERT_EQ(predicted.size(), 1U);
  EXPECT_EQ(predicted.front().track_id, original_id);
  EXPECT_GT(predicted.front().x_m, observed.front().x_m);

  EXPECT_TRUE(tracker.update(0.7, {}).empty());
  const auto replacement = tracker.update(0.8, {observation(0.2, 0.0)});
  ASSERT_EQ(replacement.size(), 1U);
  EXPECT_NE(replacement.front().track_id, original_id);
}

TEST(MultiObjectTrackerUnit, EmptyClassificationDoesNotEraseKnownClass)
{
  MultiObjectTracker tracker(immediate_config());
  tracker.update(0.0, {observation(0.0, 0.0, "person")});

  const auto tracks = tracker.update(0.1, {observation(0.0, 0.0, "")});

  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().class_id, "person");
  EXPECT_DOUBLE_EQ(tracks.front().class_score, 0.9);
}

TEST(MultiObjectTrackerUnit, CovarianceRemainsFiniteSymmetricAndPositiveOnDiagonal)
{
  MultiObjectTracker tracker(immediate_config());
  tracker.update(0.0, {observation(0.0, 0.0)});
  const auto tracks = tracker.update(0.1, {observation(0.1, -0.1)});

  ASSERT_EQ(tracks.size(), 1U);
  const auto & covariance = tracks.front().state_covariance;
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
  for (Eigen::Index index = 0; index < covariance.rows(); ++index) {
    EXPECT_GT(covariance(index, index), 0.0);
  }
}

TEST(MultiObjectTrackerUnit, RejectsInvalidConfigurationObservationAndTimeOrder)
{
  auto bad_config = TrackerConfig{};
  bad_config.association_distance_m = 0.0;
  EXPECT_THROW((MultiObjectTracker{bad_config}), std::invalid_argument);

  MultiObjectTracker tracker(immediate_config());
  auto invalid = observation(0.0, 0.0);
  invalid.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(tracker.update(0.0, {invalid}), std::invalid_argument);

  ASSERT_EQ(tracker.update(1.0, {observation(0.0, 0.0)}).size(), 1U);
  EXPECT_THROW(tracker.update(0.9, {}), std::invalid_argument);
}

}  // namespace
}  // namespace perception_tracking
