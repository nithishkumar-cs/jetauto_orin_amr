// Unit tests for the pure safety policy core. No ROS, no clock, no bus —
// just deterministic logic. These tests are the specification: if you change
// a threshold or a combination rule, a test should have to change with it.

#include <gtest/gtest.h>

#include "safety_layer/policy.hpp"

using namespace safety_layer;

namespace
{
// Fixed zones used across tests: stop <= 0.55 m, slow <= 1.25 m, 35% throttle.
const RadialZones kZones{0.55, 1.25, 0.35};
}  // namespace

// --- evaluate_obstacles: zone boundaries ------------------------------------

TEST(EvaluateObstacles, EmptyReadingIsClear)
{
  const auto d = evaluate_obstacles({}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::CLEAR);
  EXPECT_DOUBLE_EQ(d.speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, -1.0);  // "-1 = none", not "0 = touching"
}

TEST(EvaluateObstacles, BeyondSlowZoneIsClear)
{
  const auto d = evaluate_obstacles({{2.0, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::CLEAR);
  EXPECT_DOUBLE_EQ(d.speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 2.0);
}

TEST(EvaluateObstacles, InSlowZoneThrottles)
{
  const auto d = evaluate_obstacles({{1.0, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::SLOW);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.35);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 1.0);
}

TEST(EvaluateObstacles, InStopZoneStops)
{
  const auto d = evaluate_obstacles({{0.4, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::STOP);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 0.4);
}

TEST(EvaluateObstacles, StopBoundaryIsInclusive)
{
  // Exactly at stop_radius counts as STOP (<=), not SLOW.
  const auto d = evaluate_obstacles({{0.55, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::STOP);
}

TEST(EvaluateObstacles, SlowBoundaryIsInclusive)
{
  // Exactly at slow_radius counts as SLOW (<=), not CLEAR.
  const auto d = evaluate_obstacles({{1.25, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::SLOW);
}

TEST(EvaluateObstacles, NearestPointWins)
{
  // One far, one near: the nearest drives the verdict.
  const auto d = evaluate_obstacles({{2.0, 0.0}, {0.4, 0.0}, {1.0, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::STOP);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 0.4);
}

// --- evaluate_obstacles: OMNIDIRECTIONAL geometry (the design decision) ------

TEST(EvaluateObstacles, LateralObstacleIsDangerous)
{
  // Pure sideways obstacle (x=0). A forward-corridor model would MISS this;
  // a holonomic base can strafe straight into it, so it must STOP.
  const auto d = evaluate_obstacles({{0.0, 0.4}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::STOP);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 0.4);
}

TEST(EvaluateObstacles, RearObstacleIsDangerous)
{
  // Directly behind (negative x). Mecanum reverses; range is direction-agnostic.
  const auto d = evaluate_obstacles({{-0.4, 0.0}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::STOP);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 0.4);
}

TEST(EvaluateObstacles, DistanceIsEuclidean)
{
  // (0.6, 0.8) -> hypot = 1.0 (SLOW). Manhattan distance would be 1.4 (> slow,
  // i.e. CLEAR), so this point proves the metric is Euclidean, not city-block.
  const auto d = evaluate_obstacles({{0.6, 0.8}}, kZones);
  EXPECT_EQ(d.level, SafetyLevel::SLOW);
  EXPECT_NEAR(d.nearest_obstacle_m, 1.0, 1e-12);
}

// --- estop / degraded -------------------------------------------------------

TEST(EstopDecision, EngagedIsEstop)
{
  const auto d = estop_decision(true);
  EXPECT_EQ(d.level, SafetyLevel::ESTOP);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
}

TEST(EstopDecision, ClearWhenNotEngaged)
{
  const auto d = estop_decision(false);
  EXPECT_EQ(d.level, SafetyLevel::CLEAR);
  EXPECT_DOUBLE_EQ(d.speed_scale, 1.0);
}

TEST(DegradedDecision, AlwaysFailsClosed)
{
  const auto d = degraded_decision("lidar stale");
  EXPECT_EQ(d.level, SafetyLevel::SENSOR_DEGRADED);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
  EXPECT_EQ(d.reason, "lidar stale");
}

// --- report_rank ------------------------------------------------------------

TEST(ReportRank, OrdersByHeadlinePriority)
{
  EXPECT_LT(report_rank(SafetyLevel::CLEAR), report_rank(SafetyLevel::SLOW));
  EXPECT_LT(report_rank(SafetyLevel::SLOW), report_rank(SafetyLevel::SENSOR_DEGRADED));
  EXPECT_LT(report_rank(SafetyLevel::SENSOR_DEGRADED), report_rank(SafetyLevel::STOP));
  EXPECT_LT(report_rank(SafetyLevel::STOP), report_rank(SafetyLevel::ESTOP));
}

TEST(WireContract, EnumMatchesMessageConstants)
{
  // Guards against silently drifting from amr_interfaces/SafetyState.
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::CLEAR), 0);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::SLOW), 1);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::STOP), 2);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::ESTOP), 3);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::SENSOR_DEGRADED), 4);
}

// --- combine: worst-case ----------------------------------------------------

TEST(Combine, EmptyFailsClosed)
{
  const auto d = combine({});
  EXPECT_EQ(d.level, SafetyLevel::SENSOR_DEGRADED);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
}

TEST(Combine, ClearAndSlowYieldsSlow)
{
  const auto d = combine({estop_decision(false),
                          evaluate_obstacles({{1.0, 0.0}}, kZones)});
  EXPECT_EQ(d.level, SafetyLevel::SLOW);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.35);
}

TEST(Combine, SlowAndStopYieldsStop)
{
  const auto slow = evaluate_obstacles({{1.0, 0.0}}, kZones);
  const auto stop = evaluate_obstacles({{0.4, 0.0}}, kZones);
  const auto d = combine({slow, stop});
  EXPECT_EQ(d.level, SafetyLevel::STOP);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
}

TEST(Combine, EstopDominatesStop)
{
  const auto stop = evaluate_obstacles({{0.4, 0.0}}, kZones);
  const auto d = combine({stop, estop_decision(true)});
  EXPECT_EQ(d.level, SafetyLevel::ESTOP);
  EXPECT_EQ(d.reason, "e-stop engaged");
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
}

TEST(Combine, DegradedOverridesSlowThrottle)
{
  // A working sensor sees only a slow-zone obstacle, but another required input
  // is blind: we must fully stop, and the headline is the degradation.
  const auto slow = evaluate_obstacles({{1.0, 0.0}}, kZones);
  const auto d = combine({slow, degraded_decision("camera stale")});
  EXPECT_EQ(d.level, SafetyLevel::SENSOR_DEGRADED);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);  // min(0.35, 0.0)
}

TEST(Combine, StopHeadlinesOverDegraded)
{
  // Both zero the motion; a concrete obstacle is the more actionable headline.
  const auto stop = evaluate_obstacles({{0.4, 0.0}}, kZones);
  const auto d = combine({degraded_decision("camera stale"), stop});
  EXPECT_EQ(d.level, SafetyLevel::STOP);
  EXPECT_DOUBLE_EQ(d.speed_scale, 0.0);
}

TEST(Combine, ReportsGlobalNearest)
{
  // nearest_obstacle_m is the min across all sources that reported one.
  const auto a = evaluate_obstacles({{1.0, 0.0}}, kZones);   // nearest 1.0
  const auto b = evaluate_obstacles({{0.4, 0.0}}, kZones);   // nearest 0.4
  const auto d = combine({a, b, estop_decision(false)});     // estop has no nearest
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 0.4);
}

TEST(Combine, AllClearStaysClear)
{
  const auto d = combine({estop_decision(false),
                          evaluate_obstacles({{3.0, 0.0}}, kZones),
                          evaluate_obstacles({}, kZones)});
  EXPECT_EQ(d.level, SafetyLevel::CLEAR);
  EXPECT_DOUBLE_EQ(d.speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 3.0);  // the one source that saw something
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
