// Unit tests for the pure safety policy core. No ROS, no clock, no bus —
// just deterministic logic. These tests are the specification: if you change
// a threshold or a combination rule, a test should have to change with it.

#include <gtest/gtest.h>

#include <cmath>

#include "safety_gate/policy.hpp"

using namespace safety_gate;

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

// --- severity ordering ------------------------------------------------------

TEST(SeverityOrder, EscalatesByHeadlinePriority)
{
  // combine() picks the headline with a plain `>`, so this order IS the policy.
  EXPECT_LT(SafetyLevel::CLEAR, SafetyLevel::SLOW);
  EXPECT_LT(SafetyLevel::SLOW, SafetyLevel::SENSOR_DEGRADED);
  EXPECT_LT(SafetyLevel::SENSOR_DEGRADED, SafetyLevel::STOP);
  EXPECT_LT(SafetyLevel::STOP, SafetyLevel::ESTOP);
}

TEST(WireContract, EnumMatchesMessageConstants)
{
  // Guards against silently drifting from amr_interfaces/SafetyState. The same
  // numbers serve as the wire values and the priority ranking, so a change here
  // is a change to both — update SafetyState.msg in lockstep.
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::CLEAR), 0);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::SLOW), 1);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::SENSOR_DEGRADED), 2);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::STOP), 3);
  EXPECT_EQ(static_cast<uint8_t>(SafetyLevel::ESTOP), 4);
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
  const auto d = combine({estop_decision(false), evaluate_obstacles({{1.0, 0.0}}, kZones)});
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
  const auto a = evaluate_obstacles({{1.0, 0.0}}, kZones);  // nearest 1.0
  const auto b = evaluate_obstacles({{0.4, 0.0}}, kZones);  // nearest 0.4
  const auto d = combine({a, b, estop_decision(false)});    // estop has no nearest
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 0.4);
}

TEST(Combine, AllClearStaysClear)
{
  const auto d = combine(
    {estop_decision(false), evaluate_obstacles({{3.0, 0.0}}, kZones),
     evaluate_obstacles({}, kZones)});
  EXPECT_EQ(d.level, SafetyLevel::CLEAR);
  EXPECT_DOUBLE_EQ(d.speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(d.nearest_obstacle_m, 3.0);  // the one source that saw something
}

// --- step: e-stop latching --------------------------------------------------

namespace
{
Params make_params()
{
  Params p;
  p.zones = kZones;
  p.max_source_age_s = 0.3;
  p.hysteresis_m = 0.05;
  p.required_sources = {"lidar"};
  return p;
}

/// A fresh lidar reading with the given points.
Inputs fresh(std::vector<Point2D> points, bool estop = false, bool reset = false)
{
  Inputs in;
  in.sources.push_back({"lidar", std::move(points), 0.0, true});
  in.estop_engaged = estop;
  in.reset_requested = reset;
  return in;
}
}  // namespace

TEST(Step, EstopLatchesAndSurvivesSignalRelease)
{
  const auto p = make_params();

  // Tick 1: button pressed, field otherwise clear.
  const auto a = step(fresh({}, /*estop=*/true), {}, p);
  EXPECT_EQ(a.decision.level, SafetyLevel::ESTOP);
  EXPECT_TRUE(a.next.estop_latched);

  // Tick 2: signal drops (glitch, dropped packet, publisher restart). The
  // memoryless estop_decision() would say CLEAR here — the latch must not.
  const auto b = step(fresh({}, /*estop=*/false), a.next, p);
  EXPECT_EQ(b.decision.level, SafetyLevel::ESTOP);
  EXPECT_DOUBLE_EQ(b.decision.speed_scale, 0.0);
  EXPECT_TRUE(b.next.estop_latched);
}

TEST(Step, ResetClearsLatchOnlyAfterSignalReleases)
{
  const auto p = make_params();
  const auto tripped = step(fresh({}, true), {}, p);

  // Reset while the button is still held must NOT clear the trip.
  const auto held = step(fresh({}, /*estop=*/true, /*reset=*/true), tripped.next, p);
  EXPECT_EQ(held.decision.level, SafetyLevel::ESTOP);
  EXPECT_TRUE(held.next.estop_latched);

  // Released, then reset: now it clears.
  const auto cleared = step(fresh({}, /*estop=*/false, /*reset=*/true), held.next, p);
  EXPECT_EQ(cleared.decision.level, SafetyLevel::CLEAR);
  EXPECT_FALSE(cleared.next.estop_latched);
  EXPECT_DOUBLE_EQ(cleared.decision.speed_scale, 1.0);
}

TEST(Step, ReleaseAloneNeverClearsTheLatch)
{
  const auto p = make_params();
  auto s = step(fresh({}, true), {}, p).next;
  for (int i = 0; i < 100; ++i) {
    const auto r = step(fresh({}, /*estop=*/false), s, p);
    ASSERT_EQ(r.decision.level, SafetyLevel::ESTOP);
    s = r.next;
  }
}

// --- step: staleness --------------------------------------------------------

TEST(Step, StaleRequiredSourceFailsClosed)
{
  const auto p = make_params();
  Inputs in;
  in.sources.push_back({"lidar", {}, /*age_s=*/0.5, /*required=*/true});  // > 0.3

  const auto r = step(in, {}, p);
  EXPECT_EQ(r.decision.level, SafetyLevel::SENSOR_DEGRADED);
  EXPECT_DOUBLE_EQ(r.decision.speed_scale, 0.0);
  EXPECT_EQ(r.decision.reason, "lidar stale");
}

TEST(Step, StaleSourcePointsAreNotTrusted)
{
  // A stale source reporting a clear field must not be read as "all clear".
  auto p = make_params();
  p.required_sources = {};
  Inputs in;
  in.sources.push_back({"camera", {}, /*age_s=*/9.0, /*required=*/false});

  const auto r = step(in, {}, p);
  EXPECT_DOUBLE_EQ(r.decision.speed_scale, 0.0);  // no fresh source => nothing says go
  EXPECT_EQ(r.decision.level, SafetyLevel::SENSOR_DEGRADED);
}

TEST(Step, NanAgeCountsAsStale)
{
  const auto p = make_params();
  Inputs in;
  in.sources.push_back({"lidar", {}, std::nan(""), true});
  EXPECT_EQ(step(in, {}, p).decision.level, SafetyLevel::SENSOR_DEGRADED);
}

TEST(Step, MissingRequiredSourceFailsClosed)
{
  // The lidar node never started: nothing in Inputs at all. The camera is
  // fresh and sees a clear field, which must NOT be enough to move.
  auto p = make_params();
  p.required_sources = {"lidar"};
  Inputs in;
  in.sources.push_back({"camera", {}, 0.0, false});

  const auto r = step(in, {}, p);
  EXPECT_EQ(r.decision.level, SafetyLevel::SENSOR_DEGRADED);
  EXPECT_DOUBLE_EQ(r.decision.speed_scale, 0.0);
  EXPECT_EQ(r.decision.reason, "lidar missing");
}

TEST(Step, FreshRequiredSourceClearsNormally)
{
  const auto p = make_params();
  const auto r = step(fresh({{3.0, 0.0}}), {}, p);
  EXPECT_EQ(r.decision.level, SafetyLevel::CLEAR);
  EXPECT_DOUBLE_EQ(r.decision.speed_scale, 1.0);
}

// --- step: hysteresis -------------------------------------------------------

TEST(Step, StopStateStickyWithinHysteresisBand)
{
  const auto p = make_params();  // stop 0.55, hysteresis 0.05

  const auto in_stop = step(fresh({{0.50, 0.0}}), {}, p);
  ASSERT_EQ(in_stop.decision.level, SafetyLevel::STOP);

  // 0.57 is outside the plain stop radius but inside stop + hysteresis.
  // Memoryless evaluation would drop to SLOW here and start the chatter.
  const auto still = step(fresh({{0.57, 0.0}}), in_stop.next, p);
  EXPECT_EQ(still.decision.level, SafetyLevel::STOP);

  // Beyond the widened radius it finally de-escalates.
  const auto released = step(fresh({{0.65, 0.0}}), still.next, p);
  EXPECT_EQ(released.decision.level, SafetyLevel::SLOW);
}

TEST(Step, EscalationIsNeverDelayed)
{
  const auto p = make_params();
  const auto clear = step(fresh({{3.0, 0.0}}), {}, p);
  ASSERT_EQ(clear.decision.level, SafetyLevel::CLEAR);

  // Entering the stop zone takes effect on the very first tick.
  const auto entered = step(fresh({{0.30, 0.0}}), clear.next, p);
  EXPECT_EQ(entered.decision.level, SafetyLevel::STOP);
}

TEST(Step, StaleTickDoesNotEraseZoneMemory)
{
  const auto p = make_params();
  const auto stopped = step(fresh({{0.50, 0.0}}), {}, p);
  ASSERT_EQ(stopped.next.last_zone_level, SafetyLevel::STOP);

  // A tick where the only source is stale must not decay the memory to CLEAR,
  // or the obstacle would appear to jump out of the stop zone on recovery.
  Inputs stale_in;
  stale_in.sources.push_back({"lidar", {}, 5.0, true});
  const auto gap = step(stale_in, stopped.next, p);
  EXPECT_EQ(gap.next.last_zone_level, SafetyLevel::STOP);

  const auto back = step(fresh({{0.57, 0.0}}), gap.next, p);
  EXPECT_EQ(back.decision.level, SafetyLevel::STOP);  // still sticky
}

// --- step: combination ------------------------------------------------------

TEST(Step, EstopOutranksObstacleHeadline)
{
  const auto p = make_params();
  const auto r = step(fresh({{0.30, 0.0}}, /*estop=*/true), {}, p);
  EXPECT_EQ(r.decision.level, SafetyLevel::ESTOP);
  EXPECT_DOUBLE_EQ(r.decision.speed_scale, 0.0);
}

TEST(Step, NoSourcesAtAllFailsClosed)
{
  auto p = make_params();
  p.required_sources = {};
  const auto r = step(Inputs{}, {}, p);
  EXPECT_DOUBLE_EQ(r.decision.speed_scale, 0.0);
  EXPECT_EQ(r.decision.level, SafetyLevel::SENSOR_DEGRADED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
