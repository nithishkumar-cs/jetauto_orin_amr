// Unit tests for the latching e-stop. No ROS, no clock, no bus — just
// deterministic logic. These tests are the specification: if you change a
// latching rule, a test should have to change with it.
//
// The obstacle-zone tests that used to live here were deleted along with the
// zone logic; nav2_collision_monitor owns that now and ships its own tests.

#include <gtest/gtest.h>

#include "safety_gate/estop.hpp"

using namespace safety_gate;

TEST(Estop, StartsClear)
{
  const EstopState fresh;
  EXPECT_FALSE(fresh.latched);
  EXPECT_DOUBLE_EQ(estop_speed_scale(fresh), 1.0);
}

TEST(Estop, EngagingLatches)
{
  const auto s = update_estop({}, /*engaged=*/true, /*reset_requested=*/false);
  EXPECT_TRUE(s.latched);
  EXPECT_DOUBLE_EQ(estop_speed_scale(s), 0.0);
}

TEST(Estop, SurvivesSignalRelease)
{
  // The whole point: a momentary drop in the signal — dropped packet, bouncing
  // contact, publisher restart — must not let the robot move again.
  const auto tripped = update_estop({}, true, false);
  const auto released = update_estop(tripped, /*engaged=*/false, /*reset_requested=*/false);
  EXPECT_TRUE(released.latched);
  EXPECT_DOUBLE_EQ(estop_speed_scale(released), 0.0);
}

TEST(Estop, ReleaseAloneNeverClears)
{
  auto s = update_estop({}, true, false);
  for (int i = 0; i < 100; ++i) {
    s = update_estop(s, false, false);
    ASSERT_TRUE(s.latched) << "cleared itself on tick " << i;
  }
}

TEST(Estop, ResetWhileStillEngagedDoesNotClear)
{
  const auto tripped = update_estop({}, true, false);
  const auto held = update_estop(tripped, /*engaged=*/true, /*reset_requested=*/true);
  EXPECT_TRUE(held.latched);
}

TEST(Estop, ResetAfterReleaseClears)
{
  const auto tripped = update_estop({}, true, false);
  const auto cleared = update_estop(tripped, /*engaged=*/false, /*reset_requested=*/true);
  EXPECT_FALSE(cleared.latched);
  EXPECT_DOUBLE_EQ(estop_speed_scale(cleared), 1.0);
}

TEST(Estop, ResetOnAnUntrippedLatchIsHarmless)
{
  const auto s = update_estop({}, false, true);
  EXPECT_FALSE(s.latched);
}

TEST(Estop, EngageWinsOverSimultaneousReset)
{
  // Both asserted in the same tick: engaging must win.
  const auto s = update_estop({}, /*engaged=*/true, /*reset_requested=*/true);
  EXPECT_TRUE(s.latched);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
