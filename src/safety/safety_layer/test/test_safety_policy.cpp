#include "safety_layer/safety_policy.hpp"

#include <gtest/gtest.h>

namespace
{

amr_interfaces::msg::Obstacle obstacle(double x, double y)
{
  amr_interfaces::msg::Obstacle obs;
  obs.position.x = x;
  obs.position.y = y;
  return obs;
}

}  // namespace

TEST(SafetyPolicy, ClearWhenNoForwardObstacle)
{
  safety_layer::SafetyConfig config;
  const auto decision = safety_layer::evaluate_obstacles({obstacle(-1.0, 0.0)}, config, 0.01);
  EXPECT_EQ(decision.code, safety_layer::SafetyCode::Clear);
  EXPECT_DOUBLE_EQ(decision.command_scale, 1.0);
}

TEST(SafetyPolicy, SlowWhenObstacleInsideSlowZone)
{
  safety_layer::SafetyConfig config;
  const auto decision = safety_layer::evaluate_obstacles({obstacle(1.0, 0.0)}, config, 0.01);
  EXPECT_EQ(decision.code, safety_layer::SafetyCode::Slow);
  EXPECT_DOUBLE_EQ(decision.command_scale, config.slow_scale);
}

TEST(SafetyPolicy, StopWhenObstacleInsideStopZone)
{
  safety_layer::SafetyConfig config;
  const auto decision = safety_layer::evaluate_obstacles({obstacle(0.3, 0.0)}, config, 0.01);
  EXPECT_EQ(decision.code, safety_layer::SafetyCode::Stop);
  EXPECT_DOUBLE_EQ(decision.command_scale, 0.0);
}

TEST(SafetyPolicy, SensorDegradedWhenInputIsStale)
{
  safety_layer::SafetyConfig config;
  const auto decision = safety_layer::evaluate_obstacles({}, config, 2.0);
  EXPECT_EQ(decision.code, safety_layer::SafetyCode::SensorDegraded);
  EXPECT_DOUBLE_EQ(decision.command_scale, 0.0);
}

