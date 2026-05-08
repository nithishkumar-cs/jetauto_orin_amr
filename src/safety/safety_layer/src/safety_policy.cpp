#include "safety_layer/safety_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace safety_layer
{

SafetyDecision evaluate_obstacles(
  const std::vector<amr_interfaces::msg::Obstacle>& obstacles,
  const SafetyConfig& config,
  double input_age_s)
{
  if (input_age_s > config.stale_input_timeout_s) {
    return {
      SafetyCode::SensorDegraded,
      "fused obstacle input is stale",
      -1.0,
      0.0};
  }

  double nearest_forward = std::numeric_limits<double>::infinity();

  for (const auto& obstacle : obstacles) {
    const double x = obstacle.position.x;
    const double y = obstacle.position.y;
    if (x <= 0.0 || std::abs(y) > config.lateral_half_width_m) {
      continue;
    }
    nearest_forward = std::min(nearest_forward, x);
  }

  if (!std::isfinite(nearest_forward)) {
    return {SafetyCode::Clear, "clear", -1.0, 1.0};
  }

  if (nearest_forward <= config.stop_distance_m) {
    return {SafetyCode::Stop, "obstacle inside stop zone", nearest_forward, 0.0};
  }

  if (nearest_forward <= config.slow_distance_m) {
    return {SafetyCode::Slow, "obstacle inside slow zone", nearest_forward, config.slow_scale};
  }

  return {SafetyCode::Clear, "clear", nearest_forward, 1.0};
}

}  // namespace safety_layer

