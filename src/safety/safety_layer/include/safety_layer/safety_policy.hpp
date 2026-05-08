#pragma once

#include <amr_interfaces/msg/obstacle.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace safety_layer
{

enum class SafetyCode : uint8_t
{
  Clear = 0,
  Slow = 1,
  Stop = 2,
  LocalizationLost = 3,
  SensorDegraded = 4
};

struct SafetyConfig
{
  double stop_distance_m{0.65};
  double slow_distance_m{1.50};
  double lateral_half_width_m{0.45};
  double stale_input_timeout_s{0.50};
  double slow_scale{0.35};
};

struct SafetyDecision
{
  SafetyCode code{SafetyCode::Clear};
  std::string reason{"clear"};
  double nearest_obstacle_m{-1.0};
  double command_scale{1.0};
};

SafetyDecision evaluate_obstacles(
  const std::vector<amr_interfaces::msg::Obstacle>& obstacles,
  const SafetyConfig& config,
  double input_age_s);

}  // namespace safety_layer

