
#include "safety_layer/policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace safety_layer
{

int report_rank(SafetyLevel level)
{
  switch (level) {
    case SafetyLevel::CLEAR:
      return 0;
    case SafetyLevel::SLOW:
      return 1;
    case SafetyLevel::SENSOR_DEGRADED:
      return 2;
    case SafetyLevel::STOP:
      return 3;
    case SafetyLevel::ESTOP:
      return 4;
  }
  return 0;  // unreachable; keeps the compiler quiet
}

Decision evaluate_obstacles(const std::vector<Point2D> & points, const RadialZones & zones)
{
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    nearest = std::min(nearest, std::hypot(p.x, p.y));  // radial: direction-agnostic
  }

  Decision d;
  if (std::isinf(nearest)) {
    // Fresh reading, nothing in range. NOT the same as "no data" (that is the
    // watchdog's job upstream) — here we affirmatively saw a clear field.
    d.level = SafetyLevel::CLEAR;
    d.speed_scale = 1.0;
    d.reason = "clear";
    d.nearest_obstacle_m = -1.0;
    return d;
  }

  d.nearest_obstacle_m = nearest;
  if (nearest <= zones.stop_radius_m) {
    d.level = SafetyLevel::STOP;
    d.speed_scale = 0.0;
    d.reason = "obstacle in stop zone";
  } else if (nearest <= zones.slow_radius_m) {
    d.level = SafetyLevel::SLOW;
    d.speed_scale = std::clamp(zones.slow_scale, 0.0, 1.0);
    d.reason = "obstacle in slow zone";
  } else {
    d.level = SafetyLevel::CLEAR;
    d.speed_scale = 1.0;
    d.reason = "clear";
  }
  return d;
}

Decision estop_decision(bool engaged)
{
  if (engaged) {
    return {SafetyLevel::ESTOP, 0.0, "e-stop engaged", -1.0};
  }
  return {SafetyLevel::CLEAR, 1.0, "e-stop clear", -1.0};
}

Decision degraded_decision(const std::string & reason)
{
  return {SafetyLevel::SENSOR_DEGRADED, 0.0, reason, -1.0};
}

Decision combine(const std::vector<Decision> & decisions)
{
  if (decisions.empty()) {
    // No inputs at all is a misconfiguration; never assume the field is clear.
    return {SafetyLevel::SENSOR_DEGRADED, 0.0, "no safety inputs", -1.0};
  }

  Decision out = decisions.front();  // wins the headline unless out-ranked
  double min_scale = decisions.front().speed_scale;
  double nearest = -1.0;

  for (const auto & d : decisions) {
    min_scale = std::min(min_scale, d.speed_scale);
    if (d.nearest_obstacle_m >= 0.0) {
      nearest = (nearest < 0.0) ? d.nearest_obstacle_m : std::min(nearest, d.nearest_obstacle_m);
    }
    if (report_rank(d.level) > report_rank(out.level)) {
      out = d;  // adopt the higher-priority headline
    }
  }

  out.speed_scale = std::clamp(min_scale, 0.0, 1.0);
  out.nearest_obstacle_m = nearest;
  return out;
}

}  // namespace safety_layer
