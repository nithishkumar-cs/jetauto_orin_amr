
#include "safety_gate/policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace safety_gate
{

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
    if (d.level > out.level) {
      out = d;  // adopt the higher-priority headline
    }
  }

  out.speed_scale = std::clamp(min_scale, 0.0, 1.0);
  out.nearest_obstacle_m = nearest;
  return out;
}

namespace
{

/// Widen the zone we are currently inside, so leaving a state needs more
/// clearance than entering it did. Entering is never delayed — only the
/// de-escalation is damped, which is the safe direction to be sticky in.
RadialZones widen_for_hysteresis(const RadialZones & z, SafetyLevel last_zone, double hysteresis_m)
{
  const double h = std::max(0.0, hysteresis_m);
  RadialZones e = z;
  if (last_zone == SafetyLevel::STOP) {
    e.stop_radius_m = z.stop_radius_m + h;
    e.slow_radius_m = std::max(z.slow_radius_m, e.stop_radius_m);
  } else if (last_zone == SafetyLevel::SLOW) {
    e.slow_radius_m = z.slow_radius_m + h;
  }
  return e;
}

}  // namespace

Step step(const Inputs & in, const PolicyState & prev, const Params & params)
{
  Step out;
  out.next = prev;

  // 1. E-stop latch. Engaging always latches. A reset only clears the latch
  //    once the signal itself has released — otherwise a reset held at the same
  //    time as the button would defeat the whole point of latching.
  if (in.estop_engaged) {
    out.next.estop_latched = true;
  } else if (in.reset_requested) {
    out.next.estop_latched = false;
  }

  std::vector<Decision> verdicts;
  verdicts.push_back(estop_decision(out.next.estop_latched));

  // 2. Per-source evaluation against hysteresis-widened zones.
  const RadialZones zones =
    widen_for_hysteresis(params.zones, prev.last_zone_level, params.hysteresis_m);

  bool any_source_evaluated = false;
  SafetyLevel worst_zone = SafetyLevel::CLEAR;

  for (const auto & src : in.sources) {
    const bool stale = !(src.age_s <= params.max_source_age_s);  // NaN counts as stale
    if (stale) {
      if (src.required) {
        verdicts.push_back(degraded_decision(src.name + " stale"));
      }
      continue;  // never trust points from a stale source, required or not
    }

    const Decision d = evaluate_obstacles(src.points, zones);
    verdicts.push_back(d);
    any_source_evaluated = true;
    if (d.level > worst_zone) {
      worst_zone = d.level;
    }
  }

  // 3. A required source absent from Inputs entirely. Without this, "the lidar
  //    node never started" is indistinguishable from "the lidar sees nothing".
  for (const auto & name : params.required_sources) {
    const bool present = std::any_of(
      in.sources.begin(), in.sources.end(),
      [&name](const SourceReading & s) { return s.name == name; });
    if (!present) {
      verdicts.push_back(degraded_decision(name + " missing"));
    }
  }

  // 4. Nothing actually looked at the world this tick. estop_decision(false)
  //    contributes speed_scale 1.0, and on its own that would read as
  //    permission to move — but an unpressed e-stop is not evidence the field
  //    is clear. Motion requires at least one source that affirmatively saw.
  if (!any_source_evaluated) {
    verdicts.push_back(degraded_decision("no fresh obstacle source"));
  }

  // 5. Worst case wins.
  out.decision = combine(verdicts);

  // Only refresh the hysteresis memory when a source actually looked. A tick
  // where every source was stale keeps the previous zone memory rather than
  // silently decaying to CLEAR.
  if (any_source_evaluated) {
    out.next.last_zone_level = worst_zone;
  }

  return out;
}

}  // namespace safety_gate
