// Pure safety policy core for the JetAuto AMR.
//
// This header defines the *decision logic* of the safety layer with ZERO ROS
// dependencies: plain structs and free functions over STL types. That is
// deliberate — the policy is the part that must be trustworthy, so it is kept
// small, deterministic, and unit-testable in isolation (no node, no message
// bus, no clock). The ROS node is a thin adapter that feeds this core.
//
// Geometry model: OMNIDIRECTIONAL radial zones, measured from the base frame
// origin. JetAuto is a 4-wheel mecanum (holonomic) base — it strafes and
// rotates in place — so danger cannot be modelled as a forward-only corridor.
// An obstacle at (x=0, y=0.5) is exactly as dangerous as one at (x=0.5, y=0).
// This is the STOP-bubble half of the eventual hybrid design (radial bubble +
// direction-aware slow corridor); the corridor is added once perception and the
// lidar driver exist to feed it.

#ifndef SAFETY_LAYER__POLICY_HPP_
#define SAFETY_LAYER__POLICY_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace safety_layer
{

/// Severity levels. Values match amr_interfaces/SafetyState constants EXACTLY,
/// so static_cast<uint8_t>(level) is the on-the-wire state value. Do NOT use
/// this numeric order for priority — it is the message contract, not a ranking.
/// Reporting priority is a separate concern, see report_rank().
enum class SafetyLevel : uint8_t
{
  CLEAR = 0,            ///< nothing in range; full speed
  SLOW = 1,            ///< obstacle in the slow zone; throttle to slow_scale
  STOP = 2,            ///< obstacle in the stop zone; zero velocity
  ESTOP = 3,           ///< e-stop engaged; zero velocity
  SENSOR_DEGRADED = 4, ///< a required safety input is stale/untrusted; fail-closed
};

/// An obstacle sample in the base frame (e.g. base_link), metres.
/// Source-agnostic: a lidar return, a projected detection centroid, or a
/// synthetic test point all look identical here.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// Radial zone thresholds, measured from the base frame origin (metres).
/// Precondition for sane behaviour: 0 <= stop_radius_m <= slow_radius_m and
/// slow_scale in [0, 1]. evaluate_obstacles() checks the stop zone first, so a
/// mis-ordered pair degrades safely (everything inside stop_radius is STOP)
/// rather than dangerously, but callers should validate params at startup.
struct RadialZones
{
  double stop_radius_m{0.55};
  double slow_radius_m{1.25};
  double slow_scale{0.35};  ///< velocity multiplier applied while in the slow zone
};

/// A single safety verdict. speed_scale and level are intentionally decoupled:
/// level/reason are the *reported headline*, speed_scale is the *motion effect*.
/// Combining verdicts takes the worst of each independently (see combine()).
struct Decision
{
  SafetyLevel level{SafetyLevel::CLEAR};
  double speed_scale{1.0};        ///< 1.0 = full speed, 0.0 = full stop
  std::string reason{"clear"};
  double nearest_obstacle_m{-1.0}; ///< closest sample distance, or -1.0 if none
};

/// Reporting priority for worst-case combination. Higher wins the headline when
/// several conditions hold at once. This is distinct from SafetyLevel's numeric
/// value (the wire contract). Order rationale:
///   CLEAR < SLOW < SENSOR_DEGRADED < STOP < ESTOP
/// A concrete obstacle in the stop zone (STOP) is a more actionable headline
/// than "blind on some input" (SENSOR_DEGRADED); an operator/hardware e-stop
/// (ESTOP) dominates everything. Motion is unaffected by this order — all three
/// of DEGRADED/STOP/ESTOP zero the speed_scale regardless.
int report_rank(SafetyLevel level);

/// Radial zone check over obstacle samples. Returns CLEAR (and nearest=-1) for
/// an empty-but-FRESH reading — "we looked and saw nothing". Staleness is NOT
/// this function's job; the node's watchdog decides when a *missing* reading
/// becomes SENSOR_DEGRADED. Do not conflate "no points" with "no data".
Decision evaluate_obstacles(const std::vector<Point2D> & points, const RadialZones & zones);

/// E-stop verdict. Engaged => ESTOP (scale 0). Not engaged => CLEAR.
Decision estop_decision(bool engaged);

/// Fail-closed verdict for a stale/untrusted required input. Always scale 0.
Decision degraded_decision(const std::string & reason);

/// Worst-case combination of independent verdicts ("anything screaming danger
/// wins"). Two independent reductions:
///   - speed_scale := min over all verdicts   (most restrictive throttle wins)
///   - level/reason := the verdict with the highest report_rank
///   - nearest_obstacle_m := min over verdicts that reported one (>= 0)
/// An empty list is a misconfiguration and fails closed to SENSOR_DEGRADED.
Decision combine(const std::vector<Decision> & decisions);

}  // namespace safety_layer

#endif  // SAFETY_LAYER__POLICY_HPP_
