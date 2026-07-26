// Pure safety policy core for the JetAuto AMR.
//
// This header defines the *decision logic* of the safety layer with ZERO ROS
// dependencies: plain structs and free functions over STL types. That is
// deliberate — the policy is the part that must be trustworthy, so it is kept
// small, deterministic, and unit-testable in isolation (no node, no message
// bus, no clock). The ROS node is a thin adapter that feeds this core.
//
// The policy DOES have state — latched e-stop, hysteresis — but it never HOLDS
// it. step() takes the previous PolicyState and returns the next one, so the
// caller owns the memory and the policy stays a pure function: same inputs plus
// same previous state always produce the same result. Determinism comes from
// having no *hidden* state, not from having no state. Time enters the same way:
// the node measures message ages and passes them in as plain numbers, so the
// staleness rule is decided here (and tested here) without a clock.
//
// Geometry model: OMNIDIRECTIONAL radial zones, measured from the base frame
// origin. JetAuto is a 4-wheel mecanum (holonomic) base — it strafes and
// rotates in place — so danger cannot be modelled as a forward-only corridor.
// An obstacle at (x=0, y=0.5) is exactly as dangerous as one at (x=0.5, y=0).
// This is the STOP-bubble half of the eventual hybrid design (radial bubble +
// direction-aware slow corridor); the corridor is added once perception and the
// lidar driver exist to feed it.

#ifndef SAFETY_GATE__POLICY_HPP_
#define SAFETY_GATE__POLICY_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace safety_gate
{

/// Severity levels, ordered by escalating severity so that a plain `>`
/// comparison picks the correct headline. Values match
/// amr_interfaces/SafetyState constants EXACTLY, so static_cast<uint8_t>(level)
/// is the on-the-wire state value — one ordering serves both the wire contract
/// and the priority ranking.
///
/// Motion effect is deliberately NOT encoded in this order: SENSOR_DEGRADED,
/// STOP and ESTOP all zero the speed_scale. The order only decides which reason
/// is reported when several conditions hold at once.
enum class SafetyLevel : uint8_t {
  CLEAR = 0,            ///< nothing in range; full speed
  SLOW = 1,             ///< obstacle in the slow zone; throttle to slow_scale
  SENSOR_DEGRADED = 2,  ///< a required safety input is stale/untrusted; fail-closed
  STOP = 3,             ///< obstacle in the stop zone; zero velocity
  ESTOP = 4,            ///< e-stop engaged; zero velocity
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
  double speed_scale{1.0};  ///< 1.0 = full speed, 0.0 = full stop
  std::string reason{"clear"};
  double nearest_obstacle_m{-1.0};  ///< closest sample distance, or -1.0 if none
};

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
///   - level/reason := the verdict with the highest SafetyLevel
///   - nearest_obstacle_m := min over verdicts that reported one (>= 0)
/// An empty list is a misconfiguration and fails closed to SENSOR_DEGRADED.
Decision combine(const std::vector<Decision> & decisions);

// --- Stateful policy --------------------------------------------------------
//
// The functions above are memoryless: each answers one question about one
// instant. Three safety behaviours cannot be expressed that way, because they
// are defined over time:
//
//   * A latched e-stop. A momentary drop in the e-stop signal — dropped packet,
//     bouncing contact, publisher restart — must NOT let the robot move again.
//     Clearing a trip requires a deliberate human reset.
//   * Hysteresis. An obstacle hovering on a zone boundary would otherwise
//     chatter STOP/SLOW at sensor rate and lurch the base.
//   * Staleness. "The lidar has not spoken in 0.4 s" is not computable from a
//     single reading.
//
// step() adds those without giving the policy a clock or a member variable: the
// caller passes the previous PolicyState in and gets the next one back, and the
// node measures message ages and passes them in as plain numbers.

/// One sensor's contribution for this tick. The node builds one of these per
/// configured source EVERY tick, even when nothing arrived — that is how a
/// silent sensor stays visible (it shows up with a large age_s) instead of
/// simply vanishing from the vector and being mistaken for "all clear".
struct SourceReading
{
  std::string name;             ///< stable id, used in reason strings
  std::vector<Point2D> points;  ///< obstacle samples, base frame, metres
  double age_s{0.0};            ///< seconds since this source last published
  bool required{true};          ///< if true, staleness fails closed
};

/// Everything observed this tick.
struct Inputs
{
  std::vector<SourceReading> sources;
  bool estop_engaged{false};    ///< e-stop signal right now
  bool reset_requested{false};  ///< deliberate operator reset of a latched trip
};

/// Tuning. Separate from Inputs because these come from config, not sensors.
struct Params
{
  RadialZones zones;
  double max_source_age_s{0.3};  ///< older than this and a source is untrusted
  double hysteresis_m{0.05};     ///< extra clearance required to de-escalate

  /// Sources that MUST be present and fresh. A name listed here but absent from
  /// Inputs::sources fails closed — this is what makes "the lidar node never
  /// started" a stop rather than an unnoticed blind spot.
  std::vector<std::string> required_sources;
};

/// What the policy remembers between ticks. The caller owns it.
struct PolicyState
{
  bool estop_latched{false};

  /// Zone verdict from the previous tick, used for hysteresis. Deliberately the
  /// ZONE level only (CLEAR/SLOW/STOP), never the combined headline — otherwise
  /// one stale frame reporting SENSOR_DEGRADED would erase the memory of an
  /// obstacle that is still sitting in the stop zone.
  SafetyLevel last_zone_level{SafetyLevel::CLEAR};
};

/// A decision plus the state to feed back on the next tick.
struct Step
{
  Decision decision;
  PolicyState next;
};

/// Full stateful evaluation. Deterministic: the same Inputs and the same
/// previous PolicyState always produce the same Step.
///
/// Order of business:
///   1. Update the e-stop latch. Engaged latches it; reset clears it, but only
///      while the signal is no longer engaged (you cannot reset a held button).
///   2. Evaluate every fresh source against hysteresis-widened zones.
///      Stale required sources become SENSOR_DEGRADED; stale optional sources
///      are ignored.
///   3. Fail closed for any required source missing from Inputs entirely.
///   4. combine() everything, worst case wins.
Step step(const Inputs & in, const PolicyState & prev, const Params & params);

}  // namespace safety_gate

#endif  // SAFETY_GATE__POLICY_HPP_
