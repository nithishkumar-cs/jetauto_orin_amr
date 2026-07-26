// Latching e-stop for the JetAuto AMR.
//
// SCOPE: this package used to hold the whole software safety policy — radial
// obstacle zones, multi-source combination, staleness. All of that now belongs
// to nav2_collision_monitor, which does it better (arbitrary polygons rather
// than circles, scan/pointcloud/range sources, built-in source_timeout, and it
// is maintained upstream). What collision_monitor does NOT have is any e-stop
// input: its only inputs are observation sources, cmd_vel, and a footprint
// topic. That gap is the entire reason this package still exists.
//
// The chain is:
//
//   /cmd_vel --> [safety_gate: e-stop latch] --> /cmd_vel_raw
//            --> [nav2_collision_monitor: zones] --> /cmd_vel/safety_limited
//            --> [driver]
//
// HONEST LABEL: this is collision avoidance and an operator stop, running on
// Linux over DDS. It is NOT a safety-rated function. A certified AMR stop is a
// scanner meeting IEC 61496 Type 3 driving a safety relay in hardware, with no
// computer in the path (ISO 3691-4). JetAuto has no such hardware, so nothing
// here should be described as a safety guarantee.
//
// The latch is kept ROS-free for one reason only — it makes the tests instant
// and fixture-free. That is a testing convenience, not a safety property.

#ifndef SAFETY_GATE__ESTOP_HPP_
#define SAFETY_GATE__ESTOP_HPP_

namespace safety_gate
{

/// What the latch remembers between ticks. The caller owns it; nothing here
/// holds state or reads a clock, so the same inputs always give the same result.
struct EstopState
{
  bool latched{false};
};

/// Advance the latch by one tick.
///
///   engaged=true          always latches, regardless of reset_requested.
///   reset_requested=true  clears the latch ONLY while engaged is false.
///
/// The second rule is the point of the whole thing: a reset held down at the
/// same time as the button must not clear the trip, and a momentary drop in the
/// e-stop signal — dropped packet, bouncing contact, publisher restart — must
/// not let the robot move again on its own.
EstopState update_estop(const EstopState & prev, bool engaged, bool reset_requested);

/// Velocity multiplier to apply to cmd_vel: 0.0 while latched, 1.0 otherwise.
double estop_speed_scale(const EstopState & state);

}  // namespace safety_gate

#endif  // SAFETY_GATE__ESTOP_HPP_
