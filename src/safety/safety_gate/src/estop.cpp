#include "safety_gate/estop.hpp"

namespace safety_gate
{

EstopState update_estop(const EstopState & prev, bool engaged, bool reset_requested)
{
  EstopState next = prev;
  if (engaged) {
    // Engaging always latches, even if a reset is asserted in the same tick.
    next.latched = true;
  } else if (reset_requested) {
    next.latched = false;
  }
  return next;
}

double estop_speed_scale(const EstopState & state)
{
  return state.latched ? 0.0 : 1.0;
}

}  // namespace safety_gate
