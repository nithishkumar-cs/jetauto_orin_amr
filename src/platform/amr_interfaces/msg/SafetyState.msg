# Safety layer verdict, published alongside the gated /cmd_vel/safety_limited.

# Ordered by escalating severity, so a plain numeric comparison picks the
# correct headline when several conditions hold at once: higher value wins.
# A concrete obstacle in the stop zone outranks "blind on some input", and an
# operator/hardware e-stop outranks everything.
#
# Motion effect is NOT encoded here — SENSOR_DEGRADED, STOP and ESTOP all mean
# zero velocity. This ordering only decides which reason gets reported.
#
# Inserting a new state means renumbering everything above it. That is cheap
# today (nothing consumes this message yet) and expensive once bags and nodes
# depend on the values, so settle the set before it ships.
uint8 CLEAR=0
uint8 SLOW=1
uint8 SENSOR_DEGRADED=2
uint8 STOP=3
uint8 ESTOP=4
# LOCALIZATION_LOST intentionally omitted: safety has no localization-health
# input yet. Add the constant back only when safety subscribes to such a signal.

std_msgs/Header header
uint8 state                            # one of the constants above
string reason                          # human-readable explanation
float32 nearest_obstacle_m             # closest in-corridor obstacle; -1.0 if none
builtin_interfaces/Duration input_age  # age of the obstacle input that drove this verdict
