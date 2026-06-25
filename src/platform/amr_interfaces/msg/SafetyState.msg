# Safety layer verdict, published alongside the gated /cmd_vel/safety_limited.

uint8 CLEAR=0
uint8 SLOW=1
uint8 STOP=2
uint8 ESTOP=3
uint8 SENSOR_DEGRADED=4
# LOCALIZATION_LOST intentionally omitted: safety has no localization-health
# input yet. Add the constant back only when safety subscribes to such a signal.

std_msgs/Header header
uint8 state                            # one of the constants above
string reason                          # human-readable explanation
float32 nearest_obstacle_m             # closest in-corridor obstacle; -1.0 if none
builtin_interfaces/Duration input_age  # age of the obstacle input that drove this verdict
