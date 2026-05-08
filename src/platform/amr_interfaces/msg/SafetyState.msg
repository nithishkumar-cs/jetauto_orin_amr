uint8 CLEAR=0
uint8 SLOW=1
uint8 STOP=2
uint8 LOCALIZATION_LOST=3
uint8 SENSOR_DEGRADED=4

std_msgs/Header header
uint8 state
string reason
float32 nearest_obstacle_m
builtin_interfaces/Duration input_age

