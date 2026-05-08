uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 STALE=3

std_msgs/Header header
string node_name
uint8 status
string message
float32 frequency_hz
builtin_interfaces/Duration age

