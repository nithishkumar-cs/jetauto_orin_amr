# Offline software command-chain check — 2026-10-13

## Result

The configured ROS 2 command chain works for fresh synthetic LiDAR data,
but **fails the scan-loss stop requirement**. No controller, motor driver,
simulator, physical robot, or Ethernet link participated.

`tests/integration_tests/check_safety_chain.py` launched the real
`nav2_collision_monitor`, `nav2_lifecycle_manager`, and `estop_gate_node`
in an isolated localhost-only ROS domain. It waited for the collision
monitor's lifecycle state to reach `active [3]`, published synthetic
`LaserScan` and `Twist` messages, and observed both command outputs.

| Input condition | `/cmd_vel/collision_limited` | `/cmd_vel/safety_limited` | Result |
| --- | ---: | ---: | --- |
| Clear scan; 0.4 m/s command | 0.4 | 0.4 | Pass-through verified |
| Five returns at 0.9 m | 0.14 | 0.14 | 35% slowdown verified |
| Five returns at 0.4 m | 0.0 | 0.0 | Stop zone verified |
| E-stop press | — | 0.0 immediately | Latch trigger verified |
| E-stop latched; 0.4 m/s command | 0.4 | 0.0 | Final gate verified |
| E-stop reset; 0.4 m/s command | 0.4 | 0.4 | Reset verified |
| Scan older than 0.3 s; 0.4 m/s command | **0.4** | **0.4** | **Failed motion gate** |
| E-stop press after scan loss; 0.4 m/s command | 0.4 | 0.0 | Operator stop still works |

The stale-scan case is deliberately an assertion failure (exit 1) in the
test. It must not be recorded as an all-green safety test. The collision
monitor logged that the source was old (about 0.60 s) and was being ignored.
This is consistent with [Nav2 Humble's collision-monitor processing](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp),
which collects points only from valid sources and otherwise starts from a
pass-through action. Its [source-validity check](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/source.cpp)
returns false when the data age exceeds `source_timeout`; it does not create
a stop action. `stop_pub_timeout` only stops forwarding repeated zero
commands after the configured interval; it does not hold a stop after the
obstacle disappears.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tests/integration_tests/check_safety_chain.py
```

The script forces a high, per-process `ROS_DOMAIN_ID` and
`ROS_LOCALHOST_ONLY=1`; it launches no driver and terminates its three child
processes on exit. The test uses `base_link` as the synthetic scan frame and
a static `odom -> base_link` transform, so it does not validate a real LiDAR
mount or mobile TF path. The 0.55 m stop and 1.25 m slow polygons are still
unmeasured against actual braking distance. No latency distribution or
physical stop distance was measured.

## Decision / next gate

Keep Nav2 motion control disabled. Before enabling a driver, add a separate
fail-closed scan-health command interlock (or an equivalent driver-level
watchdog with a proved contract) and verify no nonzero final command can
pass when scans have never arrived or become stale. Keep the simple
operator e-stop latch separate. A motor-command timeout at the eventual
driver is also needed so a lost command stream cannot leave the last motor
setpoint active. Repeat this chain test under live Isaac timing and then
with a real LiDAR and measured braking margins before any hardware motion.
