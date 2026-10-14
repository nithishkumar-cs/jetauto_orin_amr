# Offline scan-health interlock — 2026-10-13

## Outcome

The previous [offline chain check](offline_safety_chain_2026-10-13.md)
showed Nav2 Humble's collision monitor forwards nonzero commands after
`/scan` becomes stale. A separate `scan_health_gate` now sits between the
collision monitor and the unchanged e-stop latch:

```text
/cmd_vel -> collision_monitor -> /cmd_vel/collision_limited
         -> scan_health_gate -> /cmd_vel/scan_health_limited
         -> estop_gate -> /cmd_vel/safety_limited -> future driver
```

The gate starts closed. A scan opens it only if it has a nonempty frame,
nonzero recent timestamp, plausible scan bounds, and at least one finite
in-range return. It checks both the ROS header's age and elapsed steady
time since local receipt against the configured 0.3 s timeout. A malformed,
old-stamped, or absent scan closes it. While closed, it replaces each
incoming command with zero and also publishes zero at 20 Hz without needing
a new command. A new usable scan permits later commands again. The e-stop
remains the final latch and still requires an explicit reset after a press.

## Evidence

The localhost-only integration harness ran the installed four-node chain
with synthetic `LaserScan`, `Twist`, and e-stop messages. The collision
monitor was confirmed lifecycle `active [3]` before commands were sent.

| Input | Collision-monitor output | Final output |
| --- | ---: | ---: |
| No scan since startup, 0.4 m/s command | 0.4 | **0.0** |
| Fresh clear scan, 0.4 m/s command | 0.4 | 0.4 |
| Five 0.9 m returns, 0.4 m/s command | 0.14 | 0.14 |
| Five 0.4 m returns, 0.4 m/s command | 0.0 | 0.0 |
| E-stop latched, fresh scan, 0.4 m/s command | 0.4 | 0.0 |
| Scan expires, no new command | — | **0.0 published** |
| Stale scan, 0.4 m/s command | 0.4 | **0.0** |
| Newly received scan stamped 1 s old | 0.4 | **0.0** |
| Newly received scan stamped 1 s ahead | 0.4 | **0.0** |
| Fresh scan after timeout | 0.4 | 0.4 |
| Scan with only NaN ranges | 0.4 | **0.0** |
| E-stop latched across scan recovery | 0.4 | 0.0 |
| E-stop reset after scan recovery | 0.4 | 0.4 |

The complete harness passed all 17 printed checks and exited 0. The new
scan-health package's two ROS component tests passed, covering startup,
timer-driven stop, old/invalid scans, and recovery. The test uses
`base_link` as the synthetic scan frame and a static `odom -> base_link`
transform; it does not exercise a real moving transform or a LiDAR mount.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install \
  --packages-select scan_health_gate robot_bringup
source install/setup.bash
python3 tests/integration_tests/check_safety_chain.py
ROS_LOG_DIR=/tmp ROS_LOCALHOST_ONLY=1 colcon test --merge-install \
  --packages-select scan_health_gate estop_gate robot_bringup
```

The harness chooses an isolated high ROS domain, forces localhost DDS,
starts no controller or driver, and stops its child processes afterward.

## Remaining gates

This is a supervisory software interlock, **not a safety-rated stop**. It
does not prove that collision monitor transformed or used a scan, detect
plausible-but-wrong ranges, guarantee a bound on DDS/executor latency, or
measure stopping distance. The eventual base driver needs its own
command-loss watchdog: a healthy scan with a dead command publisher would
otherwise leave the last motor setpoint to driver behavior. Keep motion
control disabled until live Isaac timing, TF, restart/failure behavior,
driver timeout, and braking margins are validated. Ethernet and hardware
were not used in this check.

The one-finite-return rule is intentionally conservative: a legitimate
all-infinity scan will hold zero. The 0.3 s timeout and 0.1 s future-stamp
tolerance are starting values, not measured transport guarantees; check
them against the actual LiDAR and live Isaac clock behavior before motion.

The later [recorded Isaac scan/TF replay](offline_isaac_safety_replay_2026-10-13.md)
tested moving-TF loss and extended the gate to hold zero when the scan cannot
be transformed through `odom` to the current base frame.
