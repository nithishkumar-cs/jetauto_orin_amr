# Offline Isaac scan/TF safety replay — 2026-10-13

## Result

Replayed the first 11 simulated seconds of the existing, read-only
`/home/nithish/jetauto_geometry_validation_20261012/localization_bag` through
the lifecycle-managed Nav2 collision monitor, `scan_health_gate`, and
`estop_gate`. The bag supplies `/clock`, `/scan`, `/tf`, and `/tf_static` from
the moving Isaac robot. The harness publishes only synthetic 0.4 m/s commands;
it starts no controller, driver, or simulator process.

The earlier gate checked scan freshness but not TF. Nav2 Humble returns no
scan points when its scan-to-base transform through `odom` fails, so it can
forward a nonzero command while fresh scans are still arriving. The gate now
checks that same transform request before forwarding a command and publishes
zero at its existing 20 Hz timer rate while the transform is unavailable.

| Replay window | Final nonzero outputs | Result |
| --- | ---: | --- |
| Before first scan | 0 | Stopped |
| Recorded scans + moving TF | 28/28 | Command passed |
| Deliberate scan dropout | 0/33 | Stopped |
| Scan recovery | 13/13 | Command passed |
| Deliberate moving-TF dropout, scans still arriving | 0/23 | Stopped |
| TF recovery | 23/23 | Command passed |

During the TF-dropout window, Nav2's `/cmd_vel/collision_limited` had 7/7
nonzero outputs. That confirms the test exercised the fail-open case rather
than merely observing the monitor's own obstacle stop. The test windows avoid
the transition edges; they do not establish a worst-case stop latency. An
additional component regression stops on a scan from an unknown frame and
recovers after its static transform appears. All three selected package test
suites passed with no failures, and the previous synthetic safety harness
also exited 0. Message counts can vary slightly between replay runs because
of DDS callback scheduling.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install --packages-select scan_health_gate
source install/setup.bash
python3 tests/integration_tests/check_isaac_safety_replay.py
python3 tests/integration_tests/check_safety_chain.py
ROS_LOG_DIR=/tmp ROS_LOCALHOST_ONLY=1 colcon test --merge-install \
  --packages-select scan_health_gate estop_gate robot_bringup
```

The replay harness chooses an isolated ROS domain and localhost-only DDS. It
plays the bag's existing `/clock` at 1× simulated time, suppressing `/scan`
from 3.4–5.3 s and moving `/tf` from 6.7–8.5 s. It does not add a second clock
or alter the source bag. All child processes are stopped on exit. The normal
synthetic chain regression also passed after the TF requirement was added.

## Limits

This is a supervisory software interlock, not a safety-rated stop. Its TF
listener is separate from collision monitor's; a successful lookup cannot
prove the monitor consumed the exact same scan. It does not catch plausible
but wrong ranges, all clock faults, DDS/executor delay, or a driver retaining
its last command. Real stopping distance and zone sizes remain unmeasured.
Keep Nav2 motion control off until live timing, the driver command-loss
watchdog, and braking margin are validated.
