# Collision-monitor zone measurement in Isaac — 2026-10-13

## Scope

First measurement of the `collision_monitor.yaml` stop (0.55 m) and slowdown
(1.25 m) squares against a **moving** base. These values were carried over from
the deleted hand-written radial policy and have been flagged unvalidated since
July; until Nav2 closed the control loop earlier today nothing drove the robot,
so they could not be measured at all.

Isaac-only, on the live camera-enabled command-drive scene over direct
Ethernet. No hardware.

## Method

`tests/integration_tests/measure_collision_zones.py` builds a straight
`nav_msgs/Path` from the robot's current map pose and sends it directly to
Nav2's `FollowPath` action.

The path is constructed rather than planned **on purpose**: the global planner
routes around obstacles, so it cannot produce an approach into one. With a
hand-built path the only things that can stop the base are the collision
monitor, the controller's own collision checking, and the goal checker —
and comparing `/cmd_vel` (controller output) against `/cmd_vel/collision_limited`
(post-monitor) identifies which one acted.

The script also recomputes zone occupancy itself: it transforms every scan
return into `base_link` and counts points inside the same squares the monitor
uses, so the measured trigger point can be compared against the configuration.

Two independent bounds protect the simulated base, both checked every scan:
a travel cap, and an abort on measured obstacle proximity. Either engages the
software e-stop. The proximity abort was deliberately made the binding bound
(1.55 m cap vs. an expected ~1.34 m trigger) so that dead reckoning could not
end the run before the monitor had a chance to act.

## Scene

Robot at map `(1.297, 0.019)`, yaw ≈ 0. A static box sat 1.89 m ahead at about
−9°; open space extended 4.5 m straight ahead. Both zones were empty at start
(0 points in each). Requested route 1.80 m, 37 poses.

## Results (single trial)

| Quantity | Measured | Configured |
|---|---|---|
| Slowdown engaged at | 1.288 m nearest return (travel 0.632 m) | 1.25 m half-extent |
| Slowdown ratio | 0.350 (205 samples, min = max = 0.35) | 0.35 |
| Stop engaged at | 0.619 m nearest return (travel 1.343 m) | 0.55 m half-extent |
| Travel after stop engaged | 0.0074 m | — |
| Final separation, base centre to nearest return | 0.613 m | — |
| Peak speed with zones clear | 0.100 m/s at all three chain stages | `desired_linear_vel: 0.10` |
| Controller speed while monitor held zero | 0.085 m/s | — |

Measured separations are radial; the configured values are square half-extents
along x, so the nearest return sits slightly further out than the half-extent
at a non-zero bearing. Both triggers are consistent with the configuration.

The monitor's own log confirms the sequence: repeated `Robot to slowdown for
35.000000 percents due to SlowZone polygon`, then `Robot to stop due to
StopZone polygon`.

**The monitor was unambiguously the layer that stopped the base.** The
controller kept asking for 0.085 m/s while `/cmd_vel/collision_limited`
carried zero. The robot came to rest 0.613 m from the nearest return, i.e.
0.28 m of clearance beyond the 0.33 m footprint radius.

The route also showed something the earlier 0.40 m runs could not: with zones
clear, the full 0.10 m/s reached the simulator. The short routes only ever hit
0.067 m/s because regulated pure pursuit never finished accelerating.

### Controller behaviour when held

`FollowPath` returned status 6 (ABORTED) rather than hanging or retrying
forever. The configured `SimpleProgressChecker` requires 0.10 m of movement
within 15 s, which a base held still cannot meet; that is the probable cause,
but `controller_server`'s log line was not captured to confirm it specifically.

### Recovery gap (verified separately)

With the stop zone occupied, a direct −0.06 m/s reverse command for 4 s
produced **0.0000 m of movement and zero forwarded commands**. The monitor's
stop action zeroes the command regardless of direction; there is no reverse
exemption. The base cannot drive itself out of its own stop zone through the
normal command path.

Further, `/cmd_vel/collision_limited` published **nothing at all** during that
probe, rather than a stream of zeros. `stop_pub_timeout: 2.0` means the monitor
emits zero velocity for 2 s after the last forwarded command and then goes
silent. **Once that window expires, nothing downstream is published, and the
base stays stopped only because the actuator side has its own watchdog.**

## What this does and does not establish

Established: both zones are correctly wired, trigger at their configured
geometry, and act on a moving base; the 35% throttle is exact; the monitor
takes precedence over the controller; and the controller fails safe by
aborting.

**Not established — the original gate is still open.** `CLAUDE.md` asks for the
stop zone to be measured *at max speed* against *real stopping distance*. This
was 0.10 m/s in a simulator. The 7.4 mm of post-trigger travel is Isaac's
response for a near-massless base at ~0.03 m/s and says nothing about how far
the real JetAuto rolls. A 0.55 m stop zone against 7.4 mm of sim coast is a
~78× margin that will not survive contact with real mass, real wheel slip, and
a real control period. **Do not read this report as validating the zone sizes
for hardware.**

Also not established:

- **Single trial.** A repeat is blocked: the robot is now parked with 8 points
  inside its own stop zone and, per the recovery gap above, cannot reverse out.
  Resetting needs Windows-side intervention in the Isaac scene.
- One static box approached head-on from one bearing. No lateral or diagonal
  approach, no mecanum strafe, no moving obstacle, no multiple simultaneous
  obstacles.
- `max_points: 3` was not independently confirmed. At the sample where the
  monitor's output first went to zero this script counted 1 point in the stop
  square; one scan period later it counted 8. At 5.6 Hz wall and 0.067 m/s that
  is ~12 mm of travel between samples, so the discrepancy is sampling skew, not
  a contradiction — but it is not a measurement of the threshold either.
- Simulation ran at 0.59× wall time.

## Consequences for the hardware driver

The silent-after-2 s behaviour makes an actuator-side watchdog **mandatory**,
not optional, for the `base_driver` still to be written under `src/drivers/`.
A driver that holds its last commanded velocity when its input topic goes quiet
would keep driving into the obstacle the monitor just stopped for. The driver
must zero its output on a monotonic-clock command timeout shorter than anything
the base can do damage in, independently of what the safety chain publishes.

The recovery gap also needs a decision: a robot that stops with an obstacle
inside its stop zone currently has no software path out. Options are an
operator override that bypasses the monitor, a smaller reverse-only zone, or
accepting manual recovery. Not decided here.

## Reproducing

```bash
# survey zone occupancy without moving
python3 tests/integration_tests/measure_collision_zones.py \
    --isaac-simulation-only --survey-only

# drive the approach
python3 tests/integration_tests/measure_collision_zones.py \
    --isaac-simulation-only --distance-m 1.80 --travel-cap-m 1.55 --abort-range-m 0.45
```

Requires a `navigation_mode:=follow_path_test` bringup and a live Isaac scene
with clear space ahead. The script latches the software e-stop on exit; publish
`/estop/reset` before the next motion run.

## Next

Measure real stopping distance on hardware at max speed once `base_driver`
exists, and size the stop zone from that with margin. Before then, the cheap
simulator work left is a lateral/diagonal approach and a strafe, which exercise
the square zones' corners where a radial policy and a square polygon differ
most.
