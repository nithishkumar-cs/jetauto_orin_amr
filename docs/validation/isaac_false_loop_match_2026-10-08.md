# Isaac loop-closure mismatch diagnosis — 2026-10-08

## Result

The large position jump in the extended-route replay is caused by an
incorrect relative-pose match. The mapper associated a scan at about
123.55 s with a scan chain from about 88–90 s. A later
[relative-pose audit](isaac_loop_match_relative_audit_2026-10-11.md) measured
the proposed translation error at **2.176 m** against Isaac's exact odometry;
the original observation that the scan centers were 3.23 m apart was not
enough to establish a false match by itself. The match passed Karto's
fine-response test and its
1.5 m *matched-pose* link-distance gate; immediately afterward, map-frame
position error rose from 0.203 m to 1.502 m. This is not evidence of a
successful loop closure or a Nav2-ready map.

The old diagnostic marker was also too strong a label: Karto printed it
after calling `LinkChainToScan`, but that function may silently skip the
edge when its link-distance gate fails. It cannot be counted as a graph edge
without checking that gate. The [earlier extended-route report](isaac_extended_loop_closure_2026-10-05.md)
records 15 such markers over the full bag, **not 15 verified graph links**.

## Focused replay

The first 160 simulated seconds of the unchanged raw Isaac bag at
`/home/nithish/jetauto_loop_closure_route_20261005/bag` were replayed at 1×
with `loop_search_maximum_distance=3.0` and `do_loop_closing=true`. Temporary
instrumentation in the Git-ignored Karto vendor checkout logged candidate
scan IDs, poses, response, the distance gate, and the corrected pose. The
console log is `/tmp/jetauto_loop_trace_20261008_retry/mapper_console.log`;
the 717 captured `/pose` messages are in
`/tmp/jetauto_loop_trace_20261008_retry/pose_bag`. These `/tmp` artifacts
are local and may not survive a restart. The original raw bag was not edited.

| Scan/time | Fine response | Closest old scan | Squared matched distance | Link gate (2.25 m²) |
| --- | ---: | ---: | ---: | --- |
| 555 / 122.75 s | 0.783 | 402 / 90.05 s | 8.901 m² | Failed |
| 556 / 122.95 s | 0.738 | 402 / 90.05 s | 9.062 m² | Failed |
| 557 / 123.15 s | 0.692 | 402 / 90.05 s | 9.256 m² | Failed |
| 558 / 123.35 s | 0.693 | 402 / 90.05 s | 9.369 m² | Failed |
| 559 / 123.55 s | 0.565 | 402 / 90.05 s | 1.385 m² | Passed |

All five responses exceed the wrapper's 0.45 fine-response default. The
first four emitted the old `KARTO_ACCEPTED_LOOP_CLOSURE` marker but could
not link to this chain: `LinkChainToScan` requires squared distance below
`link_scan_maximum_distance²`, or 2.25 m², plus a numerical tolerance.
`CorrectPoses()` was still called. At scan 559, the reported matched pose
placed the scan close enough to scan 402 for the gate to pass. The corrected
sensor X position changed from 1.998 m before that match to 3.289 m after
optimization. The score and gate are algorithmic checks, not ground-truth
validation.

Isaac's `/odom/wheel` puts the old scan at (5.179, 2.454) m and the current
scan at (1.952, 2.486) m, 3.228 m apart. The LiDAR's 0.15 m forward mount
offset does not eliminate this separation. Matching `/pose` to exact Isaac
odometry with at most 0.02 s stamp skew gives:

| Simulated time | Position error |
| ---: | ---: |
| 123.15 s | 0.194 m |
| 123.35 s | 0.203 m |
| 123.55 s | 1.502 m |
| 123.75 s | 1.518 m |

The earlier full replay jumped at 123.35 s, rather than 123.55 s in this
focused retry; the accepted scan and exact time are therefore replay-sensitive.
Both runs show the same failure mode. The same-input no-loop control in the
earlier report did not show the jump at 123.35 s, although it had its own
scan-matching error. Merely disabling loop closing is not a validated fix.

## Diagnostic-probe correction and status

The optional patch at
`patches/slam_toolbox/2.6.10-loop-closure-probe.patch` now prints
`KARTO_LOOP_LINK_GATE` with the nearest candidate, squared distance, limit,
and pass/fail, and renames the old marker to
`KARTO_FINE_ACCEPTED_LOOP_CANDIDATE`. A passed gate means Karto attempts a
link; it does not by itself prove a new edge or a correct match. The patch is
diagnostic only and does not change the normal mapper parameters or matching
logic. It was compile-checked, then removed from the local vendor checkout;
the installed overlay was rebuilt with only the tracked heading fix.

The next validation should distinguish real revisit matches from these
repetitive-scene matches using exact Isaac pose, then evaluate any proposed
matcher/route change against full pose accuracy. Do not enable Nav2 motion
control on this map.

The [2026-10-09 landmark-variant replay](isaac_landmark_loop_replay_2026-10-09.md)
avoided this particular link in that replay. Its earlier claim that all
three later gate passes were false solely because the scan centers were
physically separate was too strong; the subsequent
[relative-pose audit](isaac_loop_match_relative_audit_2026-10-11.md) uses the
actual proposed transform and finds both accurate and inaccurate matches.
