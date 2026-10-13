# Integration Tests

Cross-package launch and contract tests.

`check_safety_chain.py` runs the installed Nav2 collision monitor,
lifecycle manager, `scan_health_gate`, and `estop_gate` with synthetic ROS
messages. Source Humble and the workspace overlay, then run
`python3 tests/integration_tests/check_safety_chain.py`. It asserts that
missing, stale, and unusable LiDAR data force zero at the final output;
see `docs/validation/offline_scan_health_interlock_2026-10-13.md`.
It starts no driver.
