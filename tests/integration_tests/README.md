# Integration Tests

Cross-package launch and contract tests.

`check_safety_chain.py` runs the installed Nav2 collision monitor,
lifecycle manager, and `estop_gate` with synthetic ROS messages. Source
Humble and the workspace overlay, then run
`python3 tests/integration_tests/check_safety_chain.py`. It currently exits
1 on the documented stale-LiDAR fail-open gap; see
`docs/validation/offline_safety_chain_2026-10-13.md`. It starts no driver.
