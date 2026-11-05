# Nav2 Humble collision monitor backport

This package starts from upstream Navigation2 Humble 1.1.20 commit
`3c3db59d6969d8ecee8e68468693d006397f4a0c` (Apache-2.0). It backports
the later Nav2 behavior that stops when an enabled observation source has no
usable data. The original Humble monitor ignored stale data and could forward
raw velocity with no collision points.

Each source reports whether its consumed message is current and transformable.
For a positive `source_timeout`, an invalid enabled source makes the monitor
publish zero before evaluating polygons. A zero timeout preserves the upstream
option to disable source-loss stopping. Nav2 still owns all stop/slow polygons.

Build with `colcon build --packages-select nav2_collision_monitor --merge-install
--symlink-install`, then source this workspace's `install/setup.bash` before
bringup. The component tests cover repeated reverse commands on an expired
scan and resume after a fresh clear scan. The recorded challenge-5 full-chain
replay yielded no nonzero final commands after this backport; live Isaac testing
is still pending.
