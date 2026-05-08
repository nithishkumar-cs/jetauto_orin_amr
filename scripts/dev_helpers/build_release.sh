#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

# ROS setup scripts assume some tracing variables may be unset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

colcon build \
  --merge-install \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
