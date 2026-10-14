#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
case "${mode}" in
  debug) build_type=Debug ;;
  profile) build_type=RelWithDebInfo ;;
  production) build_type=Release ;;
  *)
    echo "Usage: $0 {debug|profile|production}" >&2
    exit 2
    ;;
esac

ros_distro="${ROS_DISTRO:-humble}"

# ROS setup scripts assume some tracing variables may be unset.
set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u

colcon build \
  --merge-install \
  --symlink-install \
  --cmake-args \
    "-DCMAKE_BUILD_TYPE=${build_type}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
