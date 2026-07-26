#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
profile="${JETAUTO_CHECK_PROFILE:-host}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

# Packages buildable on a plain x86 host (no CUDA/Jetson hardware).
# Trimmed to the post-reset tree; add each package back here as it is rebuilt.
# Keep GPU-only packages (e.g. perception_inference) out of this list.
host_packages=(
  amr_interfaces
  safety_layer
)

build_args=(--merge-install --symlink-install)
test_args=(--merge-install)

case "${profile}" in
  host)
    build_args+=(--packages-select "${host_packages[@]}")
    test_args+=(--packages-select "${host_packages[@]}")
    ;;
  full)
    ;;
  *)
    echo "Unknown JETAUTO_CHECK_PROFILE='${profile}'. Use 'host' or 'full'." >&2
    exit 2
    ;;
esac

colcon list --base-paths src
colcon build "${build_args[@]}"
colcon test "${test_args[@]}" --event-handlers console_cohesion+
colcon test-result --verbose
