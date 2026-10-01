#!/usr/bin/env bash
set -euo pipefail

# Build a pinned, locally patched SLAM Toolbox into this workspace's overlay.
# The vendor source is ignored by Git; the exact patch and pin are tracked.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
vendor_dir="${repo_root}/src/localization/slam_toolbox_vendor"
patch_file="${repo_root}/patches/slam_toolbox/2.6.10-heading-aware-scan-filter.patch"
upstream_commit="8293d21fe5c816d0405e0cc1f3eb46c70659c267"

if [[ "${ROS_DISTRO:-humble}" != "humble" ]]; then
  echo "This patch is pinned to ROS Humble's SLAM Toolbox 2.6.10." >&2
  exit 1
fi

if [[ ! -e "${vendor_dir}" ]]; then
  git clone --depth 1 --branch 2.6.10 \
    https://github.com/SteveMacenski/slam_toolbox.git "${vendor_dir}"
fi

if [[ ! -d "${vendor_dir}/.git" ]]; then
  echo "Vendor directory exists but is not an upstream Git checkout: ${vendor_dir}" >&2
  exit 1
fi
if [[ "$(git -C "${vendor_dir}" rev-parse HEAD)" != "${upstream_commit}" ]]; then
  echo "Vendor checkout does not match pinned SLAM Toolbox 2.6.10 commit." >&2
  exit 1
fi

if git -C "${vendor_dir}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
  echo "Heading-aware scan patch is already applied."
else
  git -C "${vendor_dir}" apply --check "${patch_file}"
  git -C "${vendor_dir}" apply "${patch_file}"
fi

# ROS setup scripts may refer to unset shell variables.
set +u
source /opt/ros/humble/setup.bash
set -u

cd "${repo_root}"
# A fresh build is memory-heavy on the 8 GB Jetson; keep C++ compilation serial.
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --merge-install --symlink-install \
  --packages-select slam_toolbox \
  --allow-overriding slam_toolbox \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

set +u
source "${repo_root}/install/setup.bash"
set -u
if [[ "$(ros2 pkg prefix slam_toolbox)" != "${repo_root}/install" ]]; then
  echo "Patched SLAM Toolbox did not take precedence in the workspace overlay." >&2
  exit 1
fi

echo "Patched SLAM Toolbox installed in ${repo_root}/install."
echo "Source /opt/ros/humble/setup.bash, then ${repo_root}/install/setup.bash before launching."
