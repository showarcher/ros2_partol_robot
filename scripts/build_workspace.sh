#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_workspace="$(cd "${workspace_dir}/.." && pwd)/robot_rl"

ros_distro="${ROS_DISTRO:-humble}"
system_setup="/opt/ros/${ros_distro}/setup.bash"
base_setup="${base_workspace}/install/setup.bash"

if [[ ! -f "${system_setup}" ]]; then
  echo "ROS setup not found: ${system_setup}" >&2
  exit 1
fi
if [[ ! -f "${base_setup}" ]]; then
  echo "Base workspace is not built: ${base_setup}" >&2
  exit 1
fi

source "${system_setup}"
source "${base_setup}"
cd "${workspace_dir}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
