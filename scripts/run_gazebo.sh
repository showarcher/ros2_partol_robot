#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_workspace="$(cd "${workspace_dir}/.." && pwd)/robot_rl"
ros_distro="${ROS_DISTRO:-humble}"

for setup_file in \
  "/opt/ros/${ros_distro}/setup.bash" \
  "${base_workspace}/install/setup.bash" \
  "${workspace_dir}/install/setup.bash"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Setup file not found: ${setup_file}" >&2
    exit 1
  fi
  source "${setup_file}"
done

if [[ -f /usr/share/gazebo/setup.sh ]]; then
  source /usr/share/gazebo/setup.sh
fi

exec ros2 launch robot_rl_description gazebo_sim.launch.py "$@"
