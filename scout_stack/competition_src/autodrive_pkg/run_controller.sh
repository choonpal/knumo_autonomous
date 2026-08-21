#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"

set +u
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
set -u

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ROS 2 Humble must be sourced before running mission control." >&2
  exit 2
fi

export PYTHONPATH="${workspace_dir}/src/autodrive_pkg${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -m autodrive_pkg.control "$@"
