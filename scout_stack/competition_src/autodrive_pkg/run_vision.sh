#!/usr/bin/env bash
set -euo pipefail

# Run the source package as a module so its imports remain valid even when
# This works regardless of the integrated workspace directory name.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"
vision_venv="${YOLO_VENV:-${HOME}/yolo11_env}"

if [[ ! -f "${vision_venv}/bin/activate" ]]; then
  echo "YOLO environment not found: ${vision_venv}" >&2
  echo "Set YOLO_VENV to the existing CUDA/TensorRT environment." >&2
  exit 2
fi

set +u
source "${vision_venv}/bin/activate"
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
set -u

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ROS 2 Humble must be sourced before running vision." >&2
  exit 2
fi

export PYTHONPATH="${workspace_dir}/src/autodrive_pkg${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -m autodrive_pkg.vision "$@"
