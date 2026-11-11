#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -eq 0 ]]; then
  set -- --mjcf "$script_dir/../../ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.xml"
fi
exec "$script_dir/python_zh.sh" -m mujoco.viewer "$@"
