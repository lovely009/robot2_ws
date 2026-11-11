#!/usr/bin/env bash
set -euo pipefail
runtime_dir="$HOME/.cache/robot2-mujoco-zh/runtime"
base_python="/home/ly/.ros/ros2_control/.venv/bin/python"
if [[ ! -f "$runtime_dir/build.json" ]]; then
  printf '%s\n' '请先使用 MuJoCo 环境的 Python 运行 tools/mujoco_zh/build.py。' >&2
  exit 1
fi
export MUJOCO_LANGUAGE=zh_CN
export MUJOCO_ZH_FONT="${MUJOCO_ZH_FONT:-/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc}"
if [[ ! -f "$MUJOCO_ZH_FONT" ]]; then
  printf '%s\n' '找不到中文字体，请将 MUJOCO_ZH_FONT 指向中文 TTF/TTC 字体。' >&2
  exit 1
fi
export PYTHONPATH="$runtime_dir${PYTHONPATH:+:$PYTHONPATH}"
exec "$base_python" "$@"
