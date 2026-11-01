#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOG_DIR="$PWD/runtime/ros-log"
export ROS_LOCALHOST_ONLY=1
export ROBOT2_SIL_LIBRARY="$PWD/build-host/librobot2_sil.so"
exec python3 local_sil_preview.py
