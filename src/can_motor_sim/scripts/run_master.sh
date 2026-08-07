#!/usr/bin/env bash
# run_master.sh — 一键启动 vcan 桥接(后台) + 交互式电机主站(前台)，无需从机
#
# 用法:
#   bash scripts/run_master.sh [A|B] [id]
#   例: bash scripts/run_master.sh A 1      # 协议 A, 设备/控制器 1
#       bash scripts/run_master.sh B 2      # 协议 B, 设备 2
#
# 说明:
#   - 桥接(socket_can_sender/receiver) 在后台启动，日志写入 /tmp/can_bridge.log
#   - 主站在前台运行，可直接输入 enable / set_mit / rp_mit / rp_reg ... 等命令
#   - 主站输入 quit 或 Ctrl-C 退出后，自动清理后台桥接
#   - 另一个终端可随时 `candump -x vcan0` 观察总线上所有帧
#
set -e

PROTOCOL="${1:-A}"
ID="${2:-1}"

# 确保 ROS2 环境已 source（若未 source，则尝试常见路径）
if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1090
  source /opt/ros/humble/setup.bash
fi
if [ -f "${HOME}/robot_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${HOME}/robot_ws/install/setup.bash"
fi

# 清理可能残留的旧 bridge 进程
pkill -f "socket_can_sender_node_exe" 2>/dev/null || true
pkill -f "socket_can_receiver_node_exe" 2>/dev/null || true
sleep 1

echo ">> 后台启动 vcan0 桥接 (interface=vcan0) ..."
ros2 launch can_motor_sim socketcan_bridge.launch.py > /tmp/can_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 4

echo ">> 启动交互式主站 (protocol=${PROTOCOL} id=${ID}) ..."
echo ">> 输入 help 查看命令；另一个终端可 candump -x vcan0 观察总线"
ros2 run can_motor_sim motor_master --protocol "${PROTOCOL}" --id "${ID}" || true

echo ">> 主站已退出，清理后台桥接 ..."
kill "${BRIDGE_PID}" 2>/dev/null || true
pkill -f "socket_can_sender_node_exe" 2>/dev/null || true
pkill -f "socket_can_receiver_node_exe" 2>/dev/null || true
echo ">> 完成"
