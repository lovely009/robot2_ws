#!/usr/bin/env bash
# init_vcan.sh — 初始化 vcan0 虚拟 CAN 总线（支持 CAN FD, MTU=72）
#
# 用法:
#   bash scripts/init_vcan.sh        (需要 sudo)
#
# 验证:
#   ip -details link show vcan0
set -e

# 1) 加载 vcan 内核模块
if ! lsmod | grep -q "vcan"; then
  sudo modprobe vcan
fi

# 2) 删除可能存在的旧 vcan0
if ip link show vcan0 >/dev/null 2>&1; then
  sudo ip link delete vcan0 >/dev/null 2>&1 || true
fi

# 3) 创建 vcan0 并启用 CAN FD（MTU 72）
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 mtu 72
sudo ip link set vcan0 up

echo "vcan0 created with MTU=72 (CAN FD enabled)."
echo "Verify with: ip -details link show vcan0"
echo "Note: this script requires sudo."
