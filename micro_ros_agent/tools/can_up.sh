#!/usr/bin/env bash
# 把 gs_usb/CANable 的 SocketCAN 接口（默认 can0）配到台架位时序并 up。
#
#   tools/can_up.sh          # 默认 can0
#   tools/can_up.sh can1
#
# 位时序必须与板子冻结标定逐位一致：仲裁 1 Mbps / 80%，数据 5 Mbps / **81.25%**。
# 数据段不是 80%——160 MHz 下 5 Mbps 只有 32 tq 一种分法，80% 要 25.6 tq，整数取不到。
# 适配器自身不存位时序，每次插上（含 USB 重新枚举）都要重配一遍。理由与实测见
# docs/guides/fdcan1-bench-guide.md 的「整链间歇静默」一节。
set -euo pipefail

IFACE="${1:-can0}"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "没有 $IFACE 这个接口——gs_usb/CANable 插上了吗？（lsusb | grep 1d50:606f）" >&2
  exit 1
fi

if [ "$(id -u)" -eq 0 ]; then
  SUDO=()
elif [ -x "$HOME/.local/bin/askpass" ]; then
  export PATH="$HOME/.local/bin:$PATH" SUDO_ASKPASS="$HOME/.local/bin/askpass"
  SUDO=(sudo -A)
else
  SUDO=(sudo)
fi

# 位时序只在 DOWN 下可改（UP 时改报 EBUSY），所以无条件先 down：重复执行是安全的。
"${SUDO[@]}" ip link set "$IFACE" down
"${SUDO[@]}" ip link set "$IFACE" type can bitrate 1000000 sample-point 0.8 \
  dbitrate 5000000 dsample-point 0.8125 fd on
"${SUDO[@]}" ip link set "$IFACE" txqueuelen 200   # gs_usb 默认 qlen 10，负载下溢出发 ENOBUFS
"${SUDO[@]}" ip link set "$IFACE" up

ip -details link show "$IFACE" | head -4
