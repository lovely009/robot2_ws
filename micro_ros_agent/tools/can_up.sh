#!/usr/bin/env bash
# 把 gs_usb/CANable 的 SocketCAN 接口（默认 can0）配到台架位时序并 up。
#
#   tools/can_up.sh                 # 默认 can0：设好并 up（幂等，重复执行安全）
#   tools/can_up.sh --check         # 只比对当前配置，不一致退出 1（一个字节都不动）
#   tools/can_up.sh can1
#   tools/can_up.sh --check can1
#
# 位时序必须与板子冻结标定逐位一致：仲裁 1 Mbps / 80%，数据 5 Mbps / **81.25%**。
# 数据段不是 80%——160 MHz 下 5 Mbps 只有 32 tq 一种分法，80% 要 25.6 tq，整数取不到。
# 适配器自身不存位时序，每次插上（含 USB 重新枚举）都要重配一遍。理由与实测见
# docs/guides/fdcan1-bench-guide.md 的「整链间歇静默」一节。
#
# 标定值**只在这里定义一次**：--check 与施加共用同一组变量。假电机
# （fdcan_motor_sim.py 的 --interface）靠调用本脚本来保证前置条件，不另写一份。
set -euo pipefail

ARB_BITRATE=1000000
ARB_SAMPLE=0.8
DATA_BITRATE=5000000
DATA_SAMPLE=0.8125
TXQUEUE=200

CHECK=0
if [ "${1:-}" = "--check" ]; then CHECK=1; shift; fi
IFACE="${1:-can0}"

if ! ip -j -d link show "$IFACE" >/dev/null 2>&1; then
  echo "没有 $IFACE 这个接口——gs_usb/CANable 插上了吗？（lsusb | grep 1d50:606f）" >&2
  exit 1
fi

if [ "$CHECK" -eq 1 ]; then
  # 实际值从 ip -j 的 JSON 取，不解析人看的文本。sample_point 会被 iproute2 舍入到
  # 三位小数（0.8125 → 0.812），所以那两项按容差比。**不看 state**：总线上还没有
  # ACK 节点时（先起 sim 再复位板子的正常次序）状态本来就不健康，那不是配置错。
  ip -j -d link show "$IFACE" | python3 -c '
import json, sys

keys = ("arb_bitrate", "arb_sample", "data_bitrate", "data_sample", "txqueue")
want = dict(zip(keys, (float(v) for v in sys.argv[1:])))
link = json.load(sys.stdin)[0]
info = link["linkinfo"]["info_data"]
arb = info.get("bittiming") or {}
data = info.get("data_bittiming") or {}
got = {"arb_bitrate": arb.get("bitrate"), "arb_sample": arb.get("sample_point"),
       "data_bitrate": data.get("bitrate"), "data_sample": data.get("sample_point"),
       "txqueue": link.get("txqlen")}

bad = []
for key in keys:
    actual, target = got[key], want[key]
    if actual is None:
        bad.append(f"{key}: 未配置（期望 {target:g}）")
    elif key.endswith("sample"):
        if abs(actual - target) > 0.002:
            bad.append(f"{key}: {actual:g} ≠ {target:g}")
    elif actual != target:
        bad.append(f"{key}: {actual} ≠ {target:g}")
if bad:
    print("；".join(bad))
    sys.exit(1)
print("%s: 配置符合标定" % link["ifname"])
' "$ARB_BITRATE" "$ARB_SAMPLE" "$DATA_BITRATE" "$DATA_SAMPLE" "$TXQUEUE"
  exit 0
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
"${SUDO[@]}" ip link set "$IFACE" type can bitrate "$ARB_BITRATE" \
  sample-point "$ARB_SAMPLE" dbitrate "$DATA_BITRATE" \
  dsample-point "$DATA_SAMPLE" fd on
"${SUDO[@]}" ip link set "$IFACE" txqueuelen "$TXQUEUE"   # gs_usb 默认 qlen 10，负载下溢出发 ENOBUFS
"${SUDO[@]}" ip link set "$IFACE" up

ip -details link show "$IFACE" | head -4
