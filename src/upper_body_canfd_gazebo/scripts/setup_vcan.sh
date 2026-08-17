#!/usr/bin/env bash
set -euo pipefail

# vcan transports CAN / CAN-FD frames through SocketCAN but has no physical bit
# timing. This project logically models 1 Mbps arbitration + 5 Mbps data phase.
sudo modprobe vcan

for interface in vcan0 vcan1; do
  if ! ip link show "${interface}" >/dev/null 2>&1; then
    sudo ip link add dev "${interface}" type vcan
  fi
  sudo ip link set "${interface}" up
done

ip -details link show vcan0
ip -details link show vcan1
