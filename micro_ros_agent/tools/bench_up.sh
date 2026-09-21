#!/usr/bin/env bash
# 台架启动：模拟器（成为 ACK 节点）→ 复位控制板。顺序反了 FDCAN1 会 bus-off。
#
#   tools/bench_up.sh                 # position 模式（播轨迹用这个，见下）
#   tools/bench_up.sh effort          # effort 模式（真实 MIT 动力学）
#
# 为什么默认 position：URDF 四关节都是 <limit effort="10.0">，而 MIT 律
# tau=kp*err 在 kp=30、误差 1.57rad 时需要 ~47 N·m，被模型钳到 10 → 肩/肘
# 关节顶不住重力，停在机械下限（实测 m2 停在 -1.5705 ≈ -π/2）。effort 模式
# 要能用得先给轨迹里的 effort 字段填重力前馈，或抬 URDF 的 effort 上限——
# 两者都是模型层的事，未经确认不要改。position 模式不经过力矩，用它做
# 「轨迹播放/全链路功能验证」，与模拟器自己的 docstring 口径一致。
set -euo pipefail
MODE="${1:-position}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "$MODE" in
  position) SIM_CM=position; GZ_CM=position ;;
  effort)   SIM_CM=effort;   GZ_CM=effort ;;
  *) echo "用法: $0 [position|effort]"; exit 2 ;;
esac

if pgrep -f fdcan_motor_sim >/dev/null; then
  echo "已有模拟器在跑——它会和新的实例抢 CANable（各自下发 SLCAN 配置、互相打断）。"
  echo "先停掉它（模拟器现在有 flock 独占锁，第二个实例会直接失败）。"; exit 1
fi

echo "[1/2] 起 Gazebo（control_mode:=$GZ_CM）与模拟器（--gazebo-control-mode $SIM_CM）"
ros2 launch robot_arm_gazebo gazebo.launch.py control_mode:="$GZ_CM" &
sleep 3
python3 -u "$root/tools/fdcan_motor_sim.py" \
  --data-bitrate-index 7 --gazebo --gazebo-control-mode "$SIM_CM" &
echo "      等通道就绪（会自动 V 探测并选 Y 索引）..."
sleep 6

echo "[2/2] 复位控制板（必须在通道打开之后）"
~/.platformio/packages/tool-openocd/bin/openocd \
  -s ~/.platformio/packages/tool-openocd/openocd/scripts \
  -f interface/stlink.cfg -c 'transport select swd' -f target/stm32h7x.cfg \
  -c 'init' -c 'reset run' -c 'shutdown' >/dev/null 2>&1 || {
    echo "  OpenOCD 复位失败——检查 ST-LINK 是否还在 USB 总线上（lsusb | grep 0483）"; }
sleep 12
echo "就绪。验证: ros2 topic echo /micro_ros_up/motor_topology --once"
echo "播轨迹:  python3 $root/tools/traj_play.py --json <示教JSON>"
wait
