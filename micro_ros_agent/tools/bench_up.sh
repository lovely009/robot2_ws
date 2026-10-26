#!/usr/bin/env bash
# 台架启动：模拟器（成为 ACK 节点）→ 复位控制板 → 起 micro_ros_bridge。
# 前两步顺序反了 FDCAN1 会 bus-off；最后一步让 /motor/* 能被编成线上下行。
#
#   tools/bench_up.sh                      # effort 模式（默认，真实 MIT 动力学）
#   tools/bench_up.sh effort <模型名>       # 指定 Gazebo 侧模型（默认 half_body_robot）
#   tools/bench_up.sh position             # position 模式，需要模型带 position 命令接口
#
# 默认 effort：half_body_robot 只导出 effort；同时声明 position/effort 会使
# gz-sim 的 JointForceCmd 覆盖 JointVelocityCmd，导致 position 命令失效。
# position 模式需显式指定带 position 命令接口的模型，否则控制器启动失败。
# 稳定跟踪需匹配重力前馈与 URDF effort 上限；模型层参数未经确认不要改。
set -euo pipefail
MODE="${1:-effort}"
MODEL="${2:-}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "$MODE" in
  position) SIM_CM=position; GZ_CM=position ;;
  effort)   SIM_CM=effort;   GZ_CM=effort ;;
  *) echo "用法: $0 [position|effort] [模型名]"; exit 2 ;;
esac
MODEL_ARG=()
if [ -n "$MODEL" ]; then
  MODEL_ARG=("model:=$MODEL")
fi

if pgrep -f fdcan_motor_sim >/dev/null; then
  echo "已有模拟器在跑——它会和新的实例抢 CANable（各自下发 SLCAN 配置、互相打断）。"
  echo "先停掉它（模拟器现在有 flock 独占锁，第二个实例会直接失败）。"; exit 1
fi

echo "[1/3] 起 Gazebo（control_mode:=$GZ_CM${MODEL:+，model=$MODEL}）与模拟器（--gazebo-control-mode $SIM_CM）"
ros2 launch robot_sim gazebo.launch.py control_mode:="$GZ_CM" "${MODEL_ARG[@]}" &
sleep 3
# 由模拟器通过 V 探测数据位率索引，避免硬编码导致数据相位 ACK 错误。
python3 -u "$root/tools/fdcan_motor_sim.py" \
  --gazebo --gazebo-control-mode "$SIM_CM" &
echo "      等通道就绪（会自动 V 探测并选 Y 索引）..."
sleep 6

echo "[2/3] 复位控制板（必须在通道打开之后）"
~/.platformio/packages/tool-openocd/bin/openocd \
  -s ~/.platformio/packages/tool-openocd/openocd/scripts \
  -f interface/stlink.cfg -c 'transport select swd' -f target/stm32h7x.cfg \
  -c 'init' -c 'reset run' -c 'shutdown' >/dev/null 2>&1 || {
    echo "  OpenOCD 复位失败——检查 ST-LINK 是否还在 USB 总线上（lsusb | grep 0483）"; }
sleep 12

# 下行唯一编码者：traj_play.py / snapshot_stream.py 发的是 /motor/*，
# 没有它命令与轨迹会静默落空。用 robot.launch.py 时 L1 已起，勿重复。
echo "[3/3] 起 micro_ros_bridge（/motor/* → /micro_ros_do/*）"
ros2 run micro_ros_bridge micro_ros_bridge_node &

echo "就绪。验证: ros2 topic echo /motor/topology --once"
echo "播轨迹:  python3 $root/tools/traj_play.py --json <示教JSON>（走 /motor/trajectory，需上面 bridge 在跑）"
wait
