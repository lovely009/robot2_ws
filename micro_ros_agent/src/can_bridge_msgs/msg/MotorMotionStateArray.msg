# 运动状态（ARD-0007 §16，MCU → 上层，目标频率 100Hz）。
# 只含运动量；诊断/故障走 MotorStatusArray（高频与低频分离，ARD §14.2）。
uint64 sync_time_ns
MotorMotionStateTarget[<=64] states
# 与线上 low_state.seq 同源：best-effort 无重传，上层据此判丢帧；追加于末尾以保留现有字段顺序。
uint32 seq
