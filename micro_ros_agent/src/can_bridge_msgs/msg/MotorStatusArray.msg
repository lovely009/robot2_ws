# 诊断状态（ARD-0007 §17，MCU → 上层，低频）。
# 不与运动目标耦合；缺省浮点 = NaN。
uint64 sync_time_ns
MotorStatusTarget[<=64] statuses
