# 运动状态（ARD-0007 §16，MCU → 上层，目标频率 100Hz）。
# 只含运动量；诊断/故障走 MotorStatusArray（高频与低频分离，ARD §14.2）。
uint64 sync_time_ns
MotorMotionStateTarget[<=64] states
