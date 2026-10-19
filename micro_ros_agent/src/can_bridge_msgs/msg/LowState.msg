# 循环面上行：50 Hz 运动状态（ARD-0007 §16 的定点化重写）。
#
# 只含运动量。state_age_ms 与 target_source 刻意不放这里：带上它们会越过 MTU 而被迫分片，
# 循环面不分片是硬约束（ADR 0015 决定 4）；两者在 MotorStatus 里给。
#
# SENTINEL_MISSING（-32768）= 本拍无有效测量（含从未收到），把「发布频率 ≠ 测量频率」
# 在 50 Hz 上显式化。
uint64 sync_time_ns
# 单调递增；best-effort 无重传，主机据此判丢帧。
uint32 seq
# 有效槽位数，必须等于各数组长度。
uint8 count
# 槽位顺序的电机 ID；motor_id ∈ [0,63] = 内核注册表下标。
uint8[<=64] motor_ids
# rad
int16[<=64] position_q
# rad/s
int16[<=64] velocity_q
# N·m
int16[<=64] effort_q
