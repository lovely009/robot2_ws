# 循环面上行：50 Hz 运动状态（ARD-0007 §16 的定点化重写）。
#
# 只含运动量。state_age_ms 与 target_source 刻意**不放这里**：带上它们消息会涨到
# 680 B（改稠密槽位也 528 B），越过 MTU 就要分片，而循环面不分片是硬约束。
# 两者在 MotorStatus 里给（10 Hz 足够）。
#
# SENTINEL_MISSING（-32768）= 本拍无有效测量（含从未收到），与"发布频率 ≠ 测量频率"
# 这件事在 50 Hz 上就显式化。
uint64 sync_time_ns
# 单调递增；best-effort 无重传，主机据此判丢帧。
uint32 seq
uint8 count
uint8[<=64] motor_ids
int16[<=64] position_q
int16[<=64] velocity_q
int16[<=64] effort_q
