# 轨迹执行状态（MCU → 上层，低频）。
#
# 为什么需要它：轨迹虽由 reliable 话题分片下发，仍可能被本地校验拒绝；没有这条
# 回执就无从判断「我发的计划是被拒了还是在跑」。被拒的细节另见 MotorEvent 的
# TRAJECTORY_REJECTED（带 trajectory_id 与首个非法路点下标）。
#
# 时间语义：elapsed_ms / total_ms 使用 MCU 单调时间。
# V3 固件成功终态为 active=false、completed=true，表示轨迹已结束并软停；
# completed 不代表机械精度验收，也不代表末点保持。旧后端可仍发布 active=true
# 的完成保持态，消费者应先判断 completed，再判断取消。
uint64 sync_time_ns
uint32 trajectory_id
bool active
bool completed
uint32 elapsed_ms
uint32 total_ms
uint16 target_count
uint16[<=64] target_ids
uint64 session_id
uint32 sequence
uint32 tick_ms
