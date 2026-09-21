# 轨迹执行状态（MCU → 上层，低频）。
#
# 为什么需要它：轨迹虽由 reliable 话题分片下发，仍可能被本地校验拒绝；没有这条
# 回执就无从判断「我发的计划是被拒了还是在跑」。被拒的细节另见 MotorEvent 的
# TRAJECTORY_REJECTED（带 trajectory_id 与首个非法路点下标）。
#
# 时间语义：elapsed_ms 是 MCU 本地单调时基下相对受理时刻的偏移，与轨迹路点的
# time_from_start_ms 同基准；total_ms 是末点时刻。播完（elapsed >= total_ms）
# 后 active=true、completed=true，归属仍保持且电机在末点保持；发送空轨迹取消后
# 才 active=false（机构无抱闸，软停会掉臂）。
uint64 sync_time_ns
uint32 trajectory_id
bool active
bool completed
uint32 elapsed_ms
uint32 total_ms
uint16 target_count
uint16[<=64] target_ids
