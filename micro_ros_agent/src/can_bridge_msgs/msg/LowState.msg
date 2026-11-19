# 同代完整运动快照的有界分组；每组独立小于 best-effort 载荷上限。
# 仅收到同 session/seq 的全部组后才可作为全量快照交付，缺组不得沿用上一代。
uint64 sync_time_ns
uint64 session_id
uint32 seq
uint32 tick_ms
uint16 schema_version
uint8 chunk_index
uint8 chunk_count
uint8 total_count
uint8 count
uint8[<=32] motor_ids
int16[<=32] position_q
int16[<=32] velocity_q
int16[<=32] effort_q
# 测量年龄与本组运动量来自同一 MCU 快照；0xFFFF=从未测量，0xFFFE=饱和。
uint16[<=32] state_age_ms
uint8[<=32] target_source
