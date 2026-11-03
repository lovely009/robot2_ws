# 同代各组来自同一 MCU 快照；接收方不得跨代补组。
uint64 session_id
uint64 snapshot_stamp
uint32 snapshot_tick_ms
uint32 sequence
uint16 schema_version 4
uint8 group_index
uint8 group_count
uint8 count
uint8[<=32] motor_ids
int16[<=32] position_q
int16[<=32] velocity_q
int16[<=32] effort_q
uint16[<=32] state_age_ms
uint8[<=32] target_source
