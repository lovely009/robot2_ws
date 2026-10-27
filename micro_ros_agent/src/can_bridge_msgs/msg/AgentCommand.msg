# 完整运动快照；有效期是执行截止，STOP_ALL 只允许空载荷。
uint8 SNAPSHOT=0
uint8 STOP_ALL=1
uint64 session_id
uint32 sequence
uint32 valid_until_tick_ms
uint16 schema_version 3
uint8 mode
uint8 count
uint8[<=48] motor_ids
int16[<=48] position_q
int16[<=48] velocity_q
int16[<=48] effort_q
