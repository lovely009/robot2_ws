# 完整运动快照；有效期是执行截止，STOP_ALL 只允许空载荷。
uint8 SNAPSHOT=0
uint8 STOP_ALL=1
# 显式建立/撤销授权；SNAPSHOT 的 lease_sequence 非零时只续租现有授权。
# 授权续租不改变 valid_until_tick_ms 所限定的运动截止。
uint8 AUTHORITY=2
uint64 session_id
uint32 sequence
uint32 valid_until_tick_ms
uint16 schema_version 4
uint8 mode
uint8 count
uint8[<=48] motor_ids
int16[<=48] position_q
int16[<=48] velocity_q
int16[<=48] effort_q

uint64 lease_id
uint32 lease_sequence
uint32 lease_valid_until_tick_ms
bool lease_active
