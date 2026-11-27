# 固件原生发布的系统与电机状态快照（Schema 6）。
uint16 SCHEMA_VERSION=6
std_msgs/Header header
uint16 schema_version
uint64 session_id
uint32 snapshot_tick_ms
uint32 sequence
uint32 info_revision
uint32 command_sequence
uint32 stop_generation
uint32 required_policy_generation

# 租约与时钟健康
bool lease_valid
uint64 lease_id
uint32 lease_sequence
uint32 lease_valid_until_tick_ms
bool lease_active
bool clock_valid
uint32 clock_sync_age_ms
uint32 clock_sync_failures
int64 clock_offset_ns

# 逐电机控制态、诊断状态、总线统计与系统电平状态
MotorControlState[<=37] control_states
MotorStatusTarget[<=37] statuses
MotorDiagnostics diagnostics
SystemStatus system_status
