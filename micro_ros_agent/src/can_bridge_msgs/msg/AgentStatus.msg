# 最新可靠完整状态；配置终态保留至 ACK，不作为历史事件队列。
uint64 session_id
uint64 snapshot_stamp
uint32 snapshot_tick_ms
uint32 sequence
uint16 schema_version 3
uint32 info_revision
uint32 command_sequence
uint32 config_request_high_water
uint32 trajectory_request_high_water
uint32 stop_generation
uint32 required_policy_generation
bool lease_valid
bool clock_valid
uint32 clock_sync_age_ms
uint32 clock_sync_failures
int64 clock_offset_ns
bool save_in_progress
uint16 last_error
uint16 last_error_motor
AgentMotorStatus[<=48] motors
AgentConfigResult[<=8] results
AgentConfigResult rejection_receipt
AgentTrajectoryReceipt trajectory_request_receipt
AgentTrajectoryState trajectory
MotorDiagnostics diagnostics

# 固件配置 topic 回读，顺序见 firmware_setting_names；仅有效标记置位才可用。
bool firmware_values_valid
uint16[6] firmware_values
bool firmware_unsaved
