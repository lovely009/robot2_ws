# reliable 周期完整快照；事件丢失后仍可恢复，缺席电机不沿用上一帧授权。
uint16 SCHEMA_VERSION=1
uint16 schema_version
uint64 sync_time_ns
# 建链后首帧确定的 epoch 毫秒会话标识；会话内 sequence 单调递增，重连后不可复用旧授权。
uint64 session_id
uint32 sequence
uint32 tick_ms
MotorControlState[<=64] states
