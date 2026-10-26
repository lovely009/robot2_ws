# 低频状态面：10 Hz 合并状态（reliable，允许分片）。
#
# 合并 MotorStatusArray + MotorTrajectoryState + SystemStatus + MotorDiagnostics
# 四条的全部字段；旧消息保留为主机 facade 的类型。
#
# 逐台布尔打包成 status_flags，位语义沿用 MotorStatusTarget.msg 的「正向语义布尔」
# 约定（0x2650 的反转只允许发生在固件 param_bridge 一处）。
#
# 枚举取值不在此定义：target_source 见 MotorMotionStateTarget.msg，lifecycle 见
# MotorStatusTarget.msg，motor_id 缺省见 SystemCommand.msg。
uint64 sync_time_ns
uint64 session_id
uint32 sequence
uint32 tick_ms
# 最近受理的循环命令序号，供主机桥重启时续接。
uint32 command_sequence
# 已受理配置请求号高水位，主机桥重启后从其后续接。
uint32 config_request_id
uint16 schema_version
uint8 count
uint8[<=64] motor_ids

# ---- 逐台：健康 ----
# 0xFFFF = 从未收到运动测量；真实年龄饱和在 0xFFFE（≈65.5 s），不与哨兵共用——
# 共用就会把 65535 ms 静默读成"从未收到"。解码端把 0xFFFF 还原成 0xFFFFFFFF。
uint16[<=64] state_age_ms
# 枚举取值见 MotorMotionStateTarget.msg。
uint8[<=64] target_source
uint8[<=64] lifecycle
uint16[<=64] unified_fault_code
uint32[<=64] vendor_fault_code
# bit0 厂家故障 bit1 过温 bit2 过流 bit3 欠压 bit4 帧解码错误
# （唯一定义点：motor_protocol_adapter.h）
uint16[<=64] protection_flags
# bit0 软限位下限钳制 bit1 软限位上限钳制 bit2 速度钳制 bit3 扭矩超峰钳制
uint16[<=64] limit_flags

# ---- 逐台：电气 ----
# 0.1 ℃；INT16_MIN = 无来源
int16[<=64] temperature_dc
# 电机侧 Iq，mA；INT32_MIN = 缺失
int32[<=64] current_ma
# mV；0xFFFFFFFF = 缺失
uint32[<=64] voltage_mv
# 厂家模式字节透传
uint8[<=64] drive_mode
# bit0 online bit1 device_protect_known bit2 device_protect_enabled
# bit3 device_direction_known bit4 device_direction(0=CW 1=CCW，原值直通)
uint8[<=64] status_flags
uint8[<=64] bus_index

# ---- 轨迹执行状态（全局一份） ----
uint32 trajectory_id
bool traj_active
bool traj_completed
uint32 traj_elapsed_ms
uint32 traj_total_ms
uint16 traj_target_count
uint16[<=64] traj_target_ids

# ---- 系统状态（字段与 SystemStatus.msg 一一对应，解码端直接搬） ----
uint16 last_error
uint16 last_error_motor
uint32 refresh_seq
uint32 set_zero_seq
uint32 clear_fault_seq
uint32 save_seq
bool save_in_progress

# ---- 诊断计数器（单调递增的生命周期累计，上层自己求差分） ----
uint32 rx_frame_count
uint32 unknown_frame_count
uint32 decode_error_count
uint32 snapshot_accepted
uint32 snapshot_rejected
uint32 config_rejected
uint32 event_dropped
uint32 tx_dropped
uint32 p0_retry
uint32 trajectory_accepted
uint32 trajectory_rejected
uint32 policy_accepted
uint32 policy_rejected
uint32 mailbox_rejected_command
uint32 mailbox_rejected_trajectory
uint32 mailbox_rejected_policy
# 出站发布失败总数。调度是否真做到「同类一拍至多一条」由它证伪，不靠推断。
uint32 publish_fail
# 本会话可靠发布调用最大耗时，微秒。
uint32 publish_max_us
uint32[<=3] ring_drop
uint32[<=3] rx_hw_lost
uint32[<=3] tx_frame_count
uint32 heap_free
