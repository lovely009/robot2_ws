# 100Hz 完整命令快照（ARD-0007 §15，上层 → MCU）。
# 完整快照语义：受控集合中未出现的电机 = 软停（ARD §13.1）；
# 任何 NaN/Inf → 整快照拒收（TBD-ROS-01=A）；
# sequence：主机调用方日志关联；线上序号由唯一 bridge 在当前固件会话内分配。
# sync_time_ns：上层发送时刻（Unix 纳秒），主机 bridge 校验排队年龄；
# MCU 侧 freshness 一律以本地单调时基（mailbox received tick）为准（ARD §12）。
uint64 sync_time_ns
uint32 sequence
uint8 count
MotorCommandTarget[<=64] targets
# 非空命令必须显式绑定当前 MCU 会话和运动租约；bridge 不代填授权。
uint64 session_id
uint64 lease_id
