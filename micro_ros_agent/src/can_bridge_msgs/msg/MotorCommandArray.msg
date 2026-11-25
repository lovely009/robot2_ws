# 主机电机命令数组，物理量使用 SI 单位。
# 完整快照语义：受控集合中未出现的电机 = 软停；
# 任意物理量含 NaN/Inf 时拒绝整条快照。
# sequence：主机调用方日志关联。
# header.stamp：上层发送时刻（标准 ROS 2 Header 时间戳）。
std_msgs/Header header
uint32 sequence
uint8 count
MotorCommandTarget[<=37] targets
# 非空命令必须显式绑定当前 MCU 会话和运动租约。
uint64 session_id
uint64 lease_id
