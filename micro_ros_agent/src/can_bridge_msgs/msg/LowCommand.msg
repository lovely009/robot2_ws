# 循环面下行：100 Hz 完整命令快照（ARD-0007 §15 的定点化重写）。
#
# 完整快照语义不变：受控集合中**未出现**的电机 = 软停；任何 NaN/Inf → 整条拒收。
# 定点与量程见 WireContract.msg（唯一来源）；越界饱和，不拒收。
#
# 尺寸：count=64 时消息体 484 B，含 ROS CDR 封装 488 B ≤ best-effort 可用约 490 B。本消息永不跨 MTU 分片，
# 这是循环面的硬约束 —— 改字段前先重算尺寸。
# 当前 MCU 已确认的运动租约；ControlLease 握手将它绑定到固件会话。
# v3 用 lease_id 替换同宽 session_id，最大帧仍为 488 B，不增加 MTU 占用。
uint64 lease_id
# 会话内严格前进，由唯一主机桥分配；重复与乱序不得刷新 freshness。
uint32 sequence
# MCU 单调毫秒截止点；到板前已过期的非空意图不得刷新接收 freshness。
uint32 valid_until_tick_ms
# 有效槽位数，必须等于各数组长度。
uint8 count
# 槽位顺序的电机 ID；motor_id ∈ [0,63] = 内核注册表下标（不是自由 CAN ID）。
uint8[<=64] motor_ids
# rad
int16[<=64] position_q
# rad/s
int16[<=64] velocity_q
# N·m，前馈 τ_ff
int16[<=64] effort_q
