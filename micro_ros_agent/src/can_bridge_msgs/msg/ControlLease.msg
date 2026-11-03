# 主机控制授权的 MCU 本地截止时间；不是使能命令。
# lease_id = session_id*1_000_000 + generation，generation 范围 1..999999。
# 分区不可跨越，乘法溢出或代数耗尽须拒绝授权并建立新会话。
# 新租约必须严格大于 MCU 保留的高水位；到期/撤销后的同一 id 不可复活。
# sequence 在一个 lease_id 内严格递增，不允许回绕。
# active=true 的截止时间须位于 MCU 当前 tick 之后 1..300 ms（uint32模时钟）。
# bridge 不得给过期源消息重新计算期限，也不得自行续租。
uint64 session_id
uint64 lease_id
uint32 sequence
uint32 valid_until_tick_ms
bool active
