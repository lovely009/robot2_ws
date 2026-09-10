# 全量设备拓扑（ARD-0007 §18）：发现窗口完成后整包发布，拓扑变化后刷新。
# 低频；事件级的即时变化走 MotorEvent。
uint64 sync_time_ns
MotorTopologyEntry[<=64] entries
