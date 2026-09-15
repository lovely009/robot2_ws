# 100Hz 完整命令快照（ARD-0007 §15，上层 → MCU）。
# 完整快照语义：受控集合中未出现的电机 = 软停（ARD §13.1）；
# 任何 NaN/Inf → 整快照拒收（TBD-ROS-01=A）；
# sequence：序号门已移除（2026-09-15），仅作诊断/日志关联，不参与裁决。
# sync_time_ns：上层发送时刻（Unix 纳秒），仅用于跨系统日志对齐；
# MCU 侧 freshness 一律以本地单调时基（mailbox received tick）为准（ARD §12）。
uint64 sync_time_ns
uint32 sequence
uint8 count
MotorCommandTarget[<=64] targets
