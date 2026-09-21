# 系统状态（MCU → 上层，5 Hz + 变化即发，best-effort）。
#
# 这是**电平**不是事件：last_error 保持到下一次错误，四个 *_seq 是单调递增的
# 完成计数（上层判完成 = 记下"发命令时读到的值"再等它变大）。丢一帧只损失
# 200 ms，故用 stream QoS（与同为电平小消息的 diagnostics / trajectory_state
# 同判据）；本消息远小于 512 B MTU，不需要 reliable 分片。
#
# 5 Hz 而不是 1 Hz：refresh 本身只要 ~30 ms，1 Hz 会让"发命令 → 轮询 seq"
# 每次多等最多 1 s，完成信号反成瓶颈。
#
# 不放 heap_free：MotorDiagnostics.heap_free 已是同源同值（motor_bridge 填充），
# 两个真相源必然漂移。
# 不放保存进度计数器：保存分批期间每个 CONFIG_RESULT 都即时驱动一帧
# device_param_report，record_dirty 逐台翻转已经是权威进度。
uint64 sync_time_ns
# 最近一次错误。0 = 无；码表与参数面时期逐字一致（零迁移）：
#   1..6  内核故障类；10 本地校验拒绝；11 提交失败；12 未知电机；
#   20 保存失败；30 回读失败；
#   40/41/42 设备参数未初始化（MISMATCH / UNVERIFIED / NO_RECORD）
uint16 last_error
# 相关电机；LAST_ERROR_MOTOR_NONE = 不适用（该错误不指向某一台）
uint16 LAST_ERROR_MOTOR_NONE=65535
uint16 last_error_motor
uint32 refresh_seq
uint32 set_zero_seq
uint32 clear_fault_seq
# 保存完成计数。**新增**：参数面时期 save 只是一个布尔，且在 W25 落盘**之前**
# 就被清掉（见决策日志勘误）。现在 save_in_progress 覆盖「受理 → W25 出终态」
# 整段——它还是 false 就说明数据真的落了盘。
uint32 save_seq
bool save_in_progress
