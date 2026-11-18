# 电机轨迹路点，时间相对轨迹起点。
# 整表时间非递减，同一电机的时间严格递增，可由多台电机共享同一时刻。
# motor_id 必须属于 MotorTrajectory.target_ids，各物理量必须有限。
# 起点、插值和力矩支持由接收入口定义。主机动作播放见 motor_hardware_interface/README.md。
uint32 time_from_start_ms
uint16 motor_id
float32 position     # rad，关节侧
float32 velocity     # rad/s，关节侧（诊断/自检用）
float32 effort       # N·m，关节侧（= 前馈 τ_ff）
