# 单台电机的命令，motor_id 对应设备，物理量使用 SI 单位。
# 任意物理量含 NaN/Inf 时拒绝整条命令，effort 为前馈力矩。
uint16 motor_id
float32 position     # rad，关节侧
float32 velocity     # rad/s，关节侧
float32 effort       # N·m，关节侧（= 前馈 τ_ff）
