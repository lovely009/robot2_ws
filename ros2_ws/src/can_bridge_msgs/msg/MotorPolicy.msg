# 低频运行时策略（ARD-0007 §19，上层 → MCU）。
# 语义（TBD-ROS-03=A）：effort 是唯一动态前馈（在 100Hz MotorCommandArray）；
# 本消息只承载 Enable / MIT 增益 / 软件限幅。不刷新命令新鲜度、不进入 100Hz。
# set_* 标志表达"本次要更新哪些字段"，未置位的字段保持原值。
# enable=false：该电机退出运动授权并立即软停/失能；enable=true：仅为许可，
# 仍需新的完整快照才会恢复运动（ARD §13.3）。
uint16 motor_id
bool enable
bool set_gains
float32 kp
float32 kd
bool set_limits
float32 min_position_rad  # rad（仅 Vaxor 本地软限位；CREB 走 MotorConfig 事务）
float32 max_position_rad  # rad
