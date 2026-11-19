# 逐电机策略元素（MotorPolicyArray 的元素）：Enable 与 MIT 增益。
#
# 三者必须一次原子更新：增益要先于使能到位，否则存在一个用旧 Kd 使能的窗口。
# 内核为此提供单一 config kind（SET_POLICY），不做「先 SET_GAINS 再
# SET_ENABLE」的两次调用。
uint16 motor_id
bool enabled
float32 kp
float32 kd
# 设备侧保护确知为关时，每次武装必须显式确认，否则拒绝；CERB 无固件软限位兜底。
# 本位不能替代 policy_generation：后者要求重新武装时提供新意图。
bool confirm_unprotected
