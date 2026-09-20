# 逐电机策略元素（MotorPolicyArray 的元素）：Enable 与 MIT 增益。
#
# 三者必须一次原子更新：增益要先于使能到位，否则存在一个用旧 Kd 使能的窗口。
# 内核为此提供单一 config kind（SET_POLICY），不做「先 SET_GAINS 再
# SET_ENABLE」的两次调用。
uint16 motor_id
bool enabled
float32 kp
float32 kd
# 未带本位的武装请求，在该台设备侧保护**确知为关**时一律拒绝（2026-09-20 owner
# 决定）：关保护 = 该关节没有任何限位在执行（CERB 是设备侧限位，固件无软限位
# 兜底），所以每次武装都要一次显式确认。它不是代数门——`policy_generation`
# 承担"未武装 → 武装需新意图"，两者语义不同，不能互相顶替。
bool confirm_unprotected
