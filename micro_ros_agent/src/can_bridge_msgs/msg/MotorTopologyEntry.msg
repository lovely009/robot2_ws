# 拓扑条目（ARD-0007 §18.2）：一台设备在拓扑中的身份与状态。
uint16 motor_id
uint8 bus_index           # 0..2（FDCAN1/2/3）
# 协议族（与 firmware adapter 注册顺序对齐）
uint8 PROTOCOL_VAXOR=0
uint8 PROTOCOL_CERB=1
uint8 protocol
string<=31 model_name     # 型号目录名（未绑定 = 空串）
uint32 capability_flags   # MOTOR_PROTOCOL_CAP_*（唯一定义点在协议 adapter 头）
bool online
bool conflict             # CONFIG_CONFLICT 锁存
# 生命周期九态不放这里：它变化快，属「状态」不属「身份」，
# 唯一权威是 MotorStatusTarget.lifecycle（20Hz）。
