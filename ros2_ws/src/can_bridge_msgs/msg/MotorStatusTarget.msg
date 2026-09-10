# 诊断状态元素（ARD-0007 §17.3/§17.4）。
# 公共位标志唯一定义点：firmware lib/motor_protocol/include/motor_protocol_adapter.h
# （TBD-STATUS-01 关闭前为临时占位分配，跨协议语义稳定）：
#   protection_flags: bit0=厂家故障 bit1=过温(预留) bit2=过流(预留)
#                     bit3=欠压(预留) bit4=帧解码错误
#   limit_flags:      bit0=软限位下限钳制 bit1=软限位上限钳制
#                     bit2=速度钳制(预留) bit3=扭矩超峰钳制
uint16 motor_id
float32 temperature        # °C，无来源 = NaN
float32 current            # A，电机侧 Iq（缺失=NaN）
float32 voltage            # V（缺失=NaN）
uint8 drive_mode           # 厂家模式字节透传
uint16 unified_fault_code  # MOTOR_CONTROL_FAULT_*（0=NONE 1=协议 2=厂家 3=Profile 4=冲突 5=离线）
uint32 vendor_fault_code   # 厂家故障字原样透传
uint32 protection_flags    # 见头注释（唯一定义点）
uint32 limit_flags         # 见头注释（唯一定义点）
bool online
# 生命周期（ARD §9 九态，与 motor_control_lifecycle_t 对齐）
uint8 LIFECYCLE_ABSENT=0
uint8 LIFECYCLE_DISCOVERED=1
uint8 LIFECYCLE_IDENTIFYING=2
uint8 LIFECYCLE_READY_DISABLED=3
uint8 LIFECYCLE_ACTIVE=4
uint8 LIFECYCLE_SAFE_STOP=5
uint8 LIFECYCLE_FAULT=6
uint8 LIFECYCLE_CONFIG_CONFLICT=7
uint8 LIFECYCLE_OFFLINE=8
uint8 lifecycle
uint32 state_age_ms        # 0xFFFFFFFF = 从未收到运动测量
