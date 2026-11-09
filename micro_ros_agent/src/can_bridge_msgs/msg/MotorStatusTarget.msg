# 诊断状态元素（ARD-0007 §17.3/§17.4）。
# 公共位标志唯一定义点：firmware lib/motor/include/motor_protocol_adapter.h
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
# MOTOR_CONTROL_FAULT_*：
#    0=NONE 1=协议 2=厂家 3=Profile 4=冲突 5=离线 6=设备参数未初始化(DEV_PARAM)
#    7=策略代数未前进(STALE_GENERATION) 8=方向已翻需重新设零(ZERO_STALE)
uint16 unified_fault_code
uint32 vendor_fault_code   # 厂家故障字原样透传
uint32 protection_flags    # 见头注释（唯一定义点）
uint32 limit_flags         # 见头注释（唯一定义点）
# 设备侧安全事实（只读遥测；与 ParamSetArray 的 FIELD_LIMIT_PROTECT /
# FIELD_DIRECTION 同源——都来自内核的设备影子，那里是**控制入口**，本话题是
# **遥测**）。
# ⚠️ 这里放的是**正向语义**布尔，不是厂家原值：0x2650:00 的 `1 = 关闭` 是个
# 反向陷阱，反转只允许发生在 param_bridge 的 submit_safety_protect 一处
# （见 param_name.h 的注释）。本话题若放原值，页面就会被迫做第二次反转——
# 那正是当初立规矩要避免的。
bool device_protect_known       # false = 本会话尚未核验过该台（值无意义，页面显示 --）
bool device_protect_enabled     # true = 保护生效（看门狗 + 限位保护在位）
bool device_direction_known
uint8 device_direction          # 0 = CW，1 = CCW（原值直通，无反转陷阱）
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
# 物理 FDCAN 总线索引（0..2 = FDCAN1..3）；来自固件发现帧所在总线。
# 附加在末尾以保留现有字段顺序。
uint8 bus_index
