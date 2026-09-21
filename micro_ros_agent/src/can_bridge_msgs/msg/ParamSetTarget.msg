# 单个参数写目标（上层 → MCU，稀疏）。见 ParamSetArray 的头注释。
#
# field 取值 = param_device_slot_t（firmware lib/param_store/include/param_name.h）：
# 与 DeviceParamEntry 的行序、mismatch_mask / record_dirty_slots 的位序、
# MotorEvent 的 DEV_INIT detail(1u<<slot) **同一套**——读写同序、可比。
#
# ⚠️ 它**不是** param_field_t 的次序（那里 3 = pos_min、5 = vel_limit）。
# 两套里 field=3 都合法 → 互用**不会**被拒绝，会**静默写错槽位**。改这里必须
# 同步改 firmware src/param_bridge.cpp 的 kFieldToParamField[]（有 static_assert）。
#
# 单位与 DeviceParamEntry 的 rec_* 两行完全一致（命令行/页面同一口径）：
#   pos_min / pos_max  rad      vel_limit  rad/s
#   torque_limit       A（设备对象 0x2538 存的就是电流，力矩列按 A 呈现）
#   limit_protect      正向语义 1 = 保护生效（反转只在固件 submit_safety_protect 一处）
#   direction          原值直通 0 = CW，1 = CCW（无 0x2650 那种"1 = 关"的反向陷阱）
uint16 motor_id
# 目标槽位，取值见下（= param_device_slot_t）
uint8 FIELD_POS_MIN=0
uint8 FIELD_POS_MAX=1
uint8 FIELD_VEL_LIMIT=2
uint8 FIELD_TORQUE_LIMIT=3
uint8 FIELD_LIMIT_PROTECT=4
uint8 FIELD_DIRECTION=5
uint8 field
float32 value
