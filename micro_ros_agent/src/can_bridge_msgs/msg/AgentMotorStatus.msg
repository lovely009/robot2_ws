# 不含 position/velocity/effort；电气单位沿用现有 wire_codec。
uint8 motor_id
uint8 lifecycle
uint8 execution_state
bool online
bool policy_enabled
bool kernel_enabled
uint32 policy_generation
uint16 unified_fault_code
uint32 vendor_fault_code
uint16 protection_flags
uint16 limit_flags
int16 temperature_dc
int32 current_ma
uint32 voltage_mv
uint8 drive_mode
uint8 status_flags
uint8 bus_index
