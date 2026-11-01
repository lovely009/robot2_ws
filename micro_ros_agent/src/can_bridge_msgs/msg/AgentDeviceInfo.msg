# 身份与参数缓存视图；查询本身不访问 CAN/Flash。
uint16 motor_id
uint8 bus_index
uint16 node_id
uint8 protocol
string<=31 model_name
uint32 capability_flags
bool online
bool conflict
uint8 lifecycle
bool record_valid
bool device_valid
uint32 device_read_age_ms
DeviceParamEntry parameters
